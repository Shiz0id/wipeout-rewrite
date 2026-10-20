// Headless simulation tests and tuning report for antigrav.
// Exits non-zero if any check fails.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../antigrav.h"
#include "test_world.h"

#define PI 3.14159265358979323846f

static int failures = 0;

#define CHECK(COND, ...) do { \
	if (!(COND)) { \
		failures++; \
		printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); \
	} \
} while (0)

static ag_vec3_t v3(float x, float y, float z) { return (ag_vec3_t){x, y, z}; }
static float len(ag_vec3_t a) { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }

static bool vec_finite(ag_vec3_t a) {
	return isfinite(a.x) && isfinite(a.y) && isfinite(a.z);
}

static ag_world_t *make_world_ex(test_world_t *tw, bool with_route, bool route_walls) {
	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1.0f);
	desc.raycast = test_world_raycast;
	desc.user = tw;
	if (with_route) {
		desc.route = tw->route;
		desc.route_len = tw->route_len;
		desc.route_closed = true;
		desc.route_walls = route_walls;
	}
	return ag_world_create(&desc);
}

static ag_world_t *make_world(test_world_t *tw, bool with_route) {
	return make_world_ex(tw, with_route, false);
}



// Straight line ----------------------------------------------------------------

static void test_acceleration(void) {
	printf("\n== Acceleration & hover (flat ground) ==\n");
	test_world_t tw;
	test_world_make_flat(&tw, false, 0);
	ag_world_t *w = make_world(&tw, false);

	ag_ship_params_t params;
	ag_ship_params_default(&params, 1.0f);
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));

	ag_input_t in = {.thrust = 1};
	ag_ship_set_input(w, ship, &in);

	float t = 0, t100 = -1, t130 = -1;
	float h_min = INFINITY, h_max = -INFINITY, h_sum = 0;
	int h_count = 0;
	const float dt = 1.0f / 60.0f;
	while (t < 20) {
		ag_world_step(w, dt);
		t += dt;
		const ag_ship_state_t *s = ag_ship_state(w, ship);
		if (t100 < 0 && s->speed >= 100) t100 = t;
		if (t130 < 0 && s->speed >= 130) t130 = t;
		if (t > 17) {
			h_min = fminf(h_min, s->ground_height);
			h_max = fmaxf(h_max, s->ground_height);
			h_sum += s->ground_height;
			h_count++;
		}
	}
	const ag_ship_state_t *s = ag_ship_state(w, ship);
	float h_mean = h_sum / h_count;
	printf("  0-100 m/s: %.2fs   0-130 m/s: %.2fs   speed after 20s: %.1f m/s (top_speed %.0f)\n", t100, t130, s->speed, params.top_speed);
	printf("  hover height: mean %.3f  range %.4f (target %.2f)\n", h_mean, h_max - h_min, params.hover_height);

	CHECK(fabsf(s->speed - params.top_speed) < params.top_speed * 0.03f, "top speed %.1f not within 3%% of %.1f", s->speed, params.top_speed);
	CHECK(t100 > 0 && t100 < 5, "0-100 took %.2fs", t100);
	CHECK(fabsf(h_mean - params.hover_height) < 0.1f, "hover height %.3f", h_mean);
	CHECK(h_max - h_min < 0.05f, "hover oscillates by %.4f", h_max - h_min);
	CHECK(s->grounded, "not grounded");

	ag_world_destroy(w);
	test_world_free(&tw);
}


// Turning ----------------------------------------------------------------------

static float heading(ag_vec3_t f) {
	return atan2f(f.x, f.z);
}

static void run_turn(const char *label, float steer, float airbrake) {
	test_world_t tw;
	test_world_make_flat(&tw, false, 0);
	ag_world_t *w = make_world(&tw, false);

	ag_ship_params_t params;
	ag_ship_params_default(&params, 1.0f);
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));
	ag_ship_apply_impulse(w, ship, v3(0, 0, params.top_speed));

	ag_input_t in = {.thrust = 1};
	ag_ship_set_input(w, ship, &in);
	for (int i = 0; i < 60; i++) {
		ag_world_step(w, 1.0f / 60.0f);
	}
	float speed_before = ag_ship_state(w, ship)->speed;

	in.steer = steer;
	in.airbrake_right = airbrake;
	ag_ship_set_input(w, ship, &in);

	float h0 = 0, turned = 0, last = heading(ag_ship_state(w, ship)->velocity);
	float speed_sum = 0, slip_max = 0;
	int n = 0;
	for (int i = 0; i < 240; i++) {
		ag_world_step(w, 1.0f / 60.0f);
		const ag_ship_state_t *s = ag_ship_state(w, ship);
		float h = heading(s->velocity);
		float d = h - last;
		if (d > PI) d -= 2 * PI;
		if (d < -PI) d += 2 * PI;
		last = h;
		if (i >= 120) {
			turned += d;
			speed_sum += s->speed;
			n++;
		}
		float slip = acosf(fminf(1, fabsf((s->velocity.x * s->forward.x + s->velocity.z * s->forward.z) / fmaxf(s->speed, 1e-3f))));
		slip_max = fmaxf(slip_max, slip);
		(void)h0;
	}
	float rate = fabsf(turned) / 2.0f;
	float speed = speed_sum / n;
	printf("  %-28s speed %5.1f -> %5.1f m/s  turn rate %.2f rad/s  radius %6.1f m  max slip %4.1f deg\n",
		label, speed_before, speed, rate, speed / rate, slip_max * 180 / PI);

	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_turning(void) {
	printf("\n== Turning at top speed (flat ground) ==\n");
	run_turn("full steer", 1, 0);
	run_turn("half steer", 0.5f, 0);
	run_turn("full steer + airbrake (drift)", 1, 1);
}


// Walls ------------------------------------------------------------------------

static void run_wall(float angle_deg, float speed, float *retained, bool *tunneled, float *riding) {
	test_world_t tw;
	test_world_make_flat(&tw, true, 0);
	ag_world_t *w = make_world(&tw, false);

	ag_ship_params_t params;
	ag_ship_params_default(&params, 1.0f);
	float a = angle_deg * PI / 180;
	ag_vec3_t fwd = v3(-sinf(a), 0, cosf(a));
	float dist = 8 + speed * 0.05f;
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(dist * sinf(a) + params.half_extents.z, 2, 0), fwd);
	ag_ship_apply_impulse(w, ship, v3(fwd.x * speed, 0, fwd.z * speed));
	ag_input_t in = {.thrust = 1};
	ag_ship_set_input(w, ship, &in);

	float before = speed, after = INFINITY;
	float hit_time = -1, t = 0;
	*tunneled = false;
	*riding = 0;
	while (t < 4 && (hit_time < 0 || t < hit_time + 1.0f)) {
		float speed_prev = ag_ship_state(w, ship)->speed;
		ag_world_step(w, 1.0f / 120.0f);
		t += 1.0f / 120.0f;
		const ag_ship_state_t *s = ag_ship_state(w, ship);
		if (s->position.x < 0) {
			*tunneled = true;
		}
		ag_event_t ev;
		while (ag_world_poll_event(w, &ev)) {
			if (ev.type == AG_EVENT_WALL_HIT && hit_time < 0) {
				hit_time = t;
				before = speed_prev;
			}
		}
		if (hit_time >= 0 && t < hit_time + 0.1f) {
			after = fminf(after, s->speed);
		}
		*riding = s->speed / before;
	}
	*retained = hit_time >= 0 ? after / before : 1;

	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_walls(void) {
	printf("\n== Wall hits at 120 m/s, no steering (speed kept by the impact) ==\n");
	float angles[] = {5, 10, 20, 30, 45, 60, 90};
	float kept[7];
	for (int i = 0; i < 7; i++) {
		bool tunneled;
		float riding;
		run_wall(angles[i], 120, &kept[i], &tunneled, &riding);
		printf("  %4.0f deg: %5.1f%%   (%5.1f%% after riding the wall for 1s)%s\n", angles[i], kept[i] * 100, riding * 100, tunneled ? "  TUNNELED" : "");
		CHECK(!tunneled, "tunneled through the wall at %.0f deg", angles[i]);
	}
	CHECK(kept[0] > 0.93f, "grazing at 5 deg kept only %.1f%%", kept[0] * 100);
	CHECK(kept[1] > 0.85f, "grazing at 10 deg kept only %.1f%%", kept[1] * 100);
	CHECK(kept[6] < 0.5f, "head-on kept %.1f%%", kept[6] * 100);
	for (int i = 1; i < 7; i++) {
		CHECK(kept[i] <= kept[i - 1] + 0.02f, "speed loss not monotonic at %.0f deg", angles[i]);
	}

	float speeds[] = {200, 300};
	for (int i = 0; i < 2; i++) {
		float k, riding;
		bool tunneled;
		run_wall(90, speeds[i], &k, &tunneled, &riding);
		printf("  head-on at %.0f m/s: %s\n", speeds[i], tunneled ? "TUNNELED" : "held");
		CHECK(!tunneled, "tunneled at %.0f m/s", speeds[i]);
	}
}


// Race -------------------------------------------------------------------------

#define RACE_SHIPS 8
#define RACE_LAPS 3

typedef struct {
	float finish_time[RACE_SHIPS];
	int finish_order[RACE_SHIPS];
	float best_lap[RACE_SHIPS];
	float top_speed[RACE_SHIPS];
	float avg_speed[RACE_SHIPS];
	int wall_hits[RACE_SHIPS];
	int hard_wall_hits[RACE_SHIPS];
	int hard_ship_hits[RACE_SHIPS];
	int ship_hits[RACE_SHIPS];
	int out_of_bounds[RACE_SHIPS];
	int boosts[RACE_SHIPS];
	int start_boosts;
	bool finite;
	bool all_finished;
	double wall_seconds;
	int steps;
} race_result_t;

static float frame_dt(int mode, int frame) {
	switch (mode) {
		case 0: return 1.0f / 30.0f;
		case 1: return 1.0f / 60.0f;
		case 2: return 1.0f / 144.0f;
		default: {
			// Jittery frame times between 4 and 40ms
			unsigned h = (unsigned)frame * 2654435761u;
			return 0.004f + (h % 1000) / 1000.0f * 0.036f;
		}
	}
}

static void run_race(test_world_t *tw, int dt_mode, bool with_countdown, bool route_walls, race_result_t *r) {
	memset(r, 0, sizeof(*r));
	r->finite = true;
	ag_world_t *w = make_world_ex(tw, true, route_walls);

	ag_ship_params_t params;
	ag_ship_params_default(&params, 1.0f);
	ag_ship_id_t ships[RACE_SHIPS];
	float distance[RACE_SHIPS] = {0};
	ag_vec3_t last_pos[RACE_SHIPS];

	for (int i = 0; i < RACE_SHIPS; i++) {
		ships[i] = ag_ship_add(w, &params, v3(0, 0, 0), v3(0, 0, 1));
		ag_ship_place_on_grid(w, ships[i], i);
		ag_ai_params_t ai;
		ag_ai_params_default(&ai);
		ai.skill = 0.55f + 0.06f * (RACE_SHIPS - i);
		ai.speed_scale = 0.97f + 0.01f * (RACE_SHIPS - i) * 0.5f;
		ai.line_offset = ((i * 37) % 7 - 3) / 3.0f * 0.5f;
		ag_ship_set_ai(w, ships[i], &ai);
		ag_ship_set_locked(w, ships[i], with_countdown);
		last_pos[i] = ag_ship_state(w, ships[i])->position;
		r->finish_time[i] = -1;
	}

	clock_t clock_start = clock();
	float t = 0, countdown = with_countdown ? 3 : 0;
	int finished = 0, frame = 0;
	while (t < 400 && finished < RACE_SHIPS) {
		float dt = frame_dt(dt_mode, frame++);
		if (countdown > 0) {
			countdown -= dt;
			if (countdown <= 0) {
				for (int i = 0; i < RACE_SHIPS; i++) {
					ag_ship_set_locked(w, ships[i], false);
				}
			}
		}
		r->steps += ag_world_step(w, dt);
		t += dt;

		ag_event_t ev;
		while (ag_world_poll_event(w, &ev)) {
			if (ev.ship < 0 || ev.ship >= RACE_SHIPS) continue;
			switch (ev.type) {
				case AG_EVENT_WALL_HIT:
					r->wall_hits[ev.ship]++;
					r->hard_wall_hits[ev.ship] += ev.intensity > 15;
					break;
				case AG_EVENT_SHIP_HIT:
					r->ship_hits[ev.ship]++;
					r->ship_hits[ev.other]++;
					r->hard_ship_hits[ev.ship] += ev.intensity > 8;
					r->hard_ship_hits[ev.other] += ev.intensity > 8;
					break;
				case AG_EVENT_OUT_OF_BOUNDS: r->out_of_bounds[ev.ship]++; break;
				case AG_EVENT_BOOST_PAD: r->boosts[ev.ship]++; break;
				case AG_EVENT_START_BOOST: r->start_boosts++; break;
				default: break;
			}
		}

		for (int i = 0; i < RACE_SHIPS; i++) {
			const ag_ship_state_t *s = ag_ship_state(w, ships[i]);
			if (!vec_finite(s->position) || !vec_finite(s->velocity)) {
				r->finite = false;
			}
			if (r->finish_time[i] < 0) {
				ag_vec3_t d = {s->position.x - last_pos[i].x, s->position.y - last_pos[i].y, s->position.z - last_pos[i].z};
				distance[i] += len(d);
				r->top_speed[i] = fmaxf(r->top_speed[i], s->speed);
				if (s->lap > RACE_LAPS) {
					r->finish_time[i] = t - (with_countdown ? 3 : 0);
					r->finish_order[finished++] = i;
					r->best_lap[i] = s->best_lap_time;
					r->avg_speed[i] = distance[i] / r->finish_time[i];
				}
			}
			last_pos[i] = s->position;
		}
		if (!r->finite) {
			break;
		}
	}
	r->all_finished = finished == RACE_SHIPS;
	r->wall_seconds = (double)(clock() - clock_start) / CLOCKS_PER_SEC;
	ag_world_destroy(w);
}

static void test_race(void) {
	printf("\n== AI race: %d craft, %d laps on the test circuit ==\n", RACE_SHIPS, RACE_LAPS);
	test_world_t tw;
	test_world_make_circuit(&tw, true);

	ag_world_t *probe = make_world(&tw, true);
	float min_radius = INFINITY;
	for (int i = 0; i < tw.route_len; i++) {
		ag_vec3_t a = tw.route[(i + tw.route_len - 2) % tw.route_len].position;
		ag_vec3_t b = tw.route[i].position;
		ag_vec3_t c = tw.route[(i + 2) % tw.route_len].position;
		float ab = len(v3(b.x - a.x, 0, b.z - a.z)), bc = len(v3(c.x - b.x, 0, c.z - b.z)), ca = len(v3(a.x - c.x, 0, a.z - c.z));
		float area2 = fabsf((b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z));
		if (area2 > 1e-6f) {
			min_radius = fminf(min_radius, ab * bc * ca / (2 * area2));
		}
	}
	printf("  circuit length %.0f m, tightest corner radius %.0f m, %d triangles\n", ag_route_length(probe), min_radius, tw.tris_len);
	ag_world_destroy(probe);

	const char *mode_names[] = {"30 fps", "60 fps", "144 fps", "jitter 4-40ms"};
	race_result_t results[4];
	for (int mode = 0; mode < 4; mode++) {
		run_race(&tw, mode, true, false, &results[mode]);
	}

	race_result_t *r = &results[1];
	printf("\n  60 fps results (hard hits: walls > 15 m/s, craft > 8 m/s):\n");
	for (int k = 0; k < RACE_SHIPS; k++) {
		if (k >= RACE_SHIPS || (!r->all_finished && r->finish_time[r->finish_order[k]] <= 0)) break;
		int i = r->finish_order[k];
		printf("  P%d  #%d  %6.2fs  best lap %6.2fs  avg %5.1f top %5.1f m/s  walls %3d (hard %2d)  craft %3d (hard %2d)  pads %2d  oob %d\n",
			k + 1, i, r->finish_time[i], r->best_lap[i], r->avg_speed[i], r->top_speed[i],
			r->wall_hits[i], r->hard_wall_hits[i], r->ship_hits[i], r->hard_ship_hits[i], r->boosts[i], r->out_of_bounds[i]);
	}
	printf("  start boosts: %d/%d\n", r->start_boosts, RACE_SHIPS);

	printf("\n  Frame rate independence:\n");
	for (int mode = 0; mode < 4; mode++) {
		race_result_t *m = &results[mode];
		printf("  %-14s finished %s  winner #%d in %.3fs  (%d steps, %.2fs cpu, %.0f steps/s)\n",
			mode_names[mode], m->all_finished ? "all" : "NOT ALL", m->finish_order[0], m->finish_time[m->finish_order[0]],
			m->steps, m->wall_seconds, m->steps / fmax(m->wall_seconds, 1e-6));
		CHECK(m->finite, "%s: non-finite state", mode_names[mode]);
		CHECK(m->all_finished, "%s: not all craft finished", mode_names[mode]);
	}

	// With a countdown, results differ by the sub-step phase at which the
	// host unlocks the craft; racing is chaotic, so allow some spread
	for (int mode = 0; mode < 4; mode++) {
		float a = results[mode].finish_time[results[mode].finish_order[0]];
		float b = results[1].finish_time[results[1].finish_order[0]];
		CHECK(fabsf(a - b) < b * 0.02f, "%s: winning time %.2f vs %.2f at 60fps", mode_names[mode], a, b);
	}

	// Without host interaction the fixed steps must give identical lap times
	// (counted in physics steps) at any frame rate
	race_result_t exact[4];
	bool identical = true;
	for (int mode = 0; mode < 4; mode++) {
		run_race(&tw, mode, false, false, &exact[mode]);
	}
	for (int mode = 0; mode < 4; mode++) {
		for (int i = 0; i < RACE_SHIPS; i++) {
			if (exact[mode].best_lap[i] != exact[1].best_lap[i]) {
				identical = false;
			}
		}
	}
	printf("  without countdown, lap times at all frame rates are %s\n", identical ? "bit-identical" : "DIFFERENT");
	CHECK(identical, "lap times differ between frame rates");

	int oob = 0, walls = 0;
	float avg = 0;
	for (int i = 0; i < RACE_SHIPS; i++) {
		oob += r->out_of_bounds[i];
		walls += r->wall_hits[i];
		avg += r->avg_speed[i] / RACE_SHIPS;
	}
	ag_ship_params_t params;
	ag_ship_params_default(&params, 1.0f);
	printf("\n  average race speed %.1f m/s (%.0f%% of top speed), %d wall hits, %d out of bounds\n",
		avg, avg / params.top_speed * 100, walls, oob);
	CHECK(avg > params.top_speed * 0.6f, "AI average speed only %.1f", avg);
	CHECK(oob <= RACE_SHIPS / 2, "%d out of bounds", oob);

	test_world_free(&tw);
}


// Mesh -------------------------------------------------------------------------

static unsigned rng_state = 12345;
static float rnd(void) {
	rng_state = rng_state * 1664525u + 1013904223u;
	return (rng_state >> 8) / 16777216.0f;
}

static void test_mesh(void) {
	printf("\n== Mesh collision: BVH vs. brute force ==\n");
	test_world_t tw;
	test_world_make_circuit(&tw, true);

	int rays = 20000, hits = 0, mismatches = 0;
	clock_t start = clock();
	ag_hit_t hb[1];
	for (int i = 0; i < rays; i++) {
		ag_route_point_t *rp = &tw.route[(int)(rnd() * tw.route_len) % tw.route_len];
		ag_vec3_t o = v3(rp->position.x + (rnd() - 0.5f) * 40, rp->position.y + rnd() * 10, rp->position.z + (rnd() - 0.5f) * 40);
		ag_vec3_t d = v3(rnd() - 0.5f, rnd() - 0.7f, rnd() - 0.5f);
		float l = len(d);
		d = v3(d.x / l, d.y / l, d.z / l);

		ag_hit_t ha;
		bool a = ag_mesh_raycast(tw.mesh, o, d, 60, &ha);
		bool b = test_world_raycast_brute(&tw, o, d, 60, hb);
		hits += a;
		if (a != b || (a && (fabsf(ha.distance - hb->distance) > 1e-3f || ha.surface_flags != hb->surface_flags))) {
			mismatches++;
		}
	}
	double brute_and_bvh = (double)(clock() - start) / CLOCKS_PER_SEC;

	start = clock();
	int bvh_rays = 1000000;
	ag_hit_t h;
	for (int i = 0; i < bvh_rays; i++) {
		ag_route_point_t *rp = &tw.route[i % tw.route_len];
		ag_vec3_t o = v3(rp->position.x, rp->position.y + 2, rp->position.z);
		ag_mesh_raycast(tw.mesh, o, v3(0, -1, 0), 8, &h);
	}
	double bvh_time = (double)(clock() - start) / CLOCKS_PER_SEC;

	printf("  %d triangles, %d random rays (%d hits): %d mismatches (%.2fs incl. brute force)\n",
		ag_mesh_triangle_count(tw.mesh), rays, hits, mismatches, brute_and_bvh);
	printf("  %.1f million short rays per second\n", bvh_rays / fmax(bvh_time, 1e-6) / 1e6);
	CHECK(mismatches == 0, "%d BVH/brute force mismatches", mismatches);
	CHECK(hits > rays / 4, "only %d of %d rays hit", hits, rays);
	test_world_free(&tw);
}


// Route walls & resampling -----------------------------------------------------

static void test_route_walls(void) {
	printf("\n== Route walls on an open floor ==\n");
	test_world_t tw;
	test_world_make_flat(&tw, false, 0);
	tw.route_len = 101;
	tw.route = malloc(sizeof(ag_route_point_t) * tw.route_len);
	for (int i = 0; i < tw.route_len; i++) {
		tw.route[i] = (ag_route_point_t){v3(0, 0, i * 20.0f), v3(0, 0, 0), 20};
	}

	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1);
	desc.raycast = test_world_raycast;
	desc.user = &tw;
	desc.route = tw.route;
	desc.route_len = tw.route_len;
	desc.route_closed = false;
	desc.route_walls = true;
	ag_world_t *w = ag_world_create(&desc);

	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);
	float a = 30 * PI / 180;
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 50), v3(sinf(a), 0, cosf(a)));
	ag_ship_apply_impulse(w, ship, v3(sinf(a) * 120, 0, cosf(a) * 120));
	ag_input_t in = {.thrust = 1};
	ag_ship_set_input(w, ship, &in);

	float max_x = 0;
	int wall_hits = 0;
	for (int i = 0; i < 240; i++) {
		ag_world_step(w, 1.0f / 60);
		max_x = fmaxf(max_x, fabsf(ag_ship_state(w, ship)->position.x));
		ag_event_t ev;
		while (ag_world_poll_event(w, &ev)) {
			wall_hits += ev.type == AG_EVENT_WALL_HIT;
		}
	}
	const ag_ship_state_t *s = ag_ship_state(w, ship);
	float limit = 10 - params.half_extents.x;
	printf("  30 deg into the edge at 120 m/s: max |x| %.2f (limit %.2f), %d wall events, %.1f m/s after\n", max_x, limit, wall_hits, s->speed);
	CHECK(max_x <= limit + 0.01f, "left the route width: %.2f", max_x);
	CHECK(wall_hits >= 1, "no wall event");
	CHECK(s->speed > 60, "lost too much speed against a route wall: %.1f", s->speed);

	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_resample(void) {
	printf("\n== Route resampling ==\n");
	ag_route_point_t controls[8];
	for (int i = 0; i < 8; i++) {
		float a = i * 2 * PI / 8;
		controls[i] = (ag_route_point_t){v3(300 * cosf(a), 0, 300 * sinf(a)), v3(0, 0, 0), 20 + i};
	}
	int count = ag_route_resample(controls, 8, true, 10, NULL, 0);
	ag_route_point_t *out = malloc(sizeof(ag_route_point_t) * count);
	int written = ag_route_resample(controls, 8, true, 10, out, count);

	float max_dev = 0, min_gap = INFINITY, max_gap = 0;
	for (int i = 0; i < count; i++) {
		ag_vec3_t p = out[i].position, q = out[(i + 1) % count].position;
		max_dev = fmaxf(max_dev, fabsf(len(p) - 300));
		float gap = len(v3(q.x - p.x, q.y - p.y, q.z - p.z));
		min_gap = fminf(min_gap, gap);
		max_gap = fmaxf(max_gap, gap);
	}
	printf("  8 points on a 300 m circle -> %d points (written %d), off the circle by <= %.2f m, spacing %.2f..%.2f m\n",
		count, written, max_dev, min_gap, max_gap);
	CHECK(written == count, "wrote %d of %d", written, count);
	CHECK(abs(count - 188) <= 2, "unexpected count %d", count);
	CHECK(max_dev < 3, "spline strays %.2f m from the circle", max_dev);
	CHECK(min_gap > 9 && max_gap < 11, "uneven spacing %.2f..%.2f", min_gap, max_gap);
	free(out);

	ag_route_point_t line[3] = {
		{v3(0, 0, 0), v3(0, 0, 0), 10}, {v3(0, 0, 50), v3(0, 0, 0), 10}, {v3(0, 0, 100), v3(0, 0, 0), 10}
	};
	ag_route_point_t lout[16];
	int lcount = ag_route_resample(line, 3, false, 10, lout, 16);
	printf("  open 100 m line -> %d points, ends at z = %.1f and %.1f\n", lcount, lout[0].position.z, lout[lcount - 1].position.z);
	CHECK(lcount == 11, "open line count %d", lcount);
	CHECK(fabsf(lout[lcount - 1].position.z - 100) < 0.5f, "open line end at %.2f", lout[lcount - 1].position.z);
}

static void test_open_race(void) {
	printf("\n== AI race on the open circuit (no walls, route walls on) ==\n");
	test_world_t tw;
	test_world_make_circuit(&tw, false);
	race_result_t r;
	run_race(&tw, 1, true, true, &r);
	int walls = 0, hard = 0, oob = 0;
	for (int i = 0; i < RACE_SHIPS; i++) {
		walls += r.wall_hits[i];
		hard += r.hard_wall_hits[i];
		oob += r.out_of_bounds[i];
	}
	printf("  %s finished, winner #%d in %.2fs, %d route wall hits (%d hard), %d out of bounds\n",
		r.all_finished ? "all" : "NOT ALL", r.finish_order[0], r.finish_time[r.finish_order[0]], walls, hard, oob);
	CHECK(r.all_finished && r.finite, "open circuit race failed");
	CHECK(oob <= RACE_SHIPS, "%d out of bounds on the open circuit", oob);
	test_world_free(&tw);
}


// Later WipEout mechanics ------------------------------------------------------

static ag_world_t *make_flat_world(test_world_t *tw, bool with_wall, bool energy) {
	test_world_make_flat(tw, with_wall, 0);
	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1);
	desc.raycast = test_world_raycast;
	desc.user = tw;
	desc.use_energy = energy;
	return ag_world_create(&desc);
}

static int count_events(ag_world_t *w, ag_event_type_t type, float *last_value) {
	int n = 0;
	ag_event_t ev;
	while (ag_world_poll_event(w, &ev)) {
		if (ev.type == type) {
			n++;
			if (last_value) *last_value = ev.value;
		}
	}
	return n;
}

static void test_airbrakes(void) {
	printf("\n== Airbrakes: one turns, both brake ==\n");
	test_world_t tw;
	ag_world_t *w = make_flat_world(&tw, false, false);
	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));
	ag_ship_apply_impulse(w, ship, v3(0, 0, params.top_speed));

	ag_input_t in = {.thrust = 1, .airbrake_right = 1};
	ag_ship_set_input(w, ship, &in);
	for (int i = 0; i < 120; i++) ag_world_step(w, 1.0f / 60);
	float single = ag_ship_state(w, ship)->speed;

	in = (ag_input_t){.thrust = 0, .airbrake_left = 1, .airbrake_right = 1};
	ag_ship_set_input(w, ship, &in);
	float t = 0;
	while (ag_ship_state(w, ship)->speed > 50 && t < 10) {
		ag_world_step(w, 1.0f / 60);
		t += 1.0f / 60;
	}
	printf("  2s on one airbrake at full thrust: %.1f m/s (of %.0f); both airbrakes, throttle off: down to 50 m/s in %.2fs\n", single, params.top_speed, t);
	CHECK(single > params.top_speed * 0.85f, "single airbrake costs too much speed: %.1f", single);
	CHECK(t < 4, "both airbrakes too weak: %.2fs to 50 m/s", t);
	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_sideshift(void) {
	printf("\n== Sideshift ==\n");
	for (int mode = 0; mode < 2; mode++) {
		test_world_t tw;
		ag_world_t *w = make_flat_world(&tw, false, false);
		ag_ship_params_t params;
		ag_ship_params_default(&params, 1);
		ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));
		ag_ship_apply_impulse(w, ship, v3(0, 0, 120));

		int shifts = 0;
		float x0 = 0, heading0 = 0;
		for (int f = 0; f < 120; f++) {
			ag_input_t in = {.thrust = 1};
			if (mode == 0) {
				// Double tap the right airbrake: 3 frames down, 3 up, 3 down
				in.airbrake_right = (f >= 30 && f < 33) || (f >= 36 && f < 39) ? 1 : 0;
			}
			else {
				in.sideshift = f >= 30 && f < 70 ? 1 : 0; // Held
			}
			if (f == 29) {
				x0 = ag_ship_state(w, ship)->position.x;
				heading0 = heading(ag_ship_state(w, ship)->forward);
			}
			ag_ship_set_input(w, ship, &in);
			ag_world_step(w, 1.0f / 60);
			shifts += count_events(w, AG_EVENT_SIDESHIFT, NULL);
		}
		const ag_ship_state_t *s = ag_ship_state(w, ship);
		float dx = s->position.x - x0;
		float turned = fabsf(heading(s->forward) - heading0) * 180 / PI;
		// Right handed, Y up, facing +z: right is -x
		printf("  %-26s moved %+.2f m sideways, heading changed %.2f deg, %d event(s)\n",
			mode == 0 ? "double tap right airbrake:" : "sideshift input held:", dx, turned, shifts);
		CHECK(shifts == 1, "expected one sideshift, got %d", shifts);
		CHECK(dx < -2.5f, "sideshift went %.2f m (expected to the right, -x)", dx);
		CHECK(turned < 2, "sideshift turned the craft by %.2f deg", turned);
		if (mode == 1) {
			CHECK(dx < -5.5f, "holding the sideshift should go further: %.2f m", dx);
		}
		ag_world_destroy(w);
		test_world_free(&tw);
	}
}

static void run_barrel_roll(int mode, int *starts, float *result, float *boost, float *energy) {
	test_world_t tw;
	ag_world_t *w = make_flat_world(&tw, false, true);
	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 60, 0), v3(0, 0, 1)); // Airborne
	ag_ship_apply_impulse(w, ship, v3(0, 0, 100));

	*starts = 0;
	*result = -1;
	*boost = 0;
	int gesture_frame = mode == 0 ? 10 : (mode == 1 ? -1 : -1000);
	for (int f = 0; f < 400 && *result < 0 && !(mode == 2 && ag_ship_state(w, ship)->grounded && f > 10); f++) {
		const ag_ship_state_t *s = ag_ship_state(w, ship);
		if (gesture_frame == -1 && s->position.y < 30) {
			gesture_frame = f;
		}
		int g = gesture_frame >= 0 ? f - gesture_frame : -1;
		ag_input_t in = {.thrust = 1};
		if (g >= 0 && g < 3) in.steer = -1;
		else if (g >= 3 && g < 6) in.steer = 1;
		else if (g >= 6 && g < 9) in.steer = -1;
		ag_ship_set_input(w, ship, &in);
		ag_world_step(w, 1.0f / 60);

		ag_event_t ev;
		while (ag_world_poll_event(w, &ev)) {
			if (ev.type == AG_EVENT_BARREL_ROLL_START) (*starts)++;
			if (ev.type == AG_EVENT_BARREL_ROLL) *result = ev.value;
		}
	}
	*boost = ag_ship_state(w, ship)->boost_time;
	*energy = ag_ship_state(w, ship)->energy;
	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_barrel_roll(void) {
	printf("\n== Barrel roll (steer left-right-left in the air) ==\n");
	int starts;
	float result, boost, energy, baseline;
	run_barrel_roll(2, &starts, &result, &boost, &baseline); // Same jump without a roll
	run_barrel_roll(0, &starts, &result, &boost, &energy);
	printf("  early in a long jump: %d roll(s), landed %s, boost %.2fs, energy %.0f\n", starts, result == 1 ? "clean" : "FAILED", boost, energy);
	CHECK(starts == 1 && result == 1 && boost > 0, "clean roll didn't boost");
	CHECK(fabsf(baseline - energy - 10) < 0.5f, "roll cost %.1f energy", baseline - energy);

	run_barrel_roll(1, &starts, &result, &boost, &energy);
	printf("  just before landing:  %d roll(s), landed %s, boost %.2fs, energy %.0f\n", starts, result == 0 ? "unfinished (no boost)" : "CLEAN?", boost, energy);
	CHECK(starts == 1 && result == 0 && boost <= 0, "unfinished roll should fail without boost");
	CHECK(fabsf(baseline - energy - 10) < 0.5f, "failed roll should still cost energy (%.1f)", baseline - energy);
}

static void test_energy(void) {
	printf("\n== Shield energy (optional) ==\n");
	float angles[] = {10, 90};
	for (int i = 0; i < 2; i++) {
		test_world_t tw;
		ag_world_t *w = make_flat_world(&tw, true, true);
		ag_ship_params_t params;
		ag_ship_params_default(&params, 1);
		float a = angles[i] * PI / 180;
		ag_vec3_t fwd = v3(-sinf(a), 0, cosf(a));
		ag_ship_id_t ship = ag_ship_add(w, &params, v3(14 * sinf(a) + 4, 2, 0), fwd);
		ag_ship_apply_impulse(w, ship, v3(fwd.x * 120, 0, fwd.z * 120));
		ag_input_t in = {.thrust = 1};
		ag_ship_set_input(w, ship, &in);
		for (int f = 0; f < 90; f++) ag_world_step(w, 1.0f / 60);
		float e = ag_ship_state(w, ship)->energy;
		printf("  wall at %2.0f deg, 120 m/s, then riding it: energy %.1f\n", angles[i], e);
		if (i == 0) CHECK(e > 75, "graze cost %.1f energy", 100 - e);
		else CHECK(e < 70, "head-on only cost %.1f energy", 100 - e);
		ag_world_destroy(w);
		test_world_free(&tw);
	}

	test_world_t tw;
	ag_world_t *w = make_flat_world(&tw, false, true);
	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));
	ag_ship_apply_impulse(w, ship, v3(0, 0, params.top_speed));
	ag_input_t in = {.thrust = 1, .hyperthrust = true};
	ag_ship_set_input(w, ship, &in);
	float top = 0;
	for (int f = 0; f < 240; f++) {
		ag_world_step(w, 1.0f / 60);
		top = fmaxf(top, ag_ship_state(w, ship)->speed);
	}
	float e = ag_ship_state(w, ship)->energy;
	printf("  hyperthrust for 4s: peak %.1f m/s (top speed %.0f), energy left %.1f (cuts out at %.0f)\n", top, params.top_speed, e, params.hyperthrust_min * params.energy_max);
	CHECK(top > params.top_speed * 1.1f, "hyperthrust only reached %.1f", top);
	CHECK(fabsf(e - params.hyperthrust_min * params.energy_max) < 1, "hyperthrust left %.1f energy", e);

	float depleted_value;
	float left = ag_ship_add_energy(w, ship, -1000);
	int depleted = count_events(w, AG_EVENT_ENERGY_DEPLETED, &depleted_value);
	CHECK(left == 0 && depleted == 1, "draining energy: %.1f, %d depleted events", left, depleted);
	CHECK(ag_ship_add_energy(w, ship, 30) == 30, "absorbing energy failed");
	ag_world_destroy(w);
	test_world_free(&tw);
}

static void run_drop(bool maglock, float *max_air, float *end_y, float *end_z) {
	test_world_t tw;
	test_world_make_drop(&tw, maglock);
	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1);
	desc.raycast = test_world_raycast;
	desc.user = &tw;
	ag_world_t *w = ag_world_create(&desc);
	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);
	ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 20), v3(0, 0, 1));
	ag_ship_apply_impulse(w, ship, v3(0, 0, 100));
	ag_input_t in = {.thrust = 1};
	ag_ship_set_input(w, ship, &in);
	*max_air = 0;
	for (int f = 0; f < 300; f++) {
		ag_world_step(w, 1.0f / 60);
		*max_air = fmaxf(*max_air, ag_ship_state(w, ship)->air_time);
	}
	*end_y = ag_ship_state(w, ship)->position.y;
	*end_z = ag_ship_state(w, ship)->position.z;
	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_maglock(void) {
	printf("\n== Mag-strips: over a 90 deg edge and down a vertical drop at 100 m/s ==\n");
	float air, y, z;
	run_drop(true, &air, &y, &z);
	printf("  mag-strip:    longest airtime %.2fs, ends at y %.0f z %.0f\n", air, y, z);
	CHECK(air < 0.05f, "left the mag-strip for %.2fs", air);
	CHECK(y < -100 && z > 400, "didn't make it down the drop (y %.0f, z %.0f)", y, z);
	run_drop(false, &air, &y, &z);
	printf("  no mag-strip: longest airtime %.2fs (flies off the edge)\n", air);
	CHECK(air > 0.3f, "should fly off the edge without a mag-strip");
}

static void test_speed_classes(void) {
	printf("\n== Speed classes ==\n");
	const char *names[] = {"Venom", "Flash", "Rapier", "Phantom"};
	test_world_t circuit;
	test_world_make_circuit(&circuit, true);
	for (int c = AG_CLASS_VENOM; c <= AG_CLASS_PHANTOM; c++) {
		ag_ship_params_t params;
		ag_ship_params_class(&params, (ag_speed_class_t)c, 1);

		test_world_t tw;
		ag_world_t *w = make_flat_world(&tw, false, false);
		ag_ship_id_t ship = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));
		ag_input_t in = {.thrust = 1};
		ag_ship_set_input(w, ship, &in);
		float t = 0, t100 = -1;
		while (t < 20) {
			ag_world_step(w, 1.0f / 60);
			t += 1.0f / 60;
			if (t100 < 0 && ag_ship_state(w, ship)->speed >= 100) t100 = t;
		}
		float top = ag_ship_state(w, ship)->speed;
		ag_world_destroy(w);
		test_world_free(&tw);

		// AI race on the walled circuit in this class
		ag_world_t *rw = make_world(&circuit, true);
		ag_ship_id_t ids[4];
		for (int i = 0; i < 4; i++) {
			ids[i] = ag_ship_add(rw, &params, v3(0, 0, 0), v3(0, 0, 1));
			ag_ship_place_on_grid(rw, ids[i], i);
			ag_ai_params_t ai;
			ag_ai_params_default(&ai);
			ag_ship_set_ai(rw, ids[i], &ai);
		}
		int oob = 0, hard = 0;
		float best = INFINITY;
		bool done = false;
		for (int f = 0; f < 60 * 200 && !done; f++) {
			ag_world_step(rw, 1.0f / 60);
			ag_event_t ev;
			while (ag_world_poll_event(rw, &ev)) {
				oob += ev.type == AG_EVENT_OUT_OF_BOUNDS;
				hard += ev.type == AG_EVENT_WALL_HIT && ev.intensity > 15;
			}
			done = true;
			for (int i = 0; i < 4; i++) {
				if (ag_ship_state(rw, ids[i])->lap <= 2) done = false;
			}
		}
		for (int i = 0; i < 4; i++) {
			float lap = ag_ship_state(rw, ids[i])->best_lap_time;
			if (lap > 0) best = fminf(best, lap);
		}
		ag_world_destroy(rw);

		printf("  %-8s top %5.1f m/s (%3.0f km/h)  0-100 m/s %.2fs  AI: best lap %.2fs, %d hard wall hits, %d out of bounds%s\n",
			names[c], top, top * 3.6f, t100, best, hard, oob, done ? "" : "  NOT FINISHED");
		CHECK(fabsf(top - params.top_speed) < params.top_speed * 0.03f, "%s top speed %.1f", names[c], top);
		CHECK(done, "%s: AI didn't finish 2 laps", names[c]);
		CHECK(oob <= 2, "%s: %d out of bounds", names[c], oob);
	}
	test_world_free(&circuit);
}


// AI ---------------------------------------------------------------------------

typedef struct {
	float best_lap[8];
	float finish[8];
	int sideshifts, rolls, clean_rolls, pads, hard_walls, depleted, oob;
	float hyperthrust_time;
	bool all_finished;
} ai_race_t;

static void run_ai_race(test_world_t *tw, const ag_ai_params_t *ai, int count, bool energy, int laps, ai_race_t *r) {
	memset(r, 0, sizeof(*r));
	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1);
	desc.raycast = test_world_raycast;
	desc.user = tw;
	desc.route = tw->route;
	desc.route_len = tw->route_len;
	desc.use_energy = energy;
	ag_world_t *w = ag_world_create(&desc);

	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);
	for (int i = 0; i < count; i++) {
		ag_ship_add(w, &params, v3(0, 0, 0), v3(0, 0, 1));
		ag_ship_place_on_grid(w, i, i);
		ag_ship_set_ai(w, i, &ai[i]);
		r->finish[i] = -1;
	}

	const float dt = 1.0f / 60;
	float t = 0;
	int finished = 0;
	while (t < 400 && finished < count) {
		ag_world_step(w, dt);
		t += dt;
		ag_event_t ev;
		while (ag_world_poll_event(w, &ev)) {
			switch (ev.type) {
				case AG_EVENT_SIDESHIFT: r->sideshifts++; break;
				case AG_EVENT_BARREL_ROLL_START: r->rolls++; break;
				case AG_EVENT_BARREL_ROLL: r->clean_rolls += ev.value == 1; break;
				case AG_EVENT_BOOST_PAD: r->pads++; break;
				case AG_EVENT_WALL_HIT: r->hard_walls += ev.intensity > 15; break;
				case AG_EVENT_ENERGY_DEPLETED: r->depleted++; break;
				case AG_EVENT_OUT_OF_BOUNDS: r->oob++; break;
				default: break;
			}
		}
		for (int i = 0; i < count; i++) {
			const ag_ship_state_t *s = ag_ship_state(w, i);
			r->hyperthrust_time += s->hyperthrusting ? dt : 0;
			if (r->finish[i] < 0 && s->lap > laps) {
				r->finish[i] = t;
				r->best_lap[i] = s->best_lap_time;
				finished++;
			}
		}
	}
	r->all_finished = finished == count;
	ag_world_destroy(w);
}

static void test_ai_tricks(void) {
	printf("\n== AI tricks (8 Expert craft, 3 laps, energy on) ==\n");
	test_world_t tw;
	test_world_make_circuit(&tw, true);

	ai_race_t with, without;
	ag_ai_params_t ai[8];
	for (int k = 0; k < 2; k++) {
		for (int i = 0; i < 8; i++) {
			ag_ai_params_default(&ai[i]);
			ai[i].line_offset = ((i * 37) % 7 - 3) / 3.0f * 0.5f;
			if (k == 1) ai[i].tricks = 0;
		}
		run_ai_race(&tw, ai, 8, true, 3, k == 0 ? &with : &without);
	}
	for (int k = 0; k < 2; k++) {
		ai_race_t *r = k == 0 ? &with : &without;
		float best = INFINITY, avg = 0;
		for (int i = 0; i < 8; i++) {
			best = fminf(best, r->best_lap[i]);
			avg += r->finish[i] / 8;
		}
		printf("  %-14s best lap %.2fs, avg finish %.2fs | sideshifts %d, barrel rolls %d (%d clean), pads %d, hyperthrust %.1fs, hard wall hits %d, energy depleted %d, oob %d\n",
			k == 0 ? "with tricks:" : "without:", best, avg, r->sideshifts, r->rolls, r->clean_rolls, r->pads, r->hyperthrust_time,
			r->hard_walls, r->depleted, r->oob);
		CHECK(r->all_finished, "AI race (%s tricks) didn't finish", k == 0 ? "with" : "without");
	}
	CHECK(with.rolls > 0 && with.clean_rolls > 0, "AI never barrel rolled");
	CHECK(with.sideshifts > 0, "AI never sideshifted");
	CHECK(with.hyperthrust_time > 0, "AI never used hyperthrust");
	CHECK(with.pads > without.pads, "AI didn't hunt boost pads (%d vs %d)", with.pads, without.pads);
	test_world_free(&tw);
}

static void test_ai_difficulty(void) {
	printf("\n== AI difficulty ladder (one craft each, 3 laps, energy on) ==\n");
	test_world_t tw;
	test_world_make_circuit(&tw, true);
	const char *names[] = {"Novice", "Pilot", "Expert", "Elite"};
	ag_ai_params_t ai[4];
	for (int i = 0; i < 4; i++) {
		ag_ai_params_difficulty(&ai[i], (ag_ai_difficulty_t)i);
	}
	ai_race_t r;
	run_ai_race(&tw, ai, 4, true, 3, &r);
	for (int i = 0; i < 4; i++) {
		printf("  %-7s best lap %6.2fs  finished %6.2fs\n", names[i], r.best_lap[i], r.finish[i]);
	}
	CHECK(r.all_finished, "difficulty race didn't finish");
	CHECK(r.best_lap[3] < r.best_lap[2] && r.best_lap[2] < r.best_lap[1] && r.best_lap[1] < r.best_lap[0], "difficulty levels aren't ordered by lap time");
	test_world_free(&tw);
}


// Lock-on, pit lane, craft hits -------------------------------------------------

static void test_combat_helpers(void) {
	printf("\n== Lock-on, pit lane, craft hits ==\n");
	test_world_t tw;
	test_world_init(&tw);
	float sz = 2000;
	test_world_add_quad(&tw, v3(-sz, 0, -sz), v3(sz, 0, -sz), v3(sz, 0, sz), v3(-sz, 0, sz), v3(0, 1, 0), AG_SURFACE_PIT);
	test_world_build(&tw);
	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1);
	desc.raycast = test_world_raycast;
	desc.user = &tw;
	desc.use_energy = true;
	ag_world_t *w = ag_world_create(&desc);
	ag_ship_params_t params;
	ag_ship_params_default(&params, 1);

	ag_ship_id_t a = ag_ship_add(w, &params, v3(0, 2, 0), v3(0, 0, 1));
	ag_ship_id_t ahead = ag_ship_add(w, &params, v3(3, 2, 60), v3(0, 0, 1));
	ag_ship_id_t behind = ag_ship_add(w, &params, v3(0, 2, -40), v3(0, 0, 1));
	ag_ship_id_t t1 = ag_ship_find_target(w, a, 100, 0.3f);
	ag_ship_id_t t2 = ag_ship_find_target(w, ahead, 100, 0.3f);
	printf("  lock-on: craft 0 targets #%d (ahead), the leader targets #%d (none ahead)\n", t1, t2);
	CHECK(t1 == ahead, "lock-on picked #%d", t1);
	CHECK(t2 == AG_INVALID_SHIP, "leader locked onto #%d", t2);
	(void)behind;

	ag_ship_add_energy(w, a, -60);
	for (int f = 0; f < 60; f++) ag_world_step(w, 1.0f / 60);
	float e = ag_ship_state(w, a)->energy;
	printf("  pit lane: 40 -> %.1f energy after 1s\n", e);
	CHECK(fabsf(e - (40 + params.pit_recharge)) < 1, "pit recharge gave %.1f", e - 40);

	// Side swipe into the nose of a stationary craft
	ag_ship_id_t hitter = ag_ship_add(w, &params, v3(-30, 2, 2.5f), v3(1, 0, 0));
	ag_ship_apply_impulse(w, hitter, v3(40, 0, 0));
	float max_yaw = 0;
	for (int f = 0; f < 60; f++) {
		ag_world_step(w, 1.0f / 60);
		max_yaw = fmaxf(max_yaw, fabsf(ag_ship_state(w, a)->yaw_rate));
	}
	printf("  side swipe on the nose at 40 m/s: target spun up to %.2f rad/s\n", max_yaw);
	CHECK(max_yaw > 0.3f, "off-center hit didn't spin the target");

	ag_world_destroy(w);
	test_world_free(&tw);
}


int main(int argc, char **argv) {
	(void)argc;
	(void)argv;
	test_acceleration();
	test_turning();
	test_walls();
	test_mesh();
	test_route_walls();
	test_resample();
	test_airbrakes();
	test_sideshift();
	test_barrel_roll();
	test_energy();
	test_maglock();
	test_combat_helpers();
	test_speed_classes();
	test_ai_tricks();
	test_ai_difficulty();
	test_race();
	test_open_race();

	printf("\n%s (%d failed checks)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
