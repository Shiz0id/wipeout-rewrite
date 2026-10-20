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

static ag_world_t *make_world(test_world_t *tw, bool with_route) {
	ag_world_desc_t desc;
	ag_world_desc_default(&desc, 1.0f);
	desc.raycast = test_world_raycast;
	desc.user = tw;
	if (with_route) {
		desc.route = tw->route;
		desc.route_len = tw->route_len;
		desc.route_closed = true;
	}
	return ag_world_create(&desc);
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

static void run_wall(float angle_deg, float speed, float *retained, bool *tunneled) {
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
	while (t < 4 && (hit_time < 0 || t < hit_time + 0.4f)) {
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
		if (hit_time >= 0) {
			after = fminf(after, s->speed);
		}
	}
	*retained = hit_time >= 0 ? after / before : 1;

	ag_world_destroy(w);
	test_world_free(&tw);
}

static void test_walls(void) {
	printf("\n== Wall hits at 120 m/s (speed kept right after the hit) ==\n");
	float angles[] = {5, 10, 20, 30, 45, 60, 90};
	float kept[7];
	for (int i = 0; i < 7; i++) {
		bool tunneled;
		run_wall(angles[i], 120, &kept[i], &tunneled);
		printf("  %4.0f deg: %5.1f%%%s\n", angles[i], kept[i] * 100, tunneled ? "  TUNNELED" : "");
		CHECK(!tunneled, "tunneled through the wall at %.0f deg", angles[i]);
	}
	CHECK(kept[0] > 0.95f, "grazing at 5 deg kept only %.1f%%", kept[0] * 100);
	CHECK(kept[1] > 0.9f, "grazing at 10 deg kept only %.1f%%", kept[1] * 100);
	CHECK(kept[6] < 0.5f, "head-on kept %.1f%%", kept[6] * 100);
	for (int i = 1; i < 7; i++) {
		CHECK(kept[i] <= kept[i - 1] + 0.02f, "speed loss not monotonic at %.0f deg", angles[i]);
	}

	float speeds[] = {200, 300};
	for (int i = 0; i < 2; i++) {
		float k;
		bool tunneled;
		run_wall(90, speeds[i], &k, &tunneled);
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

static void run_race(test_world_t *tw, int dt_mode, bool with_countdown, race_result_t *r) {
	memset(r, 0, sizeof(*r));
	r->finite = true;
	ag_world_t *w = make_world(tw, true);

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
	test_world_make_circuit(&tw);

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
		run_race(&tw, mode, true, &results[mode]);
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
		run_race(&tw, mode, false, &exact[mode]);
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
	CHECK(oob <= RACE_SHIPS, "%d out of bounds", oob);

	test_world_free(&tw);
}


int main(int argc, char **argv) {
	(void)argc;
	(void)argv;
	test_acceleration();
	test_turning();
	test_walls();
	test_race();

	printf("\n%s (%d failed checks)\n", failures ? "FAILED" : "PASSED", failures);
	return failures ? 1 : 0;
}
