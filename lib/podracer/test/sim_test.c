// Headless simulation tests and tuning report for podracer.
// Exits non-zero if any check fails.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../podracer.h"
#include "../../antigrav/test/test_world.h"

#define PI 3.14159265358979323846f
#define DT (1.0f / 60.0f)

static int failures = 0;

#define CHECK(COND, ...) do { \
	if (!(COND)) { \
		failures++; \
		printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); \
	} \
} while (0)

static ag_vec3_t v3(float x, float y, float z) { return (ag_vec3_t){x, y, z}; }

static bool vec_finite(ag_vec3_t a) {
	return isfinite(a.x) && isfinite(a.y) && isfinite(a.z);
}

static float heading(ag_vec3_t f) {
	return atan2f(f.x, f.z);
}

static float wrap_angle(float a) {
	while (a > PI) a -= 2 * PI;
	while (a < -PI) a += 2 * PI;
	return a;
}


// Worlds -----------------------------------------------------------------------

// Host side surface mapping: the shared test geometry is tagged with
// antigrav's flags; a host maps its own materials the same way
static bool mesh_raycast(void *user, ag_vec3_t origin, ag_vec3_t dir, float max_distance, ag_hit_t *hit) {
	if (!test_world_raycast(user, origin, dir, max_distance, hit)) {
		return false;
	}
	uint32_t f = hit->surface_flags, out = 0;
	if (f & AG_SURFACE_KILL) out |= PR_SURFACE_FALL;
	if (f & AG_SURFACE_BOOST) out |= PR_SURFACE_FAST;
	if (f & AG_SURFACE_MAGLOCK) out |= PR_SURFACE_MAGNET;
	if (f & AG_SURFACE_WALL) out |= PR_SURFACE_WALL;
	hit->surface_flags = out;
	return true;
}

// An endless floor at y = 0 in host units, with surface flags by z band and an
// optional wall plane x = wall_x facing +x
typedef struct {
	float z0, z1;
	uint32_t flags;
} band_t;

typedef struct {
	band_t bands[4];
	int bands_len;
	bool wall;
	float wall_x;
	float drop_z, drop_h;   // Floor steps down by drop_h past z = drop_z
} plane_world_t;

static bool plane_raycast(void *user, ag_vec3_t o, ag_vec3_t d, float max_distance, ag_hit_t *hit) {
	plane_world_t *pw = user;
	bool found = false;
	if (d.y < -1e-6f) {
		float floor_y = pw->drop_h > 0 && o.z > pw->drop_z ? -pw->drop_h : 0;
		float t = (floor_y - o.y) / d.y;
		if (t >= 0 && t <= max_distance) {
			ag_vec3_t p = v3(o.x + d.x * t, floor_y, o.z + d.z * t);
			uint32_t flags = 0;
			for (int i = 0; i < pw->bands_len; i++) {
				if (p.z >= pw->bands[i].z0 && p.z < pw->bands[i].z1) flags |= pw->bands[i].flags;
			}
			*hit = (ag_hit_t){.position = p, .normal = v3(0, 1, 0), .distance = t, .surface_flags = flags};
			max_distance = t;
			found = true;
		}
	}
	if (pw->wall && d.x < -1e-6f && o.x >= pw->wall_x) {
		float t = (pw->wall_x - o.x) / d.x;
		if (t <= max_distance) {
			*hit = (ag_hit_t){.position = v3(pw->wall_x, o.y + d.y * t, o.z + d.z * t), .normal = v3(1, 0, 0), .distance = t};
			found = true;
		}
	}
	return found;
}

static pr_world_t *make_plane_world(plane_world_t *pw, float scale) {
	pr_world_desc_t desc;
	pr_world_desc_default(&desc, scale);
	desc.raycast = plane_raycast;
	desc.user = pw;
	return pr_world_create(&desc);
}

static pr_world_t *make_mesh_world(test_world_t *tw, float scale, bool with_route, float ai_spread) {
	pr_world_desc_t desc;
	pr_world_desc_default(&desc, scale);
	desc.raycast = mesh_raycast;
	desc.user = tw;
	if (with_route) {
		desc.route = tw->route;
		desc.route_len = tw->route_len;
		desc.route_closed = true;
	}
	desc.ai_spread = ai_spread;
	return pr_world_create(&desc);
}

static void step_for(pr_world_t *w, float seconds) {
	int n = (int)(seconds / DT + 0.5f);
	for (int i = 0; i < n; i++) {
		pr_world_step(w, DT);
	}
}

static int count_events(pr_world_t *w, pr_event_type_t type, pr_event_t *last) {
	int n = 0;
	pr_event_t e;
	while (pr_world_poll_event(w, &e)) {
		if (e.type == type) {
			n++;
			if (last) *last = e;
		}
	}
	return n;
}


// Speed ------------------------------------------------------------------------

static void test_speed(void) {
	printf("\n== Speed curve & hover (flat ground, stock-like pod) ==\n");
	plane_world_t pw = {0};
	pr_world_t *w = make_plane_world(&pw, 0.45f);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, 0), v3(0, 0, 1));

	pr_input_t in = {.throttle = 1};
	pr_pod_set_input(w, pod, &in);
	float t = 0, t300 = -1, t400 = -1;
	float gd_min = INFINITY, gd_max = -INFINITY;
	while (t < 20) {
		pr_world_step(w, DT);
		t += DT;
		const pr_pod_state_t *s = pr_pod_state(w, pod);
		if (t300 < 0 && s->speed >= 300) t300 = t;
		if (t400 < 0 && s->speed >= 400) t400 = t;
		if (t > 2) {
			gd_min = fminf(gd_min, s->ground_distance);
			gd_max = fmaxf(gd_max, s->ground_distance);
		}
	}
	const pr_pod_state_t *s = pr_pod_state(w, pod);
	// The game's curve: thrust grows 1.5/s, speed = thrust * max / (accel + thrust)
	float thrust = 1.5f * 20.0f;
	float expect = thrust * st.max_speed / (st.acceleration + thrust);
	printf("  0-300: %.2fs  0-400: %.2fs  after 20s: %.1f (curve %.1f, max_speed %.0f)\n", t300, t400, s->speed, expect, st.max_speed);
	printf("  ride height %.2f..%.2f (hover_height %.1f)\n", gd_min, gd_max, st.hover_height);
	CHECK(fabsf(s->speed - expect) < 2.0f, "speed %.1f off the game's curve %.1f", s->speed, expect);
	CHECK(t300 > 0.8f && t300 < 3.0f, "0-300 took %.2fs", t300);
	CHECK(gd_max - gd_min < 0.05f && fabsf(gd_min - st.hover_height) < 0.1f, "hover not steady");
	CHECK(!s->airborne, "airborne on flat ground");

	// Host units: 0.45 m per Racer unit
	float host_speed = sqrtf(s->velocity.x * s->velocity.x + s->velocity.z * s->velocity.z);
	CHECK(fabsf(host_speed - s->speed * 0.45f) < 1.0f, "host velocity %.1f vs %.1f", host_speed, s->speed * 0.45f);

	// Coast vs air brake from speed
	pr_world_t *w2 = make_plane_world(&pw, 0.45f);
	pr_pod_id_t pod2 = pr_pod_add(w2, &st, v3(0, 3, 0), v3(0, 0, 1));
	pr_pod_set_input(w2, pod2, &in);
	step_for(w2, 20);
	float v0 = pr_pod_state(w2, pod2)->speed;
	in = (pr_input_t){0};
	pr_pod_set_input(w, pod, &in);
	step_for(w, 2);
	in.brake = true;
	pr_pod_set_input(w2, pod2, &in);
	step_for(w2, 2);
	float coast = pr_pod_state(w, pod)->speed, brake = pr_pod_state(w2, pod2)->speed;
	printf("  2s from %.0f: coasting %.0f, air brake %.0f\n", v0, coast, brake);
	CHECK(coast < v0 - 20 && brake < coast - 40, "coast %.0f / brake %.0f don't slow enough", coast, brake);

	pr_world_destroy(w);
	pr_world_destroy(w2);
}


// Turning ----------------------------------------------------------------------

static void test_turning(void) {
	printf("\n== Turning (flat ground) ==\n");
	plane_world_t pw = {0};
	pr_world_t *w = make_plane_world(&pw, 0.45f);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, 0), v3(0, 0, 1));
	pr_input_t in = {.throttle = 1};
	pr_pod_set_input(w, pod, &in);
	step_for(w, 6);

	// Below the 0.05 deadzone nothing happens
	in.steer = 0.04f;
	pr_pod_set_input(w, pod, &in);
	step_for(w, 0.5f);
	CHECK(pr_pod_state(w, pod)->turn_rate == 0, "deadzone: turn rate %.2f", pr_pod_state(w, pod)->turn_rate);

	// Full right: the squared input saturates at max_turn_rate; yaw runs 1.5x
	in.steer = 1;
	pr_pod_set_input(w, pod, &in);
	float t_full = -1, t = 0;
	while (t < 1) {
		pr_world_step(w, DT);
		t += DT;
		if (t_full < 0 && pr_pod_state(w, pod)->turn_rate <= -st.max_turn_rate + 0.01f) t_full = t;
	}
	float h0 = heading(pr_pod_state(w, pod)->forward);
	step_for(w, 1);
	const pr_pod_state_t *s = pr_pod_state(w, pod);
	float yaw = wrap_angle(heading(s->forward) - h0) * 180 / PI;
	// Right handed, y up, z forward: right is -x, so the heading (atan2(x, z)) falls
	printf("  full right: turn rate %.1f (cap %.0f) reached in %.2fs, yaw %.1f deg/s\n", s->turn_rate, st.max_turn_rate, t_full, yaw);
	CHECK(fabsf(s->turn_rate + st.max_turn_rate) < 0.01f, "turn rate %.2f", s->turn_rate);
	CHECK(fabsf(yaw + st.max_turn_rate * 1.5f) < 4.0f, "yaw %.1f deg/s, expected %.1f", yaw, -st.max_turn_rate * 1.5f);
	CHECK(s->right.x < 0, "right isn't -x in a right handed world");

	// Counter steer builds 5x faster than turn-in
	in.steer = -1;
	pr_pod_set_input(w, pod, &in);
	t = 0;
	while (pr_pod_state(w, pod)->turn_rate < 0) {
		pr_world_step(w, DT);
		t += DT;
	}
	float t_counter = t;
	printf("  turn in to full: %.2fs   unwinding a full turn: %.2fs\n", t_full, t_counter);
	CHECK(t_counter < t_full * 0.35f, "counter steer %.2fs vs turn in %.2fs", t_counter, t_full);

	// Nose down trims turning
	pr_world_t *w2 = make_plane_world(&pw, 0.45f);
	pr_pod_id_t pod2 = pr_pod_add(w2, &st, v3(0, 3, 0), v3(0, 0, 1));
	in = (pr_input_t){.throttle = 1, .steer = 0.6f, .pitch = 1};
	pr_pod_set_input(w2, pod2, &in);
	pr_world_t *w3 = make_plane_world(&pw, 0.45f);
	pr_pod_id_t pod3 = pr_pod_add(w3, &st, v3(0, 3, 0), v3(0, 0, 1));
	in.pitch = 0;
	pr_pod_set_input(w3, pod3, &in);
	step_for(w2, 2);
	step_for(w3, 2);
	printf("  steer 0.6: turn rate %.1f level, %.1f nose down\n", pr_pod_state(w3, pod3)->turn_rate, pr_pod_state(w2, pod2)->turn_rate);
	CHECK(fabsf(pr_pod_state(w2, pod2)->turn_rate) < fabsf(pr_pod_state(w3, pod3)->turn_rate) * 0.75f, "nose down doesn't trim turning");

	// Ice: less grip, more slide
	plane_world_t ice = {.bands = {{-1e9f, 1e9f, PR_SURFACE_SLIP}}, .bands_len = 1};
	float slip[2];
	for (int k = 0; k < 2; k++) {
		pr_world_t *wi = make_plane_world(k ? &ice : &pw, 0.45f);
		pr_pod_id_t pi = pr_pod_add(wi, &st, v3(0, 3, 0), v3(0, 0, 1));
		in = (pr_input_t){.throttle = 1};
		pr_pod_set_input(wi, pi, &in);
		step_for(wi, 5);
		in.steer = 1;
		pr_pod_set_input(wi, pi, &in);
		slip[k] = 0;
		for (int i = 0; i < 90; i++) {
			pr_world_step(wi, DT);
			const pr_pod_state_t *si = pr_pod_state(wi, pi);
			slip[k] = fmaxf(slip[k], fabsf(wrap_angle(heading(si->velocity) - heading(si->forward))) * 180 / PI);
		}
		pr_world_destroy(wi);
	}
	printf("  full turn slide angle: %.1f deg on track, %.1f deg on ice\n", slip[0], slip[1]);
	CHECK(slip[1] > slip[0] * 1.5f, "ice doesn't slide more");

	pr_world_destroy(w);
	pr_world_destroy(w2);
	pr_world_destroy(w3);
}


// Boost & heat -----------------------------------------------------------------

static void test_boost(void) {
	printf("\n== Boost, heat & repair ==\n");
	plane_world_t pw = {0};
	pr_world_t *w = make_plane_world(&pw, 0.45f);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, 0), v3(0, 0, 1));

	pr_input_t in = {.throttle = 1};
	pr_pod_set_input(w, pod, &in);
	step_for(w, 6);
	count_events(w, PR_EVENT_BOOST, NULL);

	// Charge: full throttle, nose down
	in.pitch = 1;
	pr_pod_set_input(w, pod, &in);
	float t = 0;
	while (pr_pod_state(w, pod)->boost_state != PR_BOOST_READY && t < 3) {
		pr_world_step(w, DT);
		t += DT;
	}
	printf("  boost charged in %.2fs\n", t);
	CHECK(pr_pod_state(w, pod)->boost_state == PR_BOOST_READY, "boost never charged");
	CHECK(count_events(w, PR_EVENT_BOOST_READY, NULL) == 1, "no BOOST_READY event");

	// Fire
	in.pitch = 0;
	in.boost = true;
	pr_pod_set_input(w, pod, &in);
	pr_world_step(w, DT);
	float before = pr_pod_state(w, pod)->speed;
	in.boost = false;
	pr_pod_set_input(w, pod, &in);
	CHECK(pr_pod_state(w, pod)->boosting, "boost didn't fire");
	CHECK(count_events(w, PR_EVENT_BOOST, NULL) == 1, "no BOOST event");
	step_for(w, 2);
	const pr_pod_state_t *s = pr_pod_state(w, pod);
	printf("  boosting 2s: speed %.0f -> %.0f, engine temp %.0f\n", before, s->speed, s->engine_temp);
	CHECK(s->speed > before + st.boost_thrust * 0.7f, "boost too weak: %.0f -> %.0f", before, s->speed);
	CHECK(fabsf(s->engine_temp - (100 - 2 * st.heat_rate)) < 2, "heat %.1f", s->engine_temp);

	// Hold it until the engine catches fire
	t = 0;
	pr_event_t e = {0};
	int overheats = 0;
	while (t < 10 && !overheats) {
		pr_world_step(w, DT);
		t += DT;
		overheats = count_events(w, PR_EVENT_OVERHEAT, &e);
	}
	s = pr_pod_state(w, pod);
	printf("  overheated after %.1fs more: part %d on fire, boosting %d\n", t, (int)e.value, s->boosting);
	CHECK(overheats == 1 && s->engine_on_fire[(int)e.value] && !s->boosting, "no overheat");

	// A burning engine halves the throttle and pulls the steering
	float v_fire = s->speed;
	step_for(w, 3);
	s = pr_pod_state(w, pod);
	printf("  3s burning: speed %.0f -> %.0f, turn rate %.1f with no steering\n", v_fire, s->speed, s->turn_rate);
	CHECK(s->speed < v_fire - 50, "fire doesn't slow the pod");
	CHECK(fabsf(s->turn_rate) > 3, "damaged side doesn't pull");

	// Repair puts it out
	in.repair = true;
	pr_pod_set_input(w, pod, &in);
	step_for(w, 1);
	s = pr_pod_state(w, pod);
	bool burning = false;
	for (int i = 0; i < 6; i++) burning |= s->engine_on_fire[i];
	CHECK(!burning && s->repairing, "repair didn't put the fire out");
	in.repair = false;
	pr_pod_set_input(w, pod, &in);
	step_for(w, 6);
	CHECK(pr_pod_state(w, pod)->speed > 400, "doesn't recover after repair (%.0f)", pr_pod_state(w, pod)->speed);

	// Air brakes cancel a boost
	in.pitch = 1;
	pr_pod_set_input(w, pod, &in);
	step_for(w, 1.2f);
	in = (pr_input_t){.throttle = 1, .boost = true};
	pr_pod_set_input(w, pod, &in);
	pr_world_step(w, DT);
	CHECK(pr_pod_state(w, pod)->boosting, "second boost didn't fire");
	in.brake = true;
	pr_pod_set_input(w, pod, &in);
	pr_world_step(w, DT);
	CHECK(!pr_pod_state(w, pod)->boosting, "air brake doesn't cancel the boost");

	pr_world_destroy(w);
}

static void test_start_boost(void) {
	printf("\n== Start boost ==\n");
	plane_world_t pw = {0};
	float speed_at_1s[2];
	for (int k = 0; k < 2; k++) {
		pr_world_t *w = make_plane_world(&pw, 0.45f);
		pr_pod_stats_t st;
		pr_pod_stats_default(&st);
		pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, 0), v3(0, 0, 1));
		pr_pod_set_locked(w, pod, true);
		step_for(w, 1);
		pr_pod_set_locked(w, pod, false);
		// k = 0: press 0.1s after GO (inside the window); k = 1: 0.5s late
		step_for(w, k == 0 ? 0.1f : 0.5f);
		pr_input_t in = {.throttle = 1};
		pr_pod_set_input(w, pod, &in);
		step_for(w, 1);
		speed_at_1s[k] = pr_pod_state(w, pod)->speed;
		int n = count_events(w, PR_EVENT_START_BOOST, NULL);
		CHECK(n == (k == 0 ? 1 : 0), "start boost events: %d", n);
		pr_world_destroy(w);
	}
	printf("  speed 1s after pressing: %.0f with start boost, %.0f without\n", speed_at_1s[0], speed_at_1s[1]);
	CHECK(speed_at_1s[0] > speed_at_1s[1] + 40, "start boost too weak");
}


// Surfaces ---------------------------------------------------------------------

static float surface_speed(uint32_t flags, bool *respawned, float *temp) {
	plane_world_t pw = {.bands = {{3000, 1e9f, flags}}, .bands_len = 1};
	pr_world_t *w = make_plane_world(&pw, 1.0f);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, -2000), v3(0, 0, 1));
	pr_input_t in = {.throttle = 1};
	pr_pod_set_input(w, pod, &in);
	step_for(w, 12);
	int resp = 0;
	for (int i = 0; i < 9 * 60; i++) {
		pr_world_step(w, DT);
		resp += count_events(w, PR_EVENT_RESPAWNED, NULL);
	}
	if (respawned) *respawned = resp > 0;
	if (temp) *temp = pr_pod_state(w, pod)->engine_temp;
	float v = pr_pod_state(w, pod)->speed;
	pr_world_destroy(w);
	return v;
}

static void test_surfaces(void) {
	printf("\n== Surfaces ==\n");
	bool resp;
	float temp;
	float base = surface_speed(0, NULL, NULL);
	float fast = surface_speed(PR_SURFACE_FAST, NULL, NULL);
	float slow = surface_speed(PR_SURFACE_SLOW, NULL, NULL);
	float sand = surface_speed(PR_SURFACE_SWST, NULL, NULL);
	surface_speed(PR_SURFACE_LAVA, NULL, &temp);
	surface_speed(PR_SURFACE_FALL, &resp, NULL);
	printf("  speed: track %.0f  fast %.0f  slow %.0f  sand trap %.0f;  temp after 9s of lava %.0f\n", base, fast, slow, sand, temp);
	CHECK(fabsf(fast - base - 200) < 10, "fast surface %.0f vs %.0f", fast, base);
	CHECK(fabsf(slow / base - 0.75f) < 0.03f, "slow surface ratio %.2f", slow / base);
	CHECK(sand < base * 0.2f, "sand trap %.0f", sand);
	CHECK(temp < 25, "lava doesn't heat the engines (temp %.0f)", temp);
	CHECK(resp, "fall surface doesn't respawn");
}


// Air --------------------------------------------------------------------------

static void test_air(void) {
	printf("\n== Falling, landing & gliding (off a 300 unit cliff) ==\n");
	plane_world_t pw = {.drop_z = 3000, .drop_h = 300};
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	float air_time[2];
	for (int k = 0; k < 2; k++) {
		pr_world_t *w = make_plane_world(&pw, 1.0f);
		pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, 0), v3(0, 0, 1));
		pr_input_t in = {.throttle = 1};
		pr_pod_set_input(w, pod, &in);
		while (pr_pod_state(w, pod)->position.z < pw.drop_z) {
			pr_world_step(w, DT);
		}
		in.pitch = k ? -1.0f : 0.0f; // Nose up glides
		pr_pod_set_input(w, pod, &in);
		float t = 0, nose = 0;
		bool was_air = false;
		pr_event_t e = {0};
		int landed = 0;
		while (t < 6 && !landed) {
			pr_world_step(w, DT);
			t += DT;
			const pr_pod_state_t *s = pr_pod_state(w, pod);
			was_air |= s->airborne;
			nose = fminf(nose, asinf(s->forward.y) * 180 / PI);
			landed += count_events(w, PR_EVENT_LANDED, &e);
		}
		air_time[k] = t;
		const pr_pod_state_t *s = pr_pod_state(w, pod);
		printf("  %s: %.2fs in the air, nose down to %.0f deg, landing %.0f units/s\n", k ? "nose up" : "level  ", t, nose, e.intensity);
		CHECK(was_air && landed == 1, "no flight/landing");
		CHECK(vec_finite(s->position), "NaN");
		// The game drops the nose at 22 deg/s (to -37) while airborne
		if (!k) CHECK(fabsf(nose + 22 * air_time[0]) < 4, "nose %.0f deg after %.2fs", nose, air_time[0]);
		// The hover spring is only damped while rising above ride height, so
		// pods bob for a couple of seconds after a big landing, as in the game
		step_for(w, 4);
		CHECK(fabsf(pr_pod_state(w, pod)->ground_distance - st.hover_height) < 1.0f, "doesn't settle after landing (%.2f)", pr_pod_state(w, pod)->ground_distance);
		pr_world_destroy(w);
	}
	// 300 units at 960 units/s^2 is ~0.79s; nose up stretches it
	CHECK(fabsf(air_time[0] - 0.79f) < 0.1f, "fall time %.2fs", air_time[0]);
	CHECK(air_time[1] > air_time[0] * 1.5f, "nose up doesn't glide");
}


// Walls ------------------------------------------------------------------------

static void run_wall(float angle_deg, bool ai, float *retained, bool *tunneled, bool *exploded, int *respawned) {
	plane_world_t pw = {.wall = true, .wall_x = 0};
	pr_world_t *w = make_plane_world(&pw, 1.0f);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	// Accelerate for ~8s in a straight line that meets the wall at angle_deg
	float a = angle_deg * PI / 180;
	ag_vec3_t fwd = v3(-sinf(a), 0, cosf(a));
	pr_pod_id_t pod = pr_pod_add(w, &st, v3(3000 * sinf(a) + 30, 3, 0), fwd);
	if (ai) {
		// An AI with no route just holds the throttle
		pr_ai_params_t aip = {.level = 1};
		pr_pod_set_ai(w, pod, &aip);
	}
	pr_input_t in = {.throttle = 1};
	pr_pod_set_input(w, pod, &in);

	float before = 0, after = INFINITY;
	float hit_time = -1, t = 0;
	*tunneled = false;
	*exploded = false;
	*respawned = 0;
	while (t < 15 && (hit_time < 0 || t < hit_time + 0.5f)) {
		float speed_prev = pr_pod_state(w, pod)->speed;
		pr_world_step(w, DT);
		t += DT;
		const pr_pod_state_t *s = pr_pod_state(w, pod);
		if (s->position.x < 0) *tunneled = true;
		pr_event_t e;
		while (pr_world_poll_event(w, &e)) {
			if (e.type == PR_EVENT_WALL_HIT && hit_time < 0) {
				hit_time = t;
				before = speed_prev;
			}
			if (e.type == PR_EVENT_EXPLODED) *exploded = true;
			if (e.type == PR_EVENT_RESPAWNED) (*respawned)++;
		}
		if (hit_time >= 0 && t < hit_time + 0.2f && !s->dead) after = fminf(after, s->speed);
	}
	*retained = hit_time >= 0 ? (after < INFINITY ? after : 0) / before : 1;
	pr_world_destroy(w);
}

static void test_walls(void) {
	printf("\n== Walls ==\n");
	float angles[] = {5, 15, 30, 45, 60};
	for (int i = 0; i < 5; i++) {
		float retained;
		bool tunneled, exploded;
		int respawned;
		run_wall(angles[i], false, &retained, &tunneled, &exploded, &respawned);
		printf("  %2.0f deg: keeps %3.0f%% of its speed%s%s\n", angles[i], retained * 100, exploded ? ", EXPLODED" : "", tunneled ? ", TUNNELED" : "");
		CHECK(!tunneled, "tunneled at %.0f deg", angles[i]);
		if (angles[i] <= 5) CHECK(retained > 0.9f && !exploded, "glancing hit too harsh (%.2f)", retained);
		if (angles[i] <= 30) CHECK(!exploded, "a %.0f deg hit shouldn't be fatal", angles[i]);
		if (angles[i] >= 60) CHECK(exploded, "a head-on hit at full speed should blow the pod up");
	}
	// AI pods respawn where a player would explode
	float retained;
	bool tunneled, exploded;
	int respawned;
	run_wall(60, true, &retained, &tunneled, &exploded, &respawned);
	(void)retained;
	(void)respawned;
	CHECK(!exploded && !tunneled, "AI head-on: exploded %d tunneled %d", exploded, tunneled);
}


// Damage -----------------------------------------------------------------------

static void test_damage(void) {
	printf("\n== Engine damage ==\n");
	plane_world_t pw = {0};
	pr_world_t *w = make_plane_world(&pw, 1.0f);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	pr_pod_id_t pod = pr_pod_add(w, &st, v3(0, 3, 0), v3(0, 0, 1));
	pr_input_t in = {.throttle = 1};
	pr_pod_set_input(w, pod, &in);
	step_for(w, 5);
	// Shoot up the left engine (scaled by damage_immunity)
	for (int k = 0; k < 3; k++) pr_pod_damage(w, pod, k, 1.0f / st.damage_immunity);
	CHECK(count_events(w, PR_EVENT_ENGINE_DAMAGED, NULL) == 3, "damage events");
	float h0 = heading(pr_pod_state(w, pod)->forward);
	step_for(w, 0.5f);
	float drift = wrap_angle(heading(pr_pod_state(w, pod)->forward) - h0) * 180 / PI;
	step_for(w, 1.5f);
	const pr_pod_state_t *s = pr_pod_state(w, pod);
	printf("  left engine wrecked: heading drifts %.0f deg in 0.5s (turn rate %.0f), speed %.0f\n", drift, s->turn_rate, s->speed);
	// Left is +x in this right handed world: heading (atan2(x, z)) rises
	CHECK(drift > 10, "a dead left engine should pull left");
	CHECK(s->speed < 300, "a dead engine should cap the throttle");
	CHECK(s->boost_state == PR_BOOST_NOT_READY, "boost with a dead engine");
	pr_world_destroy(w);
}


// Stats ------------------------------------------------------------------------

static void put_u32(uint8_t *b, uint32_t v) {
	b[0] = v; b[1] = v >> 8; b[2] = v >> 16; b[3] = v >> 24;
}

static void test_stats(void) {
	printf("\n== Stats, upgrades & the exe loader ==\n");
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	pr_pod_stats_t up = st;
	st.max_speed = 420;
	up.max_speed = 420;
	pr_pod_stats_apply_upgrade(&up, PR_UPGRADE_TOP_SPEED, 5, 1.0f);
	pr_pod_stats_apply_upgrade(&up, PR_UPGRADE_ACCELERATION, 5, 1.0f);
	pr_pod_stats_apply_upgrade(&up, PR_UPGRADE_TRACTION, 3, 0.5f);
	pr_pod_stats_apply_upgrade(&up, PR_UPGRADE_AIR_BRAKE, 2, 1.0f);
	CHECK(up.max_speed == st.max_speed + 200, "top speed upgrade %.1f", up.max_speed);
	CHECK(fabsf(up.acceleration - st.acceleration * 0.3f) < 1e-4f, "acceleration upgrade %.3f", up.acceleration);
	CHECK(fabsf(up.anti_skid - (st.anti_skid + 0.075f)) < 1e-5f, "traction upgrade %.3f", up.anti_skid);
	CHECK(fabsf(up.air_brake_inv - st.air_brake_inv * 0.83f) < 1e-3f, "air brake upgrade %.3f", up.air_brake_inv);
	pr_pod_stats_t clamp = st;
	clamp.max_speed = 640;
	pr_pod_stats_apply_upgrade(&clamp, PR_UPGRADE_TOP_SPEED, 5, 1.0f);
	CHECK(clamp.max_speed == 650, "top speed clamp %.1f", clamp.max_speed);

	float bars[PR_NUM_UPGRADES];
	pr_pod_stats_default(&st);
	pr_pod_stats_bars(&st, bars);
	printf("  default pod bars:");
	for (int i = 0; i < PR_NUM_UPGRADES; i++) printf(" %.2f", bars[i]);
	printf("\n");
	CHECK(fabsf(bars[3] - (st.max_speed - 450) * 0.005f) < 1e-5f, "top speed bar");
	for (int i = 0; i < PR_NUM_UPGRADES; i++) CHECK(bars[i] >= 0.05f && bars[i] <= 1, "bar %d out of range", i);

	// A minimal PE32 with the stat table where the retail exe has it
	size_t size = 0x400 + 0x2000;
	uint8_t *exe = calloc(1, size);
	exe[0] = 'M'; exe[1] = 'Z';
	put_u32(exe + 0x3c, 0x80);
	memcpy(exe + 0x80, "PE\0\0", 4);
	exe[0x80 + 6] = 1;           // One section
	exe[0x80 + 20] = 0xe0;       // Optional header size
	uint8_t *opt = exe + 0x80 + 24;
	opt[0] = 0x0b; opt[1] = 0x01; // PE32
	put_u32(opt + 28, 0x00400000);
	uint8_t *sec = opt + 0xe0;
	memcpy(sec, ".data", 5);
	put_u32(sec + 8, 0x2000);     // Virtual size
	put_u32(sec + 12, 0xc2000);   // RVA
	put_u32(sec + 16, 0x2000);    // Raw size
	put_u32(sec + 20, 0x400);     // Raw offset
	size_t table = 0x400 + (0x4c2bb0 - 0x400000 - 0xc2000);
	for (int pod = 0; pod < PR_NUM_STOCK_PODS; pod++) {
		pr_pod_stats_t s = st;
		s.max_speed = 450 + pod * 5;
		memcpy(exe + table + pod * 60, &s, 60);
	}
	pr_pod_stats_t loaded;
	bool ok = pr_pod_stats_from_exe(exe, size, 14, &loaded);
	CHECK(ok && loaded.max_speed == 450 + 14 * 5 && loaded.anti_skid == st.anti_skid, "exe loader");
	printf("  synthetic exe: pod 14 (%s) max_speed %.0f\n", pr_pod_name(14), loaded.max_speed);
	CHECK(!pr_pod_stats_from_exe(exe, size, 23, &loaded), "pod index range");
	CHECK(!pr_pod_stats_from_exe(exe, 0x200, 0, &loaded), "truncated exe accepted");
	memset(exe + table, 0, 60);
	CHECK(!pr_pod_stats_from_exe(exe, size, 0, &loaded), "garbage stats accepted");
	exe[0] = 'X';
	CHECK(!pr_pod_stats_from_exe(exe, size, 1, &loaded), "non-PE accepted");
	free(exe);
}


// Race -------------------------------------------------------------------------

typedef struct {
	int laps[PR_MAX_PODS];
	float best[PR_MAX_PODS];
	int walls, pod_hits, respawns, explosions;
	bool finite;
} race_result_t;

static race_result_t run_race(test_world_t *tw, int pods, float seconds, float ai_spread) {
	pr_world_t *w = make_mesh_world(tw, 0.45f, true, ai_spread);
	pr_pod_stats_t st;
	pr_pod_stats_default(&st);
	for (int k = 0; k < pods; k++) {
		pr_pod_stats_t s = st;
		// A mixed field
		s.max_speed += (k % 3) * 20;
		s.acceleration *= 1.0f - (k % 2) * 0.15f;
		pr_pod_id_t id = pr_pod_add(w, &s, tw->route[0].position, v3(0, 0, 1));
		pr_ai_params_t ai = {.level = 1, .rank_baseline = k + 1, .pace_setter = k == 0};
		pr_pod_set_ai(w, id, &ai);
		pr_pod_place_on_grid(w, id, k);
		pr_pod_set_locked(w, id, true);
	}
	step_for(w, 1);
	for (int k = 0; k < pods; k++) pr_pod_set_locked(w, k, false);

	race_result_t r = {.finite = true};
	int n = (int)(seconds / DT);
	for (int i = 0; i < n; i++) {
		pr_world_step(w, DT);
		pr_event_t e;
		while (pr_world_poll_event(w, &e)) {
			if (e.type == PR_EVENT_WALL_HIT) r.walls++;
			if (e.type == PR_EVENT_POD_HIT) r.pod_hits++;
			if (e.type == PR_EVENT_RESPAWNED) r.respawns++;
			if (e.type == PR_EVENT_EXPLODED) r.explosions++;
		}
		for (int k = 0; k < pods; k++) {
			const pr_pod_state_t *s = pr_pod_state(w, k);
			if (!vec_finite(s->position) || !vec_finite(s->velocity) || !isfinite(s->speed)) r.finite = false;
		}
	}
	for (int k = 0; k < pods; k++) {
		r.laps[k] = pr_pod_state(w, k)->lap - 1;
		r.best[k] = pr_pod_state(w, k)->best_lap_time;
	}
	pr_world_destroy(w);
	return r;
}

static void test_race(void) {
	printf("\n== AI race (3km circuit, 0.45 m per Racer unit) ==\n");
	test_world_t tw;
	test_world_make_circuit_ex(&tw, true, 12.0f);

	race_result_t solo = run_race(&tw, 1, 90, 6);
	printf("  solo: %d laps in 90s, best %.2fs, %d wall hits, %d respawns\n", solo.laps[0], solo.best[0], solo.walls, solo.respawns);
	CHECK(solo.finite, "NaN in the solo race");
	CHECK(solo.laps[0] >= 5, "solo AI only did %d laps", solo.laps[0]);
	CHECK(solo.respawns == 0, "solo AI respawned %d times", solo.respawns);
	CHECK(solo.walls <= solo.laps[0] * 2, "solo AI hit the walls %d times", solo.walls);

	const int pods = 8;
	race_result_t pack = run_race(&tw, pods, 120, 6);
	int min_laps = 1000, max_laps = 0, total = 0;
	printf("  pack of %d, laps in 120s:", pods);
	for (int k = 0; k < pods; k++) {
		printf(" %d", pack.laps[k]);
		min_laps = pack.laps[k] < min_laps ? pack.laps[k] : min_laps;
		max_laps = pack.laps[k] > max_laps ? pack.laps[k] : max_laps;
		total += pack.laps[k];
	}
	printf("\n  %d wall hits, %d pod hits, %d respawns, %d explosions\n", pack.walls, pack.pod_hits, pack.respawns, pack.explosions);
	CHECK(pack.finite, "NaN in the pack race");
	CHECK(min_laps >= 5, "a pod only did %d laps", min_laps);
	CHECK(max_laps - min_laps <= 2, "field too spread: %d..%d laps", min_laps, max_laps);
	CHECK(pack.respawns <= pods, "%d respawns", pack.respawns);
	CHECK(pack.explosions == 0, "AI pods shouldn't explode");
	CHECK(pack.walls < total * 4, "%d wall hits in %d pod laps", pack.walls, total);

	// The game's own spread: the field strings out
	race_result_t spread = run_race(&tw, pods, 120, 30);
	printf("  game spread (30): laps");
	for (int k = 0; k < pods; k++) printf(" %d", spread.laps[k]);
	printf(", %d respawns\n", spread.respawns);
	CHECK(spread.finite && spread.respawns <= pods, "spread race: %d respawns", spread.respawns);

	test_world_free(&tw);
}


// Magnet -----------------------------------------------------------------------

static void test_magnet(void) {
	printf("\n== Magnet surface (wall ride down a 60m drop) ==\n");
	for (int k = 0; k < 2; k++) {
		test_world_t tw;
		test_world_make_drop(&tw, k == 1);
		pr_world_desc_t desc;
		pr_world_desc_default(&desc, 0.45f);
		desc.raycast = mesh_raycast;
		desc.user = &tw;
		desc.route = tw.route;
		desc.route_len = tw.route_len;
		desc.route_closed = false;
		pr_world_t *w = pr_world_create(&desc);
		pr_pod_stats_t st;
		pr_pod_stats_default(&st);
		pr_pod_id_t pod = pr_pod_add(w, &st, tw.route[0].position, v3(0, 0, 1));
		pr_input_t in = {.throttle = 1};
		pr_pod_set_input(w, pod, &in);
		float max_gd_on_wall = 0, t = 0;
		bool finite = true;
		while (t < 4) {
			pr_world_step(w, DT);
			t += DT;
			const pr_pod_state_t *s = pr_pod_state(w, pod);
			finite &= vec_finite(s->position);
			if (s->position.y < -5 && s->position.y > -50) max_gd_on_wall = fmaxf(max_gd_on_wall, s->ground_distance);
		}
		const pr_pod_state_t *s = pr_pod_state(w, pod);
		printf("  %s: max distance from the wall on the way down %.0f, ends at y %.1f\n", k ? "magnet" : "plain ", max_gd_on_wall, s->position.y);
		CHECK(finite, "NaN");
		if (k) CHECK(max_gd_on_wall < 25, "magnet doesn't hold the pod to the wall");
		else CHECK(max_gd_on_wall > 30 || max_gd_on_wall == 0, "plain surface shouldn't stick");
		pr_world_destroy(w);
		test_world_free(&tw);
	}
}


// Determinism ------------------------------------------------------------------

static void test_determinism(void) {
	printf("\n== Determinism across frame rates ==\n");
	test_world_t tw;
	test_world_make_circuit_ex(&tw, true, 12.0f);
	const float frame[3] = {1.0f / 60.0f, 1.0f / 144.0f, 1.0f / 30.0f};
	pr_pod_state_t result[3];
	const int substeps = 60 * 30;
	for (int m = 0; m < 3; m++) {
		pr_world_t *w = make_mesh_world(&tw, 0.45f, true, 6);
		pr_pod_stats_t st;
		pr_pod_stats_default(&st);
		for (int k = 0; k < 4; k++) {
			pr_pod_id_t id = pr_pod_add(w, &st, tw.route[0].position, v3(0, 0, 1));
			if (k) {
				pr_ai_params_t ai = {.level = 1, .rank_baseline = k, .pace_setter = k == 1};
				pr_pod_set_ai(w, id, &ai);
			}
			pr_pod_place_on_grid(w, id, k);
		}
		pr_input_t in = {.throttle = 1, .steer = 0.1f};
		pr_pod_set_input(w, 0, &in);
		int done = 0;
		while (done < substeps) {
			// Jitter the 30fps mode to exercise the accumulator
			float dt = frame[m];
			if (m == 2 && (done / 7) % 2) dt *= 0.7f;
			int max_left = substeps - done;
			float max_dt = max_left * (1.0f / 60.0f) - 1e-6f;
			done += pr_world_step(w, dt < max_dt ? dt : max_dt + 1e-7f);
		}
		result[m] = *pr_pod_state(w, 2);
		pr_world_destroy(w);
	}
	bool same = true;
	for (int m = 1; m < 3; m++) {
		same &= memcmp(&result[m].position, &result[0].position, sizeof(ag_vec3_t)) == 0;
		same &= result[m].speed == result[0].speed;
	}
	printf("  60 / 144 / jittered 30 fps after %d substeps: %s\n", substeps, same ? "identical" : "DIFFERENT");
	CHECK(same, "frame rate changes the simulation");
	test_world_free(&tw);
}


int main(void) {
	test_speed();
	test_turning();
	test_boost();
	test_start_boost();
	test_surfaces();
	test_air();
	test_walls();
	test_damage();
	test_stats();
	test_magnet();
	test_race();
	test_determinism();
	printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
	return failures ? 1 : 0;
}
