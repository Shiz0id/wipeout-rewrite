#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "podracer.h"

#if !defined(M_PI)
	#define M_PI 3.14159265358979323846
#endif

// Internals run in Racer units: positions and speeds as the game has them.
// The host's units are converted at the raycast/route/state boundary.
//
// "PORTED" marks logic taken from the decompiled game (addresses refer to the
// retail swep1rcr.exe as documented by SW_RACER_RE); "RECONSTRUCTED" marks
// behavior the source project hasn't decompiled yet, rebuilt from community
// and TAS documentation.


// Math -------------------------------------------------------------------------

typedef ag_vec3_t vec3_t;

static inline vec3_t vec3(float x, float y, float z) { return (vec3_t){x, y, z}; }
static inline vec3_t add(vec3_t a, vec3_t b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline vec3_t sub(vec3_t a, vec3_t b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline vec3_t mulf(vec3_t a, float f) { return vec3(a.x * f, a.y * f, a.z * f); }
static inline float dot(vec3_t a, vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float len(vec3_t a) { return sqrtf(dot(a, a)); }
static inline vec3_t cross(vec3_t a, vec3_t b) {
	return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline vec3_t lerp3(vec3_t a, vec3_t b, float t) { return add(a, mulf(sub(b, a), t)); }
static inline vec3_t normalize_or(vec3_t a, vec3_t fallback) {
	float l = len(a);
	return l > 1e-6f ? mulf(a, 1.0f / l) : fallback;
}
static inline vec3_t on_plane(vec3_t a, vec3_t n) { return sub(a, mulf(n, dot(a, n))); }

static vec3_t rotate(vec3_t a, vec3_t axis, float angle) {
	float c = cosf(angle), s = sinf(angle);
	return add(add(mulf(a, c), mulf(cross(axis, a), s)), mulf(axis, dot(axis, a) * (1 - c)));
}

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

// PORTED stdMath_Decelerator (0x00480640): the game's frame-rate independent decay factor
static inline float decelerator(float rate, float dt) {
	return 1.0f - (dt * 33.333336f) / (dt * 33.333336f + rate);
}

// Ease value towards target at rate units per second (swrRace_easeTraction)
static inline float ease(float value, float target, float rate, float dt) {
	if (value < target) {
		return minf(value + rate * dt, target);
	}
	return maxf(value - rate * dt, target);
}


// Types ------------------------------------------------------------------------

typedef struct {
	bool active;
	pr_pod_stats_t stats;
	pr_pod_state_t s;
	pr_input_t input;
	pr_input_t prev_input;

	// Racer units
	vec3_t pos, prev_pos;
	vec3_t fwd, up;             // Physics frame (no visual tilt)
	vec3_t surface_up;          // Live surface normal
	vec3_t velocity_dir;        // Traction state (swrRace.velocityDir)
	vec3_t velocity_slope;
	vec3_t velocity_bump;       // Pod collision velocity (velocityCollisionOpponent)
	vec3_t velocity;            // Last frame's velocity incl. gravity, for state

	float throttle;
	float accel_thrust;
	float boost_value;
	float speed_value;
	float speed_multiplier;
	float pace_multiplier;
	float slide2;
	float turn_rate_target;
	float contact_kick;
	float tilt_manual;          // -1..1
	float tilt_target;          // Bank into the turn (degrees)
	float pitch;                // Input pitch after clamping (+ = nose down)
	float zero_g_pitch_rate;

	float fall_velocity;
	float fall_step;
	float ground_dist;
	float fall_timer;
	bool hit_bottom;

	float surface_speed_bonus;
	float surface_speed_factor;
	float surface_grip_factor;
	uint32_t surface_flags;
	bool grounded;              // Within 15 units (terrain drag applies)

	bool braking;
	bool sliding;
	bool not_accel;
	bool throttle_settled;
	bool locked;
	float start_window;         // > 0: pressing the throttle now gives a start boost
	bool start_boost;
	float boost_charge;
	float throttle_release_time;
	float wall_hit_cooldown;
	float invincible;
	float dead_timer;
	float respawn_pending;

	float engine_damage_max[6];

	// AI
	bool is_ai;
	bool ai_tethered;           // One of the two AI nearest the leading human
	pr_ai_params_t ai;
	float ai_speed_target;
	float ai_decision_timer;
	int ai_rank_target;
	float ai_look_ahead;

	// Race
	int route_seg;
	int max_lap;
	float last_valid_progress;
	vec3_t last_ground_pos, last_ground_fwd; // Respawn point without a route
	float laps;                 // Progress in laps (lap count + fraction)

	vec3_t prev_pose_pos, prev_pose_fwd, prev_pose_up;
	float prev_tilt;
} pod_t;

struct pr_world_t {
	pr_world_desc_t desc;
	float scale;                // Host units per Racer unit
	float hand;
	vec3_t down;

	float accumulator;
	float time;
	uint32_t rng;

	pod_t pods[PR_MAX_PODS];

	// Route in Racer units
	ag_route_point_t *route;
	vec3_t *route_dir;
	float *route_dist;
	int route_len, route_segs;
	float route_length;

	pr_event_t events[PR_MAX_EVENTS];
	int events_head, events_len;
};


// Helpers ----------------------------------------------------------------------

static float world_rand(pr_world_t *w) {
	w->rng = w->rng * 1664525u + 1013904223u;
	return (w->rng >> 8) / 16777216.0f;
}

static void event_push(pr_world_t *w, pr_event_type_t type, int pod, int other, float intensity, float value, vec3_t pos) {
	if (w->events_len >= PR_MAX_EVENTS) {
		return;
	}
	w->events[(w->events_head + w->events_len) % PR_MAX_EVENTS] = (pr_event_t){
		.type = type, .pod = pod, .other = other, .intensity = intensity, .value = value,
		.position = mulf(pos, w->scale),
	};
	w->events_len++;
}

bool pr_world_poll_event(pr_world_t *w, pr_event_t *event) {
	if (w->events_len == 0) {
		return false;
	}
	*event = w->events[w->events_head];
	w->events_head = (w->events_head + 1) % PR_MAX_EVENTS;
	w->events_len--;
	return true;
}

// Raycast in Racer units through the host's callback (host units)
static bool world_raycast(pr_world_t *w, vec3_t origin, vec3_t dir, float max_dist, ag_hit_t *hit) {
	if (!w->desc.raycast) {
		return false;
	}
	if (!w->desc.raycast(w->desc.user, mulf(origin, w->scale), dir, max_dist * w->scale, hit)) {
		return false;
	}
	hit->position = mulf(hit->position, 1.0f / w->scale);
	hit->distance /= w->scale;
	return true;
}

// Raycast for the ground below, passing through walls (a wall leaning over
// the pod would otherwise read as no ground)
static bool world_ground_cast(pr_world_t *w, vec3_t origin, vec3_t down, float max_dist, ag_hit_t *hit) {
	float traveled = 0;
	for (int i = 0; i < 4; i++) {
		if (!world_raycast(w, origin, down, max_dist - traveled, hit)) {
			return false;
		}
		if (!(hit->surface_flags & PR_SURFACE_WALL) && dot(hit->normal, down) < -0.2f) {
			hit->distance += traveled;
			return true;
		}
		float skip = hit->distance + 0.05f;
		origin = add(origin, mulf(down, skip));
		traveled += skip;
	}
	return false;
}

static vec3_t pod_right(pr_world_t *w, pod_t *p) {
	return mulf(cross(p->up, p->fwd), w->hand);
}


// Pod stats --------------------------------------------------------------------

static const char *pod_names[PR_NUM_STOCK_PODS] = {
	"Anakin Skywalker", "Teemto Pagalies", "Sebulba", "Ratts Tyerell", "Aldar Beedo",
	"Mawhonic", "Ark 'Bumpy' Roose", "Wan Sandage", "Mars Guo", "Ebe Endocott",
	"Dud Bolt", "Gasgano", "Clegg Holdfast", "Elan Mak", "Neva Kee",
	"Bozzie Baranta", "Boles Roor", "Ody Mandrell", "Fud Sang", "Ben Quadinaros",
	"Slide Paramita", "Toy Dampner", "'Bullseye' Navior",
};

const char *pr_pod_name(int pod) {
	return pod >= 0 && pod < PR_NUM_STOCK_PODS ? pod_names[pod] : "";
}

void pr_pod_stats_default(pr_pod_stats_t *s) {
	// Mid-field values within the documented stat ranges
	*s = (pr_pod_stats_t){
		.anti_skid = 0.45f,
		.turn_response = 300.0f,
		.max_turn_rate = 100.0f,
		.acceleration = 1.5f,
		.max_speed = 500.0f,
		.air_brake_inv = 30.0f,
		.deceleration_interval = 30.0f,
		.boost_thrust = 200.0f,
		.heat_rate = 20.0f,
		.cool_rate = 7.0f,
		.hover_height = 6.0f,
		.repair_rate = 0.3f,
		.bump_mass = 50.0f,
		.damage_immunity = 0.3f,
		.intersect_radius = 2.0f,
	};
}

static uint32_t read_u32(const uint8_t *b) {
	return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static uint16_t read_u16(const uint8_t *b) {
	return (uint16_t)(b[0] | (b[1] << 8));
}

bool pr_pod_stats_from_exe(const void *exe, size_t size, int pod, pr_pod_stats_t *out) {
	// swrRacer_PodHandlingData @ 0x004c2bb0: PodHandlingData[23], 15 floats each
	const uint32_t table_va = 0x004c2bb0;
	const uint8_t *b = exe;
	if (!b || pod < 0 || pod >= PR_NUM_STOCK_PODS || size < 0x40 || b[0] != 'M' || b[1] != 'Z') {
		return false;
	}
	uint32_t pe = read_u32(b + 0x3c);
	if (pe + 24 > size || memcmp(b + pe, "PE\0\0", 4) != 0) {
		return false;
	}
	uint16_t sections = read_u16(b + pe + 6);
	uint16_t opt_size = read_u16(b + pe + 20);
	const uint8_t *opt = b + pe + 24;
	if (pe + 24 + opt_size > size || read_u16(opt) != 0x10b) { // PE32
		return false;
	}
	uint32_t image_base = read_u32(opt + 28);
	uint32_t rva = table_va - image_base;
	const uint8_t *sec = opt + opt_size;
	for (int i = 0; i < sections; i++, sec += 40) {
		if ((size_t)(sec - b) + 40 > size) {
			return false;
		}
		uint32_t vsize = read_u32(sec + 8), vaddr = read_u32(sec + 12);
		uint32_t raw_size = read_u32(sec + 16), raw_ptr = read_u32(sec + 20);
		uint32_t span = vsize > raw_size ? vsize : raw_size;
		if (rva < vaddr || rva >= vaddr + span) {
			continue;
		}
		size_t offset = (size_t)raw_ptr + (rva - vaddr) + (size_t)pod * 15 * 4;
		if (rva - vaddr + (size_t)PR_NUM_STOCK_PODS * 60 > raw_size || offset + 60 > size) {
			return false;
		}
		float v[15];
		for (int k = 0; k < 15; k++) {
			uint32_t bits = read_u32(b + offset + k * 4);
			memcpy(&v[k], &bits, 4);
		}
		// Sanity check against the known stat ranges
		if (!(v[0] > 0 && v[0] <= 1.0f) || !(v[4] >= 300 && v[4] <= 800) || !(v[3] > 0 && v[3] < 10)) {
			return false;
		}
		memcpy(out, v, sizeof(v));
		return true;
	}
	return false;
}

// PORTED swrRace_CalculateUpgradedStat (0x004493f0)
void pr_pod_stats_apply_upgrade(pr_pod_stats_t *s, pr_upgrade_t category, int level, float health) {
	if (level < 1 || level > 5) {
		return;
	}
	int l = level - 1;
	static const float traction[] = {0.05f, 0.1f, 0.15f, 0.2f, 0.25f};
	static const float turning[] = {116, 232, 348, 464, 578};
	static const float accel[] = {0.14f, 0.28f, 0.42f, 0.56f, 0.7f};
	static const float top[] = {40, 80, 120, 160, 200};
	static const float brake[] = {0.08f, 0.17f, 0.26f, 0.35f, 0.44f};
	static const float cooling[] = {1.6f, 3.2f, 4.8f, 6.4f, 8.0f};
	static const float repair[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.45f};
	switch (category) {
		case PR_UPGRADE_TRACTION: s->anti_skid = clampf(s->anti_skid + health * traction[l], 0.01f, 1.0f); break;
		case PR_UPGRADE_TURNING: s->turn_response = clampf(s->turn_response + health * turning[l], 50, 1000); break;
		case PR_UPGRADE_ACCELERATION: s->acceleration = clampf(((1 - health) * accel[l] + 1 - accel[l]) * s->acceleration, 0.1f, 5.0f); break;
		case PR_UPGRADE_TOP_SPEED: s->max_speed = clampf(s->max_speed + health * top[l], 450, 650); break;
		case PR_UPGRADE_AIR_BRAKE: s->air_brake_inv = clampf(((1 - health) * brake[l] + 1 - brake[l]) * s->air_brake_inv, 1, 1000); break;
		case PR_UPGRADE_COOLING: s->cool_rate = clampf(s->cool_rate + health * cooling[l], 1, 20); break;
		case PR_UPGRADE_REPAIR: s->repair_rate = clampf(s->repair_rate + health * repair[l], 0, 1); break;
		default: break;
	}
}

// PORTED swrRace_ComputeStatBars (0x00449c10)
void pr_pod_stats_bars(const pr_pod_stats_t *s, float bars[PR_NUM_UPGRADES]) {
	bars[0] = s->anti_skid;
	bars[1] = s->turn_response * 0.001f;
	bars[2] = 1.0f - sqrtf(s->acceleration) * 0.4761905f;
	bars[3] = (s->max_speed - 450.0f) * 0.005f;
	bars[4] = 8.0f / sqrtf(s->air_brake_inv * 0.5f) - 1.68f;
	bars[5] = s->cool_rate * 0.05f;
	bars[6] = s->repair_rate;
	for (int i = 0; i < PR_NUM_UPGRADES; i++) {
		bars[i] = clampf(bars[i], 0.05f, 1.0f);
	}
}


// Route ------------------------------------------------------------------------

static void *world_alloc(pr_world_t *w, size_t size) {
	return w->desc.alloc ? w->desc.alloc(size, w->desc.alloc_user) : malloc(size);
}

static void world_free(pr_world_t *w, void *p) {
	if (p) {
		if (w->desc.free) w->desc.free(p, w->desc.alloc_user);
		else free(p);
	}
}

static int route_next(pr_world_t *w, int i) {
	return (i + 1) % w->route_len;
}

static vec3_t route_up_at(pr_world_t *w, int i) {
	return normalize_or(w->route[i].up, mulf(w->down, -1));
}

static float route_wrap(pr_world_t *w, float d) {
	if (w->desc.route_closed) {
		d = fmodf(d, w->route_length);
		return d < 0 ? d + w->route_length : d;
	}
	return clampf(d, 0, w->route_length);
}

static int route_seg_at(pr_world_t *w, float d) {
	int lo = 0, hi = w->route_segs - 1;
	while (lo < hi) {
		int mid = (lo + hi + 1) / 2;
		if (w->route_dist[mid] <= d) lo = mid;
		else hi = mid - 1;
	}
	return lo;
}

static void route_sample(pr_world_t *w, float d, vec3_t *pos, vec3_t *dir, vec3_t *up, float *width) {
	d = route_wrap(w, d);
	int i = route_seg_at(w, d), j = route_next(w, i);
	float seg = w->route_dist[i + 1] - w->route_dist[i];
	float t = seg > 1e-6f ? clampf((d - w->route_dist[i]) / seg, 0, 1) : 0;
	if (pos) *pos = lerp3(w->route[i].position, w->route[j].position, t);
	if (dir) *dir = normalize_or(lerp3(w->route_dir[i], w->route_dir[j], t), w->route_dir[i]);
	if (up) *up = normalize_or(lerp3(route_up_at(w, i), route_up_at(w, j), t), mulf(w->down, -1));
	if (width) *width = lerpf(w->route[i].width, w->route[j].width, t);
}

static int route_nearest(pr_world_t *w, vec3_t p, int seg, float *progress, float *distance) {
	int first = 0, count = w->route_segs;
	if (seg >= 0) {
		count = w->route_segs < 13 ? w->route_segs : 13;
		first = seg - count / 2;
	}
	int best = 0;
	float best_dist = INFINITY, best_t = 0;
	for (int k = 0; k < count; k++) {
		int i = first + k;
		if (w->desc.route_closed) {
			i = (i % w->route_segs + w->route_segs) % w->route_segs;
		}
		else if (i < 0 || i >= w->route_segs) {
			continue;
		}
		vec3_t a = w->route[i].position;
		vec3_t ab = sub(w->route[route_next(w, i)].position, a);
		float ll = dot(ab, ab);
		float t = ll > 1e-9f ? clampf(dot(sub(p, a), ab) / ll, 0, 1) : 0;
		float dist = len(sub(p, add(a, mulf(ab, t))));
		if (dist < best_dist) {
			best_dist = dist;
			best = i;
			best_t = t;
		}
	}
	*progress = lerpf(w->route_dist[best], w->route_dist[best + 1], best_t);
	if (distance) *distance = best_dist;
	return best;
}

static void route_init(pr_world_t *w, const ag_route_point_t *points, int n) {
	if (!points || n < 2) {
		return;
	}
	w->route_len = n;
	w->route_segs = w->desc.route_closed ? n : n - 1;
	w->route = world_alloc(w, sizeof(ag_route_point_t) * n);
	w->route_dir = world_alloc(w, sizeof(vec3_t) * n);
	w->route_dist = world_alloc(w, sizeof(float) * (n + 1));
	for (int i = 0; i < n; i++) {
		w->route[i] = points[i];
		w->route[i].position = mulf(points[i].position, 1.0f / w->scale);
		w->route[i].width = points[i].width / w->scale;
	}
	w->route_dist[0] = 0;
	for (int i = 0; i < w->route_segs; i++) {
		w->route_dist[i + 1] = w->route_dist[i] + len(sub(w->route[route_next(w, i)].position, w->route[i].position));
	}
	w->route_length = w->route_dist[w->route_segs];
	for (int i = 0; i < n; i++) {
		int prev = i > 0 ? i - 1 : (w->desc.route_closed ? n - 1 : 0);
		int next = i < n - 1 ? i + 1 : (w->desc.route_closed ? 0 : n - 1);
		w->route_dir[i] = normalize_or(on_plane(sub(w->route[next].position, w->route[prev].position), route_up_at(w, i)), vec3(0, 0, 1));
	}
}


// World ------------------------------------------------------------------------

void pr_world_desc_default(pr_world_desc_t *d, float units_per_racer_unit) {
	memset(d, 0, sizeof(*d));
	d->units_per_racer_unit = units_per_racer_unit;
	d->gravity_dir = vec3(0, -1, 0);
	d->right_handed = true;
	d->fixed_dt = 1.0f / 60.0f;
	d->max_substeps = 8;
	d->route_closed = true;
	d->ai_level = 1.0f;
	d->ai_spread = 30.0f;
}

pr_world_t *pr_world_create(const pr_world_desc_t *desc) {
	pr_world_t *w = desc->alloc ? desc->alloc(sizeof(pr_world_t), desc->alloc_user) : malloc(sizeof(pr_world_t));
	if (!w) {
		return NULL;
	}
	memset(w, 0, sizeof(*w));
	w->desc = *desc;
	w->desc.route = NULL;
	if (w->desc.fixed_dt <= 0) w->desc.fixed_dt = 1.0f / 60.0f;
	if (w->desc.max_substeps <= 0) w->desc.max_substeps = 8;
	if (w->desc.units_per_racer_unit <= 0) w->desc.units_per_racer_unit = 1;
	w->scale = w->desc.units_per_racer_unit;
	w->hand = desc->right_handed ? -1.0f : 1.0f;
	w->down = normalize_or(desc->gravity_dir, vec3(0, -1, 0));
	w->rng = 0x5eb01ba;
	route_init(w, desc->route, desc->route_len);
	return w;
}

void pr_world_destroy(pr_world_t *w) {
	if (!w) {
		return;
	}
	world_free(w, w->route);
	world_free(w, w->route_dir);
	world_free(w, w->route_dist);
	world_free(w, w);
}

float pr_world_alpha(pr_world_t *w) {
	return clampf(w->accumulator / w->desc.fixed_dt, 0, 1);
}


// Engines ----------------------------------------------------------------------

// PORTED swrRace_TakeDamage (0x00474cd0)
static void pod_take_damage(pr_world_t *w, int id, pod_t *p, int engine, float amount) {
	if (p->invincible > 0 || p->s.dead || engine < 0 || engine >= 6 || amount <= 0) {
		return;
	}
	p->s.boosting = false;
	float before = p->s.engine_damage[engine];
	p->s.engine_damage[engine] = minf(p->stats.damage_immunity * amount + before, 1.0f);
	p->engine_damage_max[engine] = maxf(p->engine_damage_max[engine], p->s.engine_damage[engine]);
	if (p->s.engine_damage[engine] - before > 0.05f) {
		event_push(w, PR_EVENT_ENGINE_DAMAGED, id, PR_INVALID_POD, p->s.engine_damage[engine], (float)engine, p->pos);
	}
}

// PORTED swrRace_GetDamagedEngineSides (0x0046a9c0): 1 = left, 2 = right.
// The game's "damaged" status bits are set by fires and destroyed parts.
static int pod_damaged_sides(pod_t *p) {
	int sides = 0;
	for (int i = 0; i < 6; i++) {
		if (p->s.engine_on_fire[i] || p->s.engine_damage[i] >= 1.0f) {
			sides |= i < 3 ? 1 : 2;
		}
	}
	return sides;
}

// PORTED swrRace_GetEngineDamagePenalty (0x0046a9f0)
static float pod_damage_penalty(pod_t *p) {
	float penalty = 0;
	for (int i = 0; i < 6; i++) {
		if (p->s.engine_damage[i] > 0.8f) {
			penalty += i < 3 ? -0.33f : 0.33f;
		}
	}
	return penalty;
}

// PORTED swrRace_UpdateHeat (0x004788c0)
static void pod_update_heat(pr_world_t *w, int id, pod_t *p, float dt) {
	int spin_dir = 0;
	if (p->surface_flags & PR_SURFACE_LAVA) {
		p->s.engine_temp -= dt * 20.0f;
		spin_dir = p->tilt_manual < -0.5f ? -1 : (p->tilt_manual > 0.5f ? 1 : 0);
	}
	else if (p->s.boosting) {
		p->s.engine_temp -= dt * p->stats.heat_rate;
	}
	else {
		p->s.engine_temp += dt * p->stats.cool_rate;
	}
	p->s.engine_temp = minf(p->s.engine_temp, 100.0f);
	if (p->s.engine_temp > 0) {
		return;
	}

	// Overheated: a random engine part (biased by the tilt) catches fire
	p->s.engine_temp = 0;
	int part;
	if (spin_dir < 0) part = (int)(world_rand(w) * 3);
	else if (spin_dir > 0) part = 3 + (int)(world_rand(w) * 3);
	else part = (int)(world_rand(w) * 6);
	part = part > 5 ? 5 : part;
	if (!p->s.engine_on_fire[part]) {
		event_push(w, PR_EVENT_OVERHEAT, id, PR_INVALID_POD, 0, (float)part, p->pos);
	}
	p->s.engine_on_fire[part] = true;
	p->s.boosting = false;
}

// RECONSTRUCTED: repairing while driving. The game's Repair stat "fixes damage
// faster and costs less speed"; burning parts are put out once repaired.
static void pod_repair(pod_t *p, float dt) {
	if (!p->s.repairing) {
		return;
	}
	float heal = (0.15f + 0.6f * p->stats.repair_rate) * dt;
	for (int i = 0; i < 6; i++) {
		p->s.engine_damage[i] = maxf(p->s.engine_damage[i] - heal, 0);
		if (p->s.engine_damage[i] < 0.5f) {
			p->s.engine_on_fire[i] = false;
		}
	}
}


// Control ----------------------------------------------------------------------

// PORTED swrRace_Tilt (0x0046b5a0)
static void pod_tilt(pod_t *p, float target, float dt) {
	if (p->speed_value < 200.0f) {
		target = 0;
	}
	p->tilt_manual = ease(p->tilt_manual, target, 3.2f, dt);
	if (target == 0 && fabsf(p->tilt_manual) < 0.1f) {
		p->tilt_manual *= 0.5f;
	}
}

// PORTED swrRace_UpdatePlayerControl (0x0046bec0), minus input device plumbing
static void pod_control(pr_world_t *w, int id, pod_t *p, const pr_input_t *in, float dt) {
	p->throttle = 0;
	p->turn_rate_target = 0;
	if (p->s.dead || p->locked) {
		p->braking = false;
		p->s.boosting = false;
		return;
	}

	float steer = clampf(in->steer, -1, 1);
	float pitch = clampf(in->pitch * 0.8f, -0.8f, 0.8f);
	bool thrust = in->throttle >= 0.99f;
	bool bank_left = in->tilt_left, bank_right = in->tilt_right;

	int damaged = pod_damaged_sides(p);
	if (damaged) {
		bank_left = bank_right = false;
		p->s.boosting = false;
		p->s.boost_state = PR_BOOST_NOT_READY;
		p->boost_charge = 0;
	}

	// Damaged engines pull the steering towards the dead side
	if (p->throttle_settled && !p->braking) {
		// The game collapses the side mask to 1 first, so the "left" offset
		// applies whichever side is damaged
		float penalty = pod_damage_penalty(p) + (damaged ? 0.2f : 0.0f);
		penalty *= 1.5f;
		steer += penalty * 0.6f;
	}

	// Repair while holding the button (not while boosting)
	p->s.repairing = in->repair && !p->s.boosting;
	pod_repair(p, dt);

	// Zero-g: the pitch stick becomes a pitch rate demand
	float zero_g_target = 0;
	if (p->s.zero_g) {
		if (fabsf(pitch) >= 0.05f) {
			float sq = (pitch * 1.25f) * (pitch * 1.25f) * (pitch < 0 ? -1 : 1);
			zero_g_target = -(p->stats.max_turn_rate * sq * 0.8f) * 0.5f;
		}
		pitch = 0;
	}
	p->zero_g_pitch_rate = ease(p->zero_g_pitch_rate, zero_g_target, p->stats.turn_response, dt);

	// Start boost: a fresh press of the throttle inside the window after GO
	bool thrust_pressed = thrust && p->prev_input.throttle < 0.99f;
	if (p->start_window > 0) {
		p->start_window -= dt;
		if (thrust_pressed) {
			p->start_boost = true;
			p->start_window = 0;
			event_push(w, PR_EVENT_START_BOOST, id, PR_INVALID_POD, 0, 0, p->pos);
		}
	}
	if (p->start_boost && (!thrust || p->speed_value > 290.0f)) {
		p->start_boost = false;
	}

	// Air brake
	p->braking = in->brake;
	if (p->braking && p->speed_value < 70.0f) {
		p->throttle_settled = false;
	}

	// Throttle: analog below full, 1.0 at full (0.5 with a damaged side),
	// idle 0.1 when off
	if (thrust) {
		p->throttle = damaged ? 0.5f : 1.0f;
	}
	else if (in->throttle > 0.05f) {
		p->throttle = clampf(in->throttle, 0, 1);
	}
	else if (pitch >= -0.5f || p->speed_value >= 20.0f) {
		p->throttle = 0.1f;
	}
	else {
		p->throttle = pitch * 0.4f;
	}
	if (p->wall_hit_cooldown <= 0 && p->throttle > 0.3f) {
		p->throttle_settled = true;
	}

	// RECONSTRUCTED: repairing costs speed, less with a better Repair stat
	if (p->s.repairing) {
		p->throttle = minf(p->throttle, 0.45f + 0.4f * p->stats.repair_rate);
	}

	// Sliding (forced while airborne)
	p->sliding = p->s.airborne || in->slide;
	p->not_accel = !(thrust || in->throttle > 0.5f);

	// RECONSTRUCTED boost charge (per TAS notes): hold full throttle with the
	// nose down for about a second to charge; then press boost, or release
	// and re-press the throttle, to fire it
	if (!p->s.boosting && !damaged && thrust && pitch > 0.3f && p->throttle_settled && p->s.boost_state != PR_BOOST_READY) {
		p->boost_charge += dt;
		p->s.boost_state = PR_BOOST_CHARGING;
		if (p->boost_charge >= 1.0f) {
			p->s.boost_state = PR_BOOST_READY;
			event_push(w, PR_EVENT_BOOST_READY, id, PR_INVALID_POD, 0, 0, p->pos);
		}
	}
	else if (p->s.boost_state == PR_BOOST_CHARGING) {
		p->s.boost_state = PR_BOOST_NOT_READY;
		p->boost_charge = 0;
	}
	if (!thrust) {
		p->throttle_release_time += dt;
	}
	bool refire = thrust_pressed && p->throttle_release_time < 0.3f;
	if (thrust) {
		p->throttle_release_time = 0;
	}
	bool boost_pressed = in->boost && !p->prev_input.boost;
	if (p->s.boost_state == PR_BOOST_READY && (boost_pressed || refire) && !p->s.repairing) {
		p->s.boosting = true;
		p->s.boost_state = PR_BOOST_NOT_READY;
		p->boost_charge = 0;
		event_push(w, PR_EVENT_BOOST, id, PR_INVALID_POD, 0, 0, p->pos);
	}
	if (p->s.boosting && !thrust && in->throttle <= 0.5f) {
		p->s.boosting = false;
	}

	// Lean
	pod_tilt(p, bank_left && !bank_right ? -1.0f : (bank_right && !bank_left ? 1.0f : 0), dt);

	// Turn rate target from the squared steering input. Racer's sign: positive
	// turn rate turns left, so steering right gives a negative target.
	if (fabsf(steer) >= 0.05f) {
		float sq = (steer * 1.25f) * (steer * 1.25f) * (steer < 0 ? -1 : 1);
		p->turn_rate_target = -(p->stats.max_turn_rate * sq * 0.8f);
	}

	// Nose down: less turning, more throttle; nose up: more turning, less
	// throttle (on the ground)
	p->pitch = pitch;
	if (pitch > 0.1f || pitch < -0.1f) {
		p->turn_rate_target *= 1.0f - pitch * 0.4f;
		if (p->throttle > 0.5f && (pitch > 0 || !p->s.airborne)) {
			p->throttle += pitch * 0.4f;
		}
	}
}


// AI ---------------------------------------------------------------------------

// PORTED swrRace_AI (0x0046b670): pacing only (speed multiplier), no driving
static void pod_ai_pace(pr_world_t *w, pod_t *p, float dt, float gap_to_pacer, float gap_to_human) {
	p->ai_speed_target = w->desc.ai_level * p->ai.level;
	float inv_track = 500000.0f / maxf(w->route_length, 1);
	float spread = w->desc.ai_spread * 0.0001f;
	float band = spread * inv_track;

	if (p->ai.pace_setter) {
		if (p->s.rank == 1) p->ai_speed_target *= 1.06f;
		else if (gap_to_human > band * 3.0f) p->ai_speed_target *= 1.4f;
		else p->ai_speed_target *= 1.1f;
	}
	else if (p->s.rank != 1) {
		p->ai_decision_timer -= dt;
		if (p->ai_decision_timer < 0) {
			p->ai_decision_timer = world_rand(w) * 10.0f + 8.0f;
			float r = world_rand(w);
			if (r < 0.15f && p->ai_rank_target - 1 >= 2 && abs(p->ai_rank_target - 1 - p->ai.rank_baseline) <= 2) {
				p->ai_rank_target--;
			}
			else if (r > 0.85f && abs(p->ai_rank_target + 1 - p->ai.rank_baseline) <= 2) {
				p->ai_rank_target++;
			}
		}
		float offset = ((p->ai_rank_target - 1.0f) * spread - 0.0008f) * inv_track;
		float v;
		if (p->ai_tethered && gap_to_pacer > band * 0.25f) {
			// Tethered to the human: pace directly on the gap to them
			v = gap_to_human > 0 ? gap_to_human * 10.3f : gap_to_human * 10.02f;
		}
		else {
			v = (gap_to_pacer - offset) * 10.0f;
		}
		float target = v * 40.0f / inv_track + 1.045f;
		p->ai_speed_target = clampf(target, 0.5f, 1.6f) * p->ai.level;
	}
	p->pace_multiplier = ease(p->pace_multiplier, p->ai_speed_target, 0.2f, dt);
}

// PORTED swrRace_AutopilotSteer (0x0046af40) and swrRace_ApplyPodProximityForce
// (0x0046b430), steering along the route instead of the track spline
static void pod_ai_control(pr_world_t *w, int id, pod_t *p, pr_input_t *in, float *turn_target) {
	*in = (pr_input_t){0};
	*turn_target = 0;
	if (!w->route || p->s.dead) {
		return;
	}
	in->throttle = 1;
	if (p->locked) {
		return;
	}

	// Look ahead ~90 units along the route, lifted to the pod's height
	float progress = p->s.route_progress / w->scale;
	vec3_t target;
	route_sample(w, progress + p->ai_look_ahead, &target, NULL, NULL, NULL);
	vec3_t aim = add(target, mulf(w->down, dot(sub(p->pos, target), w->down)));
	vec3_t dir = sub(aim, p->pos);
	float l = len(dir);
	float throttle_scale = p->throttle_settled ? 0.98f : 0.48f;

	// RECONSTRUCTED: the game's AI splines carry speed hints; a host route
	// doesn't, so plan speed from the route's curvature against what the
	// pod can turn, and coast or air brake in time for the corner
	float speed = maxf(p->speed_value, 1);
	float turn_cap = deg2rad(p->stats.max_turn_rate * 1.5f * p->pace_multiplier * 0.8f);
	float allowed = INFINITY;
	vec3_t prev_dir;
	route_sample(w, progress, NULL, &prev_dir, NULL, NULL);
	for (float d = 20.0f; d <= speed * 1.5f + 40.0f; d += 20.0f) {
		vec3_t dir_d, up_d;
		route_sample(w, progress + d, NULL, &dir_d, &up_d, NULL);
		vec3_t a = on_plane(prev_dir, up_d), b = on_plane(dir_d, up_d);
		float bend = atan2f(len(cross(a, b)), dot(a, b));
		prev_dir = dir_d;
		// Using the track's width straightens the line somewhat
		float curvature = bend / 20.0f * 0.75f;
		if (curvature < 1e-5f) continue;
		float corner_speed = turn_cap / curvature;
		allowed = minf(allowed, corner_speed + 150.0f * (d - 20.0f) / speed);
	}
	if (speed > allowed + 15.0f) {
		in->brake = true;
		throttle_scale = 0;
	}
	else if (speed > allowed) {
		throttle_scale = 0;
	}

	if (l > 0.01f) {
		dir = mulf(dir, 1.0f / l);
		// Steer on the direction of travel, not the nose: a drifting pod
		// otherwise keeps turning into the slide
		vec3_t travel = normalize_or(on_plane(p->velocity_dir, p->up), p->fwd);
		float err = dot(dir, mulf(cross(p->up, travel), w->hand)); // + = target to the right
		float steer = 0;
		if (fabsf(err) > 0.02f) {
			steer = err / 0.7f;
			if (fabsf(err) > 0.3f) {
				throttle_scale *= (1.0f - fabsf(err)) * 0.5f;
			}
		}
		// RECONSTRUCTED: feed forward the route's own bend just ahead. The
		// game's spline already is the racing line; steering on the look-ahead
		// error alone turns a third too little and runs wide in corners.
		vec3_t d0, d1, u0;
		route_sample(w, progress + 10.0f, NULL, &d0, &u0, NULL);
		route_sample(w, progress + 50.0f, NULL, &d1, NULL, NULL);
		float bend_right = dot(d1, mulf(cross(u0, d0), w->hand)); // ~ sin(bend), + = turns right
		float ff = rad2deg(speed * asinf(clampf(bend_right, -1, 1)) / 40.0f) / 1.5f;
		// Scaled by the pace in pod_step, like the game's throttle
		*turn_target = -p->stats.max_turn_rate * clampf(steer, -1, 1) - ff / maxf(p->pace_multiplier, 0.5f);
	}
	in->throttle = throttle_scale; // Below full: the analog path in pod_control
	if (throttle_scale >= 0.98f) {
		in->throttle = 1.0f;
	}

	// Steer away from the nearest pod within 50 units; block a lone human
	// closing from behind
	int nearest = -1, nearby = 0;
	float nearest_dist = 50.0f;
	for (int b = 0; b < PR_MAX_PODS; b++) {
		pod_t *o = &w->pods[b];
		if (b == id || !o->active || o->s.dead) continue;
		float d = len(sub(o->pos, p->pos));
		if (d < 50.0f) {
			nearby++;
			if (d < nearest_dist) {
				nearest_dist = d;
				nearest = b;
			}
		}
	}
	if (nearest >= 0) {
		pod_t *o = &w->pods[nearest];
		vec3_t delta = sub(o->pos, p->pos);
		float closeness = (50.0f - nearest_dist) * 0.2f;
		float force = closeness * closeness * 0.8f;
		float side = dot(delta, pod_right(w, p)); // + = other pod on the right
		float sign = 1;
		if (nearby == 1 && !o->is_ai && dot(delta, p->fwd) < 0) {
			sign = -1; // Block
		}
		// Steering away from a pod on the right means turning left (+)
		float push = side > 0 ? sign * force : (side < 0 ? -sign * force : 0);
		// RECONSTRUCTED: don't get pushed into the wall (the game's lanes
		// keep clear of it; a route only knows its width)
		vec3_t center, rdir, rup;
		float width;
		route_sample(w, progress, &center, &rdir, &rup, &width);
		float lateral = dot(sub(p->pos, center), mulf(cross(rup, rdir), w->hand)); // + = right of center
		float room = width * 0.5f - fabsf(lateral) - p->stats.intersect_radius * 3.0f;
		bool towards_edge = (push < 0) == (lateral > 0); // Turning right (-) moves right
		if (towards_edge) {
			push *= clampf(room / 15.0f, 0, 1);
		}
		*turn_target += push;
	}

	// Tilt into banked side surfaces
	in->tilt_right = (p->surface_flags & PR_SURFACE_SIDE) != 0;
}


// Physics ----------------------------------------------------------------------

// PORTED swrRace_UpdateSpeed (0x004783e0)
static float pod_update_speed(pod_t *p, float dt) {
	float accel = (p->s.boosting || p->start_boost) ? 4.0f : 1.5f;
	if (p->throttle <= 0.1f) {
		if (-0.1f <= p->throttle) {
			if (p->is_ai || p->accel_thrust >= 0.2f) {
				p->accel_thrust *= decelerator(p->stats.deceleration_interval, dt);
			}
			else {
				p->accel_thrust *= decelerator(10.0f, dt);
			}
		}
		else {
			float v = dt * accel * p->throttle + p->accel_thrust;
			p->accel_thrust = v;
			if (-0.6f < p->throttle && v < p->throttle * 0.5f) {
				p->accel_thrust *= decelerator(20.0f, dt);
			}
		}
	}
	else {
		float v = dt * accel * p->throttle + p->accel_thrust;
		p->accel_thrust = v;
		float ceiling = p->throttle < 0.99f ? p->throttle / (1.0f - p->throttle) : 10000.0f;
		if (ceiling < v) {
			p->accel_thrust *= decelerator(p->stats.deceleration_interval, dt);
		}
	}
	if (p->braking) {
		p->accel_thrust *= decelerator(p->stats.air_brake_inv, dt);
	}

	float speed;
	if (p->accel_thrust <= 0) {
		speed = -((-p->accel_thrust * p->stats.max_speed) / (p->stats.acceleration - p->accel_thrust));
	}
	else {
		speed = (p->accel_thrust * p->stats.max_speed) / (p->stats.acceleration + p->accel_thrust);
	}
	speed *= p->speed_multiplier;

	if (p->ground_dist < 15.0f) {
		p->grounded = true;
		speed *= p->surface_speed_factor;
	}
	else {
		p->grounded = false;
	}
	speed += p->surface_speed_bonus;

	if (p->is_ai && p->pitch < -0.5f) {
		speed *= 1.3f; // Glide dive back onto the line
	}
	return speed;
}

// PORTED swrRace_ApplyBoost (0x004787f0)
static float pod_apply_boost(pod_t *p, float dt) {
	if (!p->s.boosting) {
		if (p->boost_value > 0) p->boost_value *= decelerator(5.0f, dt);
		if (p->boost_value < 0.001f) p->boost_value = 0;
	}
	else {
		p->boost_value += dt * 1.5f;
	}
	if (p->braking) {
		p->s.boosting = false;
	}
	return p->boost_value > 0 ? (p->boost_value * p->stats.boost_thrust) / (p->boost_value + 0.33f) : 0;
}

// PORTED swrRace_ApplyTraction (0x00478a70)
static vec3_t pod_apply_traction(pod_t *p, float speed, vec3_t dir, float dt) {
	vec3_t d = speed > 0 ? dir : mulf(dir, -1);
	float opposing = dot(d, p->velocity_dir);
	if (opposing < 0) {
		p->velocity_dir = add(p->velocity_dir, mulf(d, -opposing));
	}
	vec3_t desired = mulf(dir, speed);

	float grip = p->stats.anti_skid * p->surface_grip_factor * p->slide2;
	float traction = (1.0f - grip * grip) * 0.99666601f;
	if (p->pace_multiplier > 1.0f) {
		traction = p->pace_multiplier <= 2.0f ? (2.0f - p->pace_multiplier) * traction : 0;
	}

	// The game blends once per frame at ~30 fps; scale the blend to dt so
	// it behaves the same at any step rate
	float keep = powf(traction, dt * 30.0f);
	p->velocity_dir = add(mulf(desired, 1.0f - keep), mulf(p->velocity_dir, keep));

	vec3_t out = mulf(normalize_or(p->velocity_dir, dir), fabsf(speed));

	float target = p->not_accel ? 0.8f : 1.0f;
	if (p->sliding) target *= 0.45f;
	p->slide2 = p->slide2 < target ? minf(p->slide2 + dt * 2.0f, target) : maxf(p->slide2 - dt * 2.0f, target);
	return out;
}

// PORTED swrRace_UpdateSurfaceTag (0x00476ea0)
static void pod_update_surface(pr_world_t *w, int id, pod_t *p, uint32_t flags, float dt) {
	float bonus_target = 0, factor_target = 1, grip_target = 1;
	if (p->s.zero_g && p->speed_value < 75.0f) {
		bonus_target = 75.0f - p->speed_value;
	}
	if (flags & PR_SURFACE_ZON) p->s.zero_g = true;
	if ((flags & PR_SURFACE_ZOFF) && p->s.zero_g) {
		p->s.zero_g = false;
		p->velocity_slope = vec3(0, 0, 0);
	}
	if (flags & PR_SURFACE_FAST) bonus_target = 200.0f;
	if (flags & PR_SURFACE_SLOW) {
		factor_target = 0.75f;
		if (p->s.zero_g) p->s.boosting = false;
	}
	if (flags & PR_SURFACE_SWST) {
		factor_target = 0.1f;
		p->s.boosting = false;
	}
	if (flags & PR_SURFACE_SLIP) grip_target = 0.2f;

	p->surface_speed_bonus = ease(p->surface_speed_bonus, bonus_target, 25.0f, dt);
	p->surface_speed_factor = ease(p->surface_speed_factor, factor_target, 0.5f, dt);
	p->surface_grip_factor = ease(p->surface_grip_factor, grip_target, 0.5f, dt);

	uint32_t tags = flags & 0xfff00u;
	if (tags != (p->surface_flags & 0xfff00u)) {
		event_push(w, PR_EVENT_SURFACE, id, PR_INVALID_POD, 0, (float)tags, p->pos);
	}
	p->surface_flags = flags;
	p->s.magnet = (flags & PR_SURFACE_MAGNET) != 0;

	// Touching a pit floor (not flying over it)
	if ((flags & PR_SURFACE_FALL) && p->ground_dist < p->stats.hover_height * 2.0f) {
		p->respawn_pending = 0.01f;
	}
}

// PORTED swrRace_ApplySlopeSteering (0x004791d0): the downhill slide
static void pod_slope(pr_world_t *w, pod_t *p, vec3_t normal, float dt) {
	float d = dot(normal, w->down);
	if (d < -0.995f || d > 0.995f) {
		p->velocity_slope = mulf(p->velocity_slope, 0.9f);
		return;
	}
	vec3_t axis = cross(normal, w->down);
	vec3_t downhill = normalize_or(cross(normal, axis), vec3(0, 0, 0));
	float slope_sin = dot(downhill, w->down);
	float steer = p->ground_dist <= 50.0f ? (1.0f - p->ground_dist * 0.02f) * slope_sin : 0;
	float force = steer * steer * 400.0f;
	vec3_t out = add(p->velocity_slope, mulf(downhill, force * 2.0f * dt));
	float out_len = len(out);
	if (force < out_len && out_len > 1e-6f) {
		out = mulf(out, force / out_len);
	}
	p->velocity_slope = out;
}

// PORTED swrRace_ApplyGravity (0x004774f0): hover via the fall accumulator.
// Moves the pod along `down`; returns true on a hard landing.
static void pod_gravity(pr_world_t *w, int id, pod_t *p, vec3_t down, float dt) {
	float ground = p->ground_dist - p->stats.intersect_radius;
	float hover = p->stats.hover_height - p->stats.intersect_radius;

	if (p->ground_dist > 99999.0f) {
		p->fall_timer += dt;
		if (p->fall_timer > 3.0f) {
			p->respawn_pending = 0.01f;
		}
	}
	else {
		p->fall_timer = 0;
	}
	p->s.airborne = p->ground_dist > 30.0f;

	if (ground <= 12.0f) {
		p->fall_velocity += (1.0f - (12.0f - ground) / (12.0f - hover)) * dt;
		if (hover < ground && p->fall_velocity < 0) {
			p->fall_velocity *= decelerator(4.0f, dt);
		}
	}
	else if (p->speed_value < 0) {
		p->fall_velocity += dt * 2.0f;
	}
	else {
		p->fall_velocity += dt;
	}

	float step = dt * 32.0f * p->fall_velocity * 30.0f;
	if (p->pitch < 0 && p->speed_value >= 0 && step > 0) {
		step *= p->pitch * 0.9f + 1.0f; // Nose up glides
	}

	if (step <= ground) {
		p->hit_bottom = false;
	}
	else {
		float impact = step / dt * 0.5f;
		float bounce = p->fall_velocity * 8.0f;
		step = ground;
		if (p->fall_velocity > 0) {
			p->fall_velocity *= -0.2f;
		}
		if (bounce > 4.0f && !p->hit_bottom) {
			event_push(w, PR_EVENT_LANDED, id, PR_INVALID_POD, impact, 0, p->pos);
		}
		p->hit_bottom = true;
	}
	p->fall_step = step;
	p->pos = add(p->pos, mulf(down, step));
}

// PORTED swrRace_DeathSpeed (0x0047b000) thresholds: impact speed above 325
// with a speed drop above 140 explodes a pod going 200+; otherwise respawn
static void pod_crash(pr_world_t *w, int id, pod_t *p, float impact_speed, float speed_drop) {
	if (p->invincible > 0 || p->s.dead) {
		return;
	}
	if (speed_drop > 140.0f && impact_speed > 325.0f) {
		if (p->speed_value >= 200.0f && !p->is_ai) {
			p->s.dead = true;
			p->dead_timer = 2.0f + world_rand(w);
			p->s.boosting = false;
			event_push(w, PR_EVENT_EXPLODED, id, PR_INVALID_POD, impact_speed, 0, p->pos);
		}
		else {
			p->respawn_pending = 0.01f;
		}
	}
}

// RECONSTRUCTED wall collision (swrRace_DetectWallScrape/ApplyWallCollision
// aren't decompiled yet): probes around the pod, push out, lose thrust by the
// impact angle, damage the engines on that side, kick the nose away, and run
// the game's crash/death rule
static void pod_walls(pr_world_t *w, int id, pod_t *p, vec3_t prev_pos, vec3_t plane_n) {
	float hx = p->stats.intersect_radius * 3.0f;
	float hz = p->stats.intersect_radius * 6.0f;
	vec3_t fwd = normalize_or(on_plane(p->fwd, plane_n), p->fwd);
	vec3_t side = cross(plane_n, fwd);

	// Continuous check along the motion
	vec3_t move = sub(p->pos, prev_pos);
	float ml = len(move);
	vec3_t wall_n = vec3(0, 0, 0);
	float best_pen = 0;
	ag_hit_t wall_hit = {0};
	if (ml > 1e-4f) {
		vec3_t dir = mulf(move, 1.0f / ml);
		ag_hit_t hit;
		if (world_raycast(w, prev_pos, dir, ml + hx, &hit) && ((hit.surface_flags & PR_SURFACE_WALL) || dot(hit.normal, plane_n) < 0.5f)) {
			vec3_t n = normalize_or(on_plane(hit.normal, plane_n), mulf(dir, -1));
			float pen = (ml + hx - hit.distance) * maxf(0.25f, -dot(dir, n));
			if (pen > 0) {
				p->pos = add(p->pos, mulf(n, pen));
				best_pen = pen;
				wall_n = n;
				wall_hit = hit;
			}
		}
	}

	vec3_t pushed = vec3(0, 0, 0);
	for (int i = 0; i < 8; i++) {
		float a = i * (float)M_PI * 0.25f;
		float ca = cosf(a), sa = sinf(a);
		vec3_t dir = add(mulf(fwd, ca), mulf(side, sa));
		float ext = 1.0f / sqrtf((ca / hz) * (ca / hz) + (sa / hx) * (sa / hx));
		ag_hit_t hit;
		if (!world_raycast(w, p->pos, dir, ext, &hit)) continue;
		if (!((hit.surface_flags & PR_SURFACE_WALL) || dot(hit.normal, plane_n) < 0.5f)) continue;
		vec3_t n = normalize_or(on_plane(hit.normal, plane_n), mulf(dir, -1));
		float pen = (ext - hit.distance) * maxf(0.25f, -dot(dir, n)) - dot(pushed, n);
		if (pen <= 0) continue;
		p->pos = add(p->pos, mulf(n, pen));
		pushed = add(pushed, mulf(n, pen));
		if (pen > best_pen) {
			best_pen = pen;
			wall_n = n;
			wall_hit = hit;
		}
	}
	if (best_pen <= 0) {
		return;
	}

	// Velocity response on the traction state
	float speed = p->speed_value;
	vec3_t v = p->velocity_dir;
	float vn = dot(v, wall_n);
	float vl = len(v);
	if (vn >= 0 || vl < 1e-3f) {
		return;
	}
	float sin_a = clampf(-vn / vl, 0, 1);
	float cos_a = sqrtf(1.0f - sin_a * sin_a);
	p->velocity_dir = sub(v, mulf(wall_n, vn * 1.2f));
	p->velocity_slope = on_plane(p->velocity_slope, wall_n);

	// The wall takes the speed into it, plus a little scrape. Map the kept
	// fraction back onto the thrust and boost integrators through the
	// inverse of their speed curves, so the speedometer drops by as much.
	float keep = cos_a * (1.0f - 0.1f * sin_a);
	float base = p->accel_thrust > 0 ? p->accel_thrust * p->stats.max_speed / (p->stats.acceleration + p->accel_thrust) : 0;
	if (base > 0 && base * keep < p->stats.max_speed) {
		p->accel_thrust = p->stats.acceleration * base * keep / (p->stats.max_speed - base * keep);
	}
	float boost = p->boost_value > 0 ? p->boost_value * p->stats.boost_thrust / (p->boost_value + 0.33f) : 0;
	if (boost > 0 && boost * keep < p->stats.boost_thrust) {
		p->boost_value = 0.33f * boost * keep / (p->stats.boost_thrust - boost * keep);
	}
	float impact = speed * sin_a;

	// Nose turns away from the wall
	float away = dot(wall_n, pod_right(w, p)) > 0 ? -1.0f : 1.0f; // Wall on the left pushes right (-)
	p->contact_kick += away * minf(impact, 300.0f) * 0.5f;

	if (impact > 100.0f) {
		p->wall_hit_cooldown = 0.1f;
	}
	if (impact > 30.0f) {
		bool left = dot(wall_n, pod_right(w, p)) > 0;
		int first = left ? 0 : 3;
		float amount = (impact - 30.0f) / 600.0f;
		for (int k = 0; k < 3; k++) {
			pod_take_damage(w, id, p, first + k, amount * (k == 1 ? 1.0f : 0.6f));
		}
		event_push(w, PR_EVENT_WALL_HIT, id, PR_INVALID_POD, impact, sin_a, wall_hit.position);
	}
	pod_crash(w, id, p, speed, speed * (1.0f - keep));
}


// Orientation ------------------------------------------------------------------

static void pod_align_up(pod_t *p, vec3_t target, float t) {
	vec3_t axis = cross(p->up, target);
	float s = len(axis);
	float angle = atan2f(s, dot(p->up, target));
	if (s < 1e-6f || angle < 1e-6f) return;
	axis = mulf(axis, 1.0f / s);
	p->fwd = rotate(p->fwd, axis, angle * t);
	p->up = rotate(p->up, axis, angle * t);
}

static void pod_orthonormalize(pod_t *p, vec3_t fallback_up) {
	p->up = normalize_or(p->up, fallback_up);
	vec3_t f = on_plane(p->fwd, p->up);
	if (len(f) < 1e-4f) f = on_plane(fabsf(p->up.x) < 0.9f ? vec3(1, 0, 0) : vec3(0, 0, 1), p->up);
	p->fwd = normalize_or(f, vec3(0, 0, 1));
}

// PORTED in spirit from swrRace_AlignToSurface (0x004764e0) and
// swrRace_CalculateTiltFromTurn (0x0046e750): align to the surface near the
// ground, nose down in flight, bank into turns and lean with manual tilt
static void pod_orient(pr_world_t *w, pod_t *p, float dt) {
	float hover_hi = p->stats.hover_height * 1.5f;
	float hover_lo = (2.0f * p->stats.intersect_radius + p->stats.hover_height) / 3.0f;
	float blend = clampf((hover_hi - p->ground_dist) / (hover_hi - hover_lo), 0, 1);
	vec3_t world_up = mulf(w->down, -1);
	// Magnet surfaces hold the pod's frame to the surface even when it lifts
	// off over a crest
	if (p->s.magnet && p->ground_dist < 100.0f) {
		blend = 1;
	}

	if (p->s.zero_g) {
		// Free pitch from the zero-g pitch rate (x1.5, like the yaw)
		vec3_t side = cross(p->up, p->fwd);
		float a = deg2rad(p->zero_g_pitch_rate * 1.5f) * dt * w->hand;
		p->fwd = rotate(p->fwd, side, a);
		p->up = rotate(p->up, side, a);
	}
	else if (blend > 0) {
		// The game corrects a third to half of the error per ~30 fps frame
		pod_align_up(p, p->surface_up, 1.0f - powf(1.0f - 0.4f * blend, dt * 30.0f));
	}
	else {
		// Airborne: level the roll, drop the nose towards -37 degrees at 22/s;
		// pulling the nose up holds it
		vec3_t level = normalize_or(on_plane(world_up, p->fwd), p->up);
		pod_align_up(p, level, 1.0f - powf(0.875f, dt * 30.0f));
		float pitch_deg = rad2deg(asinf(clampf(dot(p->fwd, world_up), -1, 1)));
		float rate = pitch_deg > -37.0f ? -22.0f : 0;
		if (p->pitch < 0 && pitch_deg < -10.0f) rate -= p->pitch * 20.0f;
		vec3_t side = cross(p->up, p->fwd);
		float a = -deg2rad(rate) * dt;
		p->fwd = rotate(p->fwd, side, a);
		p->up = rotate(p->up, side, a);
	}
	pod_orthonormalize(p, world_up);

	// Visual bank: lean into the turn (70 deg cap for humans, 300 for AI),
	// eased at 5/s; manual tilt pulls towards +-80 deg
	float max_angle = p->is_ai ? 300.0f : 70.0f;
	float bank = clampf(-(p->s.turn_rate / maxf(p->stats.max_turn_rate, 1e-3f)) * max_angle, -80, 80);
	p->tilt_target += (bank - p->tilt_target) * minf(dt * 5.0f, 1);
	float tilt = p->s.magnet ? 0 : p->tilt_target;
	if (p->tilt_manual != 0 && !p->s.magnet) {
		tilt += (p->tilt_manual * 80.0f - tilt) * fabsf(p->tilt_manual);
	}
	p->s.tilt = tilt;
}


// Step -------------------------------------------------------------------------

static void pod_place(pr_world_t *w, pod_t *p, float progress, float lateral);

static void pod_step(pr_world_t *w, int id, pod_t *p, float dt) {
	if (p->s.dead) {
		p->dead_timer -= dt;
		if (p->dead_timer <= 0) {
			pr_pod_respawn(w, id);
		}
		return;
	}
	p->wall_hit_cooldown -= dt;
	p->invincible -= dt;
	if (p->respawn_pending > 0) {
		pr_pod_respawn(w, id);
		return;
	}

	// 1. Control (swrRace_CalcTargetTurnRate)
	pr_input_t in = p->input;
	float ai_turn = 0;
	if (p->is_ai) {
		pod_ai_control(w, id, p, &in, &ai_turn);
	}
	pod_control(w, id, p, &in, dt);
	if (p->is_ai) {
		p->turn_rate_target = ai_turn;
		if (p->surface_flags & PR_SURFACE_SIDE) pod_tilt(p, 1, dt);
	}
	p->prev_input = in;
	p->speed_multiplier = p->pace_multiplier;
	if (p->is_ai) {
		p->throttle *= p->speed_multiplier;
	}
	if (p->wall_hit_cooldown > 0) {
		p->throttle = 0;
	}
	float cap = p->stats.max_turn_rate * p->pace_multiplier;
	p->turn_rate_target = clampf(p->turn_rate_target * p->pace_multiplier, -cap, cap);

	// 2. Turn (swrRace_UpdateTurn): the rate chases the target at turn
	// response, 5x when on the wrong side of zero
	float response = p->stats.turn_response;
	float rate = p->s.turn_rate;
	if ((rate < 0 && p->turn_rate_target > rate) || (rate > 0 && p->turn_rate_target < rate)) {
		response *= 5.0f;
	}
	p->s.turn_rate = ease(rate, p->turn_rate_target, response, dt);

	// RECONSTRUCTED: leaning into the turn tightens it (TAS notes)
	float lean = -p->tilt_manual * (p->s.turn_rate >= 0 ? 1.0f : -1.0f);
	float turn = p->s.turn_rate * (1.0f + 0.25f * maxf(lean, 0));

	p->contact_kick *= decelerator(4.0f, dt);
	vec3_t yaw_axis = p->s.airborne ? p->up : p->surface_up;
	float yaw = deg2rad(turn * 1.5f + p->contact_kick) * dt * -w->hand;
	p->fwd = rotate(p->fwd, yaw_axis, yaw);
	p->up = rotate(p->up, yaw_axis, yaw);
	pod_orthonormalize(p, mulf(w->down, -1));

	// 3. Speed and motion (swrRace_IntegrateMotion)
	vec3_t prev_pos = p->pos;
	float speed = pod_update_speed(p, dt) + pod_apply_boost(p, dt);
	p->speed_value = speed;
	vec3_t plane_n = (p->s.airborne || p->s.zero_g) ? mulf(w->down, -1) : p->surface_up;
	vec3_t dir = p->s.zero_g ? p->fwd : normalize_or(on_plane(p->fwd, plane_n), p->fwd);
	vec3_t vel = pod_apply_traction(p, speed, dir, dt);

	// Steep climbs get flattened unless on a magnet surface or in zero-g
	if (!p->s.magnet && !p->s.zero_g) {
		float up_comp = -dot(vel, w->down);
		vec3_t horiz = on_plane(vel, w->down);
		float h2 = dot(horiz, horiz);
		if (up_comp > 0 && h2 * 0.1369f < up_comp * up_comp) {
			vel = add(horiz, mulf(w->down, -sqrtf(h2) * 0.2f));
		}
	}

	vel = add(vel, p->velocity_bump);
	p->velocity_bump = mulf(p->velocity_bump, decelerator(4.0f, dt));

	// Downhill slide
	if (p->throttle > 0.1f || p->throttle < -0.1f || p->invincible <= 0) {
		float sd = dot(vel, p->velocity_slope);
		float sl = len(p->velocity_slope);
		if (sd < 0 || sl <= 1.0f) {
			vel = add(vel, p->velocity_slope);
		}
		else {
			if (speed > 1.0f && sd / (speed * 60.0f) > 0) {
				p->accel_thrust += 2.0f * dt * sd / (speed * 60.0f);
			}
			vel = add(vel, mulf(p->velocity_slope, maxf(sd / sl * 0.01f, 1.0f)));
		}
	}
	p->pos = add(p->pos, mulf(vel, dt));

	// 4. Engine heat
	pod_update_heat(w, id, p, dt);

	// 5. Ground (swrRace_UpdateGroundContact / RaycastGround)
	vec3_t down = p->s.magnet ? mulf(p->surface_up, -1) : w->down;
	// Cast from above the pod by more than a step's travel, so a pod that
	// sank into a rising slope during the move still finds the ground
	float lift = 2.0f + len(sub(p->pos, prev_pos)) + p->stats.hover_height;
	ag_hit_t hit;
	bool found = world_ground_cast(w, sub(p->pos, mulf(down, lift)), down, 3000.0f, &hit);
	if (!found && p->s.magnet) {
		down = w->down;
		found = world_ground_cast(w, sub(p->pos, mulf(down, lift)), down, 3000.0f, &hit);
	}
	if (found) {
		p->ground_dist = hit.distance - lift;
		vec3_t n = hit.normal;
		if (p->s.magnet && dot(n, mulf(w->down, -1)) < 0.05f) {
			n = normalize_or(add(n, mulf(w->down, -0.05f)), n);
		}
		p->surface_up = n;
		pod_update_surface(w, id, p, hit.surface_flags, dt);
	}
	else {
		p->ground_dist = 100000.0f;
		p->surface_up = mulf(w->down, -1);
		pod_update_surface(w, id, p, 0, dt);
	}
	if (p->invincible <= 0 || p->throttle > 0.1f) {
		if (p->s.magnet) {
			p->velocity_slope = vec3(0, 0, 0);
		}
		else {
			pod_slope(w, p, p->surface_up, dt);
		}
	}
	if (!p->s.zero_g) {
		pod_gravity(w, id, p, down, dt);
	}
	if (p->ground_dist < 15.0f && !(p->surface_flags & PR_SURFACE_FALL)) {
		p->last_ground_pos = p->pos;
		p->last_ground_fwd = p->fwd;
	}

	// 6. Walls
	pod_walls(w, id, p, prev_pos, plane_n);

	// 7. Orientation
	pod_orient(w, p, dt);

	p->velocity = mulf(sub(p->pos, prev_pos), 1.0f / dt);
}

// Move a pod sideways without passing through walls
static void pod_shove(pr_world_t *w, pod_t *p, vec3_t delta) {
	float l = len(delta);
	if (l < 1e-5f) return;
	vec3_t dir = mulf(delta, 1.0f / l);
	float clearance = p->stats.intersect_radius * 3.0f;
	ag_hit_t hit;
	if (world_raycast(w, p->pos, dir, l + clearance, &hit) && ((hit.surface_flags & PR_SURFACE_WALL) || dot(hit.normal, p->up) < 0.5f)) {
		l = maxf(0, minf(l, hit.distance - clearance));
	}
	p->pos = add(p->pos, mulf(dir, l));
}

// RECONSTRUCTED pod vs. pod contact (swrRace_UpdatePhysicsContact isn't
// decompiled): push apart, trade velocity by bump mass, damage the engines on
// the struck side
static void pods_collide(pr_world_t *w) {
	for (int a = 0; a < PR_MAX_PODS; a++) {
		pod_t *pa = &w->pods[a];
		if (!pa->active || pa->s.dead) continue;
		for (int b = a + 1; b < PR_MAX_PODS; b++) {
			pod_t *pb = &w->pods[b];
			if (!pb->active || pb->s.dead) continue;
			float ra = pa->stats.intersect_radius * 3.0f, rb = pb->stats.intersect_radius * 3.0f;
			// Pods shove each other sideways and along, never into the ground
			vec3_t d = on_plane(sub(pb->pos, pa->pos), pa->up);
			float dist = len(d);
			if (dist >= ra + rb || dist < 1e-4f) continue;
			vec3_t n = mulf(d, 1.0f / dist);
			float ma = maxf(pa->stats.bump_mass, 1), mb = maxf(pb->stats.bump_mass, 1);
			float pen = ra + rb - dist;
			pod_shove(w, pa, mulf(n, -pen * mb / (ma + mb)));
			pod_shove(w, pb, mulf(n, pen * ma / (ma + mb)));
			float v_rel = dot(sub(pb->velocity, pa->velocity), n);
			if (v_rel >= 0) continue;
			// Heavier pods shove lighter ones (bump mass)
			float j = -v_rel * 1.3f;
			pa->velocity_bump = sub(pa->velocity_bump, mulf(n, j * mb / (ma + mb)));
			pb->velocity_bump = add(pb->velocity_bump, mulf(n, j * ma / (ma + mb)));
			float impact = -v_rel;
			if (impact > 40.0f) {
				float amount = (impact - 40.0f) / 1500.0f;
				bool a_left = dot(n, pod_right(w, pa)) < 0;
				bool b_left = dot(mulf(n, -1), pod_right(w, pb)) < 0;
				for (int k = 0; k < 3; k++) {
					pod_take_damage(w, a, pa, (a_left ? 0 : 3) + k, amount);
					pod_take_damage(w, b, pb, (b_left ? 0 : 3) + k, amount);
				}
				event_push(w, PR_EVENT_POD_HIT, a, b, impact, 0, add(pa->pos, mulf(n, ra)));
			}
		}
	}
}

static void pod_update_race(pr_world_t *w, int id, pod_t *p, float dt) {
	if (!p->locked) {
		p->s.lap_time += dt;
	}
	if (!w->route) {
		return;
	}
	float progress;
	p->route_seg = route_nearest(w, p->pos, p->route_seg, &progress, NULL);
	float prev = p->s.route_progress / w->scale;
	if (w->desc.route_closed) {
		float delta = progress - prev;
		if (delta < -w->route_length * 0.5f) {
			p->s.lap++;
			if (p->s.lap > p->max_lap) {
				p->max_lap = p->s.lap;
				if (p->s.lap >= 2) {
					p->s.last_lap_time = p->s.lap_time;
					if (p->s.best_lap_time <= 0 || p->s.lap_time < p->s.best_lap_time) {
						p->s.best_lap_time = p->s.lap_time;
					}
					event_push(w, PR_EVENT_LAP, id, PR_INVALID_POD, 0, p->s.lap_time, p->pos);
				}
				p->s.lap_time = 0;
			}
		}
		else if (delta > w->route_length * 0.5f) {
			p->s.lap--;
		}
		p->laps = (p->s.lap - 1) + progress / w->route_length;
	}
	else {
		p->laps = progress / w->route_length;
	}
	p->s.route_progress = progress * w->scale;
	p->s.total_progress = p->laps * w->route_length * w->scale;
	if (p->ground_dist < 15.0f && !(p->surface_flags & PR_SURFACE_FALL) && !p->s.dead) {
		p->last_valid_progress = progress;
	}
}

static void world_substep(pr_world_t *w, float dt) {
	// Pacing inputs: gaps in laps to the pace-setter and the leading human
	float pacer = -INFINITY, human = -INFINITY;
	for (int i = 0; i < PR_MAX_PODS; i++) {
		pod_t *p = &w->pods[i];
		if (!p->active) continue;
		p->prev_pose_pos = p->pos;
		p->prev_pose_fwd = p->fwd;
		p->prev_pose_up = p->up;
		p->prev_tilt = p->s.tilt;
		if (p->is_ai && p->ai.pace_setter) pacer = maxf(pacer, p->laps);
		if (!p->is_ai) human = maxf(human, p->laps);
	}
	int humans = 0;
	for (int i = 0; i < PR_MAX_PODS; i++) {
		if (w->pods[i].active && !w->pods[i].is_ai) humans++;
	}

	// The two AI nearest the leading human pace on their gap to them
	int tether[2] = {-1, -1};
	float tether_gap[2] = {INFINITY, INFINITY};
	for (int i = 0; i < PR_MAX_PODS; i++) {
		pod_t *p = &w->pods[i];
		p->ai_tethered = false;
		if (!p->active || !p->is_ai || p->ai.pace_setter || human == -INFINITY) continue;
		float g = fabsf(human - p->laps);
		if (g < tether_gap[0]) {
			tether[1] = tether[0]; tether_gap[1] = tether_gap[0];
			tether[0] = i; tether_gap[0] = g;
		}
		else if (g < tether_gap[1]) {
			tether[1] = i; tether_gap[1] = g;
		}
	}
	for (int k = 0; k < 2; k++) {
		if (tether[k] >= 0) w->pods[tether[k]].ai_tethered = true;
	}

	for (int i = 0; i < PR_MAX_PODS; i++) {
		pod_t *p = &w->pods[i];
		if (!p->active) continue;
		if (p->is_ai) {
			float gap_pacer = pacer > -INFINITY ? pacer - p->laps : 0;
			float gap_human = human > -INFINITY ? human - p->laps : -100.0f;
			pod_ai_pace(w, p, dt, gap_pacer, gap_human);
			// Look ahead ~90 units (the game eases its spline look-ahead to 90^2)
			p->ai_look_ahead = 90.0f;
		}
		else {
			p->pace_multiplier = 1.0f;
			if (w->desc.human_catchup && humans > 1 && human > -INFINITY && human - p->laps > 0) {
				float inv_track = 500000.0f / maxf(w->route_length, 1);
				p->pace_multiplier = minf((human - p->laps) * 100.0f / inv_track + 1.0f, 1.25f);
			}
		}
		pod_step(w, i, p, dt);
	}

	pods_collide(w);

	for (int i = 0; i < PR_MAX_PODS; i++) {
		pod_t *p = &w->pods[i];
		if (!p->active) continue;
		pod_update_race(w, i, p, dt);

		// Public state
		vec3_t right = pod_right(w, p);
		p->s.position = mulf(p->pos, w->scale);
		p->s.velocity = mulf(p->velocity, w->scale);
		p->s.forward = p->fwd;
		p->s.up = p->up;
		p->s.right = right;
		p->s.speed = p->speed_value;
		p->s.ground_distance = p->ground_dist;
		p->s.pace_multiplier = p->pace_multiplier;
	}

	// Ranks
	for (int a = 0; a < PR_MAX_PODS; a++) {
		pod_t *pa = &w->pods[a];
		if (!pa->active) continue;
		int rank = 1;
		for (int b = 0; b < PR_MAX_PODS; b++) {
			pod_t *pb = &w->pods[b];
			if (b != a && pb->active && (pb->laps > pa->laps || (pb->laps == pa->laps && b < a))) rank++;
		}
		pa->s.rank = rank;
	}
	w->time += dt;
}

int pr_world_step(pr_world_t *w, float frame_dt) {
	float fixed = w->desc.fixed_dt;
	w->accumulator += clampf(frame_dt, 0, fixed * w->desc.max_substeps);
	int steps = 0;
	while (w->accumulator >= fixed && steps < w->desc.max_substeps) {
		world_substep(w, fixed);
		w->accumulator -= fixed;
		steps++;
	}
	if (w->accumulator >= fixed) {
		w->accumulator = fixed * 0.999f;
	}
	return steps;
}


// Pod API ----------------------------------------------------------------------

static pod_t *get_pod(pr_world_t *w, pr_pod_id_t id) {
	return (id >= 0 && id < PR_MAX_PODS && w->pods[id].active) ? &w->pods[id] : NULL;
}

static void pod_reset_motion(pr_world_t *w, pod_t *p) {
	p->throttle = 0;
	p->accel_thrust = 0;
	p->boost_value = 0;
	p->speed_value = 0;
	p->s.turn_rate = 0;
	p->turn_rate_target = 0;
	p->contact_kick = 0;
	p->velocity_dir = vec3(0, 0, 0);
	p->velocity_slope = vec3(0, 0, 0);
	p->velocity_bump = vec3(0, 0, 0);
	p->velocity = vec3(0, 0, 0);
	p->fall_velocity = 0;
	p->fall_step = 0;
	p->tilt_manual = 0;
	p->tilt_target = 0;
	p->s.tilt = 0;
	p->s.boosting = false;
	p->s.boost_state = PR_BOOST_NOT_READY;
	p->boost_charge = 0;
	p->s.engine_temp = 100.0f;
	p->wall_hit_cooldown = 0;
	p->respawn_pending = 0;
	p->surface_up = mulf(w->down, -1);
	p->prev_pos = p->pos;
	p->prev_pose_pos = p->pos;
	p->prev_pose_fwd = p->fwd;
	p->prev_pose_up = p->up;
	p->prev_tilt = 0;
}

// Put the pod at hover height above the surface below `pos`
static void pod_snap(pr_world_t *w, pod_t *p) {
	ag_hit_t hit;
	if (world_raycast(w, sub(p->pos, mulf(w->down, 50.0f)), w->down, 500.0f, &hit)) {
		p->pos = add(hit.position, mulf(hit.normal, p->stats.hover_height));
		p->up = hit.normal;
		p->surface_up = hit.normal;
		p->ground_dist = p->stats.hover_height;
	}
	pod_orthonormalize(p, mulf(w->down, -1));
}

pr_pod_id_t pr_pod_add(pr_world_t *w, const pr_pod_stats_t *stats, vec3_t position, vec3_t forward) {
	for (int i = 0; i < PR_MAX_PODS; i++) {
		pod_t *p = &w->pods[i];
		if (p->active) continue;
		memset(p, 0, sizeof(*p));
		p->active = true;
		p->stats = *stats;
		p->pos = mulf(position, 1.0f / w->scale);
		p->fwd = forward;
		p->up = mulf(w->down, -1);
		p->slide2 = 1.0f;
		p->surface_speed_factor = 1.0f;
		p->surface_grip_factor = 1.0f;
		p->pace_multiplier = 1.0f;
		p->speed_multiplier = 1.0f;
		p->route_seg = -1;
		p->ai_look_ahead = 90.0f;
		pod_orthonormalize(p, p->up);
		pod_snap(w, p);
		pod_reset_motion(w, p);
		p->last_ground_pos = p->pos;
		p->last_ground_fwd = p->fwd;
		if (w->route) {
			float progress;
			p->route_seg = route_nearest(w, p->pos, -1, &progress, NULL);
			p->s.route_progress = progress * w->scale;
			p->last_valid_progress = progress;
		}
		return i;
	}
	return PR_INVALID_POD;
}

void pr_pod_remove(pr_world_t *w, pr_pod_id_t id) {
	pod_t *p = get_pod(w, id);
	if (p) p->active = false;
}

void pr_pod_set_stats(pr_world_t *w, pr_pod_id_t id, const pr_pod_stats_t *stats) {
	pod_t *p = get_pod(w, id);
	if (p) p->stats = *stats;
}

void pr_pod_set_input(pr_world_t *w, pr_pod_id_t id, const pr_input_t *input) {
	pod_t *p = get_pod(w, id);
	if (p) p->input = *input;
}

void pr_pod_set_ai(pr_world_t *w, pr_pod_id_t id, const pr_ai_params_t *ai) {
	pod_t *p = get_pod(w, id);
	if (!p) return;
	p->is_ai = ai != NULL;
	if (ai) {
		p->ai = *ai;
		if (p->ai.level <= 0) p->ai.level = 1;
		p->ai_rank_target = p->ai.rank_baseline > 0 ? p->ai.rank_baseline : 2;
		p->ai_decision_timer = 8.0f + world_rand(w) * 10.0f;
	}
}

const pr_pod_state_t *pr_pod_state(pr_world_t *w, pr_pod_id_t id) {
	pod_t *p = get_pod(w, id);
	return p ? &p->s : NULL;
}

void pr_pod_get_pose(pr_world_t *w, pr_pod_id_t id, ag_pose_t *pose) {
	pod_t *p = get_pod(w, id);
	if (!p) return;
	float t = pr_world_alpha(w);
	vec3_t fwd = normalize_or(lerp3(p->prev_pose_fwd, p->fwd, t), p->fwd);
	vec3_t up = normalize_or(on_plane(normalize_or(lerp3(p->prev_pose_up, p->up, t), p->up), fwd), p->up);
	float tilt = lerpf(p->prev_tilt, p->s.tilt, t);
	// Positive tilt leans right
	up = rotate(up, fwd, deg2rad(tilt) * -w->hand);
	pose->position = mulf(lerp3(p->prev_pose_pos, p->pos, t), w->scale);
	pose->forward = fwd;
	pose->up = up;
	pose->right = mulf(cross(up, fwd), w->hand);
}

// Is there drivable ground (not a pit) under the route at this progress?
static bool route_ground_ok(pr_world_t *w, float progress) {
	vec3_t pos, up;
	route_sample(w, progress, &pos, NULL, &up, NULL);
	ag_hit_t hit;
	if (!world_raycast(w, add(pos, mulf(up, 50.0f)), mulf(up, -1), 150.0f, &hit)) {
		return false;
	}
	return !(hit.surface_flags & (PR_SURFACE_FALL | PR_SURFACE_WALL)) && hit.distance < 80.0f;
}

static void pod_place(pr_world_t *w, pod_t *p, float progress, float lateral) {
	vec3_t pos, dir, up;
	route_sample(w, progress, &pos, &dir, &up, NULL);
	vec3_t side = cross(up, dir);
	p->pos = add(add(pos, mulf(side, lateral)), mulf(up, p->stats.hover_height));
	p->fwd = dir;
	p->up = up;
	pod_orthonormalize(p, mulf(w->down, -1));
	pod_snap(w, p);
	pod_reset_motion(w, p);
	p->route_seg = route_seg_at(w, route_wrap(w, progress));
	p->s.route_progress = route_wrap(w, progress) * w->scale;
}

void pr_pod_place_on_grid(pr_world_t *w, pr_pod_id_t id, int slot) {
	pod_t *p = get_pod(w, id);
	if (!p || !w->route) return;
	// Racer grids are three wide
	int row = slot / 3, column = slot % 3;
	float spacing = p->stats.intersect_radius * 14.0f;
	float behind = (row + 1) * spacing;
	float width;
	route_sample(w, 0, NULL, NULL, NULL, &width);
	float lateral = (column - 1) * width * 0.25f * w->hand;
	if (w->desc.route_closed) {
		p->s.lap = 0;
		p->max_lap = 0;
		pod_place(w, p, w->route_length - behind, lateral);
		p->laps = p->s.route_progress / w->scale / w->route_length - 1.0f;
	}
	else {
		p->s.lap = 1;
		p->max_lap = 1;
		pod_place(w, p, (PR_MAX_PODS / 3 + 1) * spacing - behind, lateral);
		p->laps = p->s.route_progress / w->scale / w->route_length;
	}
	p->s.total_progress = p->laps * w->route_length * w->scale;
	p->last_valid_progress = p->s.route_progress / w->scale;
	p->s.lap_time = 0;
}

void pr_pod_set_locked(pr_world_t *w, pr_pod_id_t id, bool locked) {
	pod_t *p = get_pod(w, id);
	if (!p || p->locked == locked) return;
	p->locked = locked;
	if (!locked) {
		p->start_window = 0.2f; // Fresh throttle press within 0.2s of GO
		p->s.lap_time = 0;
	}
}

void pr_pod_respawn(pr_world_t *w, pr_pod_id_t id) {
	pod_t *p = get_pod(w, id);
	if (!p) return;
	bool was_dead = p->s.dead;
	if (w->route) {
		int lap = p->s.lap;
		float laps = p->laps;
		// A pod restarting from rest can't clear a gap: if one is coming up
		// shortly, restart past it
		float progress = p->last_valid_progress;
		for (float d = 0; d <= 200.0f; d += 10.0f) {
			if (!route_ground_ok(w, progress + d)) {
				progress += d;
				for (int i = 0; i < 200 && !route_ground_ok(w, progress); i++) {
					progress += 10.0f;
				}
				progress += 40.0f; // Clear of the far edge
				break;
			}
		}
		pod_place(w, p, progress, 0);
		p->last_valid_progress = route_wrap(w, progress);
		p->s.lap = lap;
		p->laps = laps;
	}
	else {
		p->pos = p->last_ground_pos;
		p->fwd = p->last_ground_fwd;
		p->up = mulf(w->down, -1);
		pod_orthonormalize(p, p->up);
		pod_snap(w, p);
		pod_reset_motion(w, p);
	}
	p->s.dead = false;
	p->invincible = 2.0f;
	p->fall_timer = 0;
	if (was_dead) {
		for (int i = 0; i < 6; i++) {
			p->s.engine_damage[i] = minf(p->s.engine_damage[i], 0.5f);
			p->s.engine_on_fire[i] = false;
		}
	}
	event_push(w, PR_EVENT_RESPAWNED, id, PR_INVALID_POD, 0, 0, p->pos);
}

void pr_pod_damage(pr_world_t *w, pr_pod_id_t id, int engine, float amount) {
	pod_t *p = get_pod(w, id);
	if (p) pod_take_damage(w, id, p, engine, amount);
}

void pr_pod_apply_impulse(pr_world_t *w, pr_pod_id_t id, vec3_t delta_velocity) {
	pod_t *p = get_pod(w, id);
	if (p) p->velocity_bump = add(p->velocity_bump, mulf(delta_velocity, 1.0f / w->scale));
}
