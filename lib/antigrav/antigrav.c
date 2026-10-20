#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "antigrav.h"

#if !defined(M_PI)
	#define M_PI 3.14159265358979323846
#endif

// Sign conventions: internally, a positive yaw rotates the forward vector
// towards side = up x forward. The public API speaks of "right", which is
// side for left handed and -side for right handed systems. w->hand converts
// between the two at the API boundary.


// Math -------------------------------------------------------------------------

typedef ag_vec3_t vec3_t;

static inline vec3_t vec3(float x, float y, float z) { return (vec3_t){x, y, z}; }
static inline vec3_t vec3_add(vec3_t a, vec3_t b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline vec3_t vec3_sub(vec3_t a, vec3_t b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline vec3_t vec3_mulf(vec3_t a, float f) { return vec3(a.x * f, a.y * f, a.z * f); }
static inline float vec3_dot(vec3_t a, vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float vec3_len(vec3_t a) { return sqrtf(vec3_dot(a, a)); }
static inline vec3_t vec3_lerp(vec3_t a, vec3_t b, float t) { return vec3_add(a, vec3_mulf(vec3_sub(b, a), t)); }

static inline vec3_t vec3_cross(vec3_t a, vec3_t b) {
	return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static inline vec3_t vec3_normalize_or(vec3_t a, vec3_t fallback) {
	float len = vec3_len(a);
	return len > 1e-6f ? vec3_mulf(a, 1.0f / len) : fallback;
}

// Remove the component along the unit vector n
static inline vec3_t vec3_on_plane(vec3_t a, vec3_t n) {
	return vec3_sub(a, vec3_mulf(n, vec3_dot(a, n)));
}

// Rotate a around the unit axis by angle (Rodrigues)
static vec3_t vec3_rotate(vec3_t a, vec3_t axis, float angle) {
	float c = cosf(angle), s = sinf(angle);
	return vec3_add(
		vec3_add(vec3_mulf(a, c), vec3_mulf(vec3_cross(axis, a), s)),
		vec3_mulf(axis, vec3_dot(axis, a) * (1 - c))
	);
}

// Signed angle from a to b around the unit axis; positive towards axis x a
static float vec3_signed_angle(vec3_t a, vec3_t b, vec3_t axis) {
	return atan2f(vec3_dot(vec3_cross(a, b), axis), vec3_dot(a, b));
}

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline float signf(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float minf(float a, float b) { return a < b ? a : b; }

static inline float approachf(float v, float target, float step) {
	return v < target ? minf(v + step, target) : maxf(v - step, target);
}

// Fraction to move towards a target per step for an exponential approach
static inline float blend(float rate, float dt) {
	return 1.0f - expf(-rate * dt);
}


// Types ------------------------------------------------------------------------

typedef struct {
	bool grounded;
	int hits;
	vec3_t normal;
	float height;
	uint32_t flags;
	uint32_t material;
} ground_t;

typedef struct {
	bool active;
	ag_ship_params_t params;
	ag_ship_state_t s;
	ag_input_t input;

	// Internal frame and rates; s.forward/up/right/yaw_rate are derived
	float yaw_rate;
	float pitch_rate;
	float thrust_scale;
	float boost_accel;
	bool on_boost_pad;
	bool was_grounded;

	vec3_t prev_position, prev_forward, prev_up;
	float prev_bank;

	// Sideshift & barrel roll gestures
	bool left_was_down, right_was_down;
	float left_tap_time, right_tap_time;
	int prev_sideshift_input;
	bool prev_barrel_roll_input;
	int sideshift_dir;          // Internal side sign
	int sideshift_source;       // 0 = ag_input_t.sideshift, -1/1 = left/right airbrake
	float sideshift_elapsed;
	float sideshift_cooldown;
	int last_flick;
	int flicks;
	float first_flick_time;
	float brake_left, brake_right; // Deployed airbrakes, 0..1
	float roll_dir;             // Internal side sign; 0 when not rolling
	float roll_angle, prev_roll_angle;
	bool energy_depleted;

	// AI
	bool is_ai;
	ag_ai_params_t ai;
	float ai_lateral;
	float ai_thrust_scale;
	float ai_start_timer;
	bool ai_hyperthrust;
	float ai_hyperthrust_cooldown;

	// Race
	int route_seg;
	int max_lap;
	float last_valid_progress;
	float last_valid_lateral;
	float wrong_way_time;
	float out_of_bounds_time;
	float stuck_time;
	float wall_event_cooldown;
	float ship_event_cooldown;
} ship_t;

struct ag_world_t {
	ag_world_desc_t desc;
	float hand;
	float meter;
	vec3_t gravity_up;

	float accumulator;
	float time;

	ship_t ships[AG_MAX_SHIPS];

	ag_route_point_t *route;
	vec3_t *route_dir;
	float *route_dist;
	int route_len;
	int route_segs;
	float route_length;

	ag_event_t events[AG_MAX_EVENTS];
	int events_head;
	int events_len;

	// Boost pads found by craft driving over them, for the AI
	struct {
		float progress;
		float lateral;
	} pads[64];
	int pads_len;
};


// Allocation & events ----------------------------------------------------------

static void *world_alloc(ag_world_t *w, size_t size) {
	return w->desc.alloc ? w->desc.alloc(size, w->desc.alloc_user) : malloc(size);
}

static void world_free(ag_world_t *w, void *p) {
	if (!p) {
		return;
	}
	if (w->desc.free) {
		w->desc.free(p, w->desc.alloc_user);
	}
	else {
		free(p);
	}
}

static void event_push(ag_world_t *w, ag_event_type_t type, int ship, int other, float intensity, float value, vec3_t pos, vec3_t normal, uint32_t material) {
	if (w->events_len >= AG_MAX_EVENTS) {
		return; // Host isn't polling; drop
	}
	ag_event_t *ev = &w->events[(w->events_head + w->events_len) % AG_MAX_EVENTS];
	*ev = (ag_event_t){
		.type = type, .ship = ship, .other = other, .intensity = intensity, .value = value,
		.position = pos, .normal = normal, .material = material
	};
	w->events_len++;
}

bool ag_world_poll_event(ag_world_t *w, ag_event_t *event) {
	if (w->events_len == 0) {
		return false;
	}
	*event = w->events[w->events_head];
	w->events_head = (w->events_head + 1) % AG_MAX_EVENTS;
	w->events_len--;
	return true;
}


// Route ------------------------------------------------------------------------

static vec3_t route_point_up(ag_world_t *w, int i) {
	return vec3_normalize_or(w->route[i].up, w->gravity_up);
}

static int route_next(ag_world_t *w, int i) {
	return (i + 1) % w->route_len;
}

static float route_wrap(ag_world_t *w, float d) {
	if (w->desc.route_closed) {
		d = fmodf(d, w->route_length);
		return d < 0 ? d + w->route_length : d;
	}
	return clampf(d, 0, w->route_length);
}

static int route_seg_at(ag_world_t *w, float d) {
	int lo = 0, hi = w->route_segs - 1;
	while (lo < hi) {
		int mid = (lo + hi + 1) / 2;
		if (w->route_dist[mid] <= d) {
			lo = mid;
		}
		else {
			hi = mid - 1;
		}
	}
	return lo;
}

static void route_init(ag_world_t *w, const ag_route_point_t *points, int len) {
	if (!points || len < 2) {
		return;
	}
	w->route_len = len;
	w->route_segs = w->desc.route_closed ? len : len - 1;
	w->route = world_alloc(w, sizeof(ag_route_point_t) * len);
	w->route_dir = world_alloc(w, sizeof(vec3_t) * len);
	w->route_dist = world_alloc(w, sizeof(float) * (len + 1));
	memcpy(w->route, points, sizeof(ag_route_point_t) * len);

	w->route_dist[0] = 0;
	for (int i = 0; i < w->route_segs; i++) {
		float seg_len = vec3_len(vec3_sub(w->route[route_next(w, i)].position, w->route[i].position));
		w->route_dist[i + 1] = w->route_dist[i] + seg_len;
	}
	w->route_length = w->route_dist[w->route_segs];

	// Smoothed directions at each point
	for (int i = 0; i < len; i++) {
		int prev = i > 0 ? i - 1 : (w->desc.route_closed ? len - 1 : 0);
		int next = i < len - 1 ? i + 1 : (w->desc.route_closed ? 0 : len - 1);
		vec3_t d = vec3_sub(w->route[next].position, w->route[prev].position);
		w->route_dir[i] = vec3_normalize_or(vec3_on_plane(d, route_point_up(w, i)), vec3(0, 0, 1));
	}
}

float ag_route_length(ag_world_t *w) {
	return w->route_length;
}

typedef struct {
	vec3_t position;
	vec3_t up;
	float width;
} route_sample_t;

static ag_route_point_t route_control(const ag_route_point_t *c, int len, bool closed, int i) {
	if (closed) {
		return c[((i % len) + len) % len];
	}
	if (i < 0) {
		// Extrapolate past the ends so the spline reaches them
		ag_route_point_t p = c[0];
		p.position = vec3_sub(vec3_mulf(c[0].position, 2), c[1].position);
		return p;
	}
	if (i >= len) {
		ag_route_point_t p = c[len - 1];
		p.position = vec3_sub(vec3_mulf(c[len - 1].position, 2), c[len - 2].position);
		return p;
	}
	return c[i];
}

static vec3_t lerp_up(vec3_t a, vec3_t b, float t) {
	bool a_zero = vec3_dot(a, a) < 1e-12f, b_zero = vec3_dot(b, b) < 1e-12f;
	if (a_zero && b_zero) {
		return vec3(0, 0, 0);
	}
	if (a_zero) {
		return b;
	}
	if (b_zero) {
		return a;
	}
	return vec3_normalize_or(vec3_lerp(vec3_normalize_or(a, b), vec3_normalize_or(b, a), t), a);
}

// Centripetal Catmull-Rom between control points seg and seg + 1
static route_sample_t route_spline(const ag_route_point_t *c, int len, bool closed, int seg, float u) {
	ag_route_point_t p[4];
	for (int k = 0; k < 4; k++) {
		p[k] = route_control(c, len, closed, seg - 1 + k);
	}

	float t[4] = {0};
	for (int k = 1; k < 4; k++) {
		float d = vec3_len(vec3_sub(p[k].position, p[k - 1].position));
		t[k] = t[k - 1] + maxf(sqrtf(d), 1e-3f);
	}
	float tt = lerpf(t[1], t[2], u);
	#define CR_MIX(P, Q, TA, TB) vec3_add(vec3_mulf(P, (TB - tt) / (TB - TA)), vec3_mulf(Q, (tt - TA) / (TB - TA)))
	vec3_t a1 = CR_MIX(p[0].position, p[1].position, t[0], t[1]);
	vec3_t a2 = CR_MIX(p[1].position, p[2].position, t[1], t[2]);
	vec3_t a3 = CR_MIX(p[2].position, p[3].position, t[2], t[3]);
	vec3_t b1 = CR_MIX(a1, a2, t[0], t[2]);
	vec3_t b2 = CR_MIX(a2, a3, t[1], t[3]);
	vec3_t pos = CR_MIX(b1, b2, t[1], t[2]);
	#undef CR_MIX

	return (route_sample_t){
		.position = pos,
		.up = lerp_up(p[1].up, p[2].up, u),
		.width = lerpf(p[1].width, p[2].width, u),
	};
}

int ag_route_resample(const ag_route_point_t *controls, int len, bool closed, float spacing, ag_route_point_t *out, int out_capacity) {
	if (!controls || len < 2 || spacing <= 0) {
		return 0;
	}
	const int sub = 32;
	int segs = closed ? len : len - 1;

	// Pass 1: length of the dense polyline
	float total = 0;
	route_sample_t prev = route_spline(controls, len, closed, 0, 0);
	for (int seg = 0; seg < segs; seg++) {
		for (int j = 1; j <= sub; j++) {
			route_sample_t cur = route_spline(controls, len, closed, seg, (float)j / sub);
			total += vec3_len(vec3_sub(cur.position, prev.position));
			prev = cur;
		}
	}
	if (total <= 0) {
		return 0;
	}

	// Even spacing; a closed route must close up with the same spacing
	int count = closed ? (int)(total / spacing + 0.5f) : (int)(total / spacing + 0.5f) + 1;
	if (count < 2) {
		count = 2;
	}
	float step = closed ? total / count : total / (count - 1);
	if (!out) {
		return count;
	}

	// Pass 2: emit points at multiples of step
	int written = 0;
	float dist = 0, next = 0;
	prev = route_spline(controls, len, closed, 0, 0);
	for (int seg = 0; seg < segs && written < count && written < out_capacity; seg++) {
		for (int j = 1; j <= sub && written < count && written < out_capacity; j++) {
			route_sample_t cur = route_spline(controls, len, closed, seg, (float)j / sub);
			float d = vec3_len(vec3_sub(cur.position, prev.position));
			while (d > 0 && next <= dist + d && written < count && written < out_capacity) {
				float f = (next - dist) / d;
				out[written++] = (ag_route_point_t){
					.position = vec3_lerp(prev.position, cur.position, f),
					.up = lerp_up(prev.up, cur.up, f),
					.width = lerpf(prev.width, cur.width, f),
				};
				next = written * step;
			}
			dist += d;
			prev = cur;
		}
	}

	// Float drift may leave the very last point of an open route unwritten
	if (!closed && written == count - 1 && written < out_capacity) {
		ag_route_point_t last = controls[len - 1];
		out[written++] = last;
	}
	return count;
}

void ag_route_sample(ag_world_t *w, float d, vec3_t *position, vec3_t *direction, vec3_t *up, float *width) {
	if (!w->route) {
		if (position) *position = vec3(0, 0, 0);
		if (direction) *direction = vec3(0, 0, 1);
		if (up) *up = w->gravity_up;
		if (width) *width = 0;
		return;
	}
	d = route_wrap(w, d);
	int i = route_seg_at(w, d);
	int j = route_next(w, i);
	float seg_len = w->route_dist[i + 1] - w->route_dist[i];
	float t = seg_len > 1e-6f ? clampf((d - w->route_dist[i]) / seg_len, 0, 1) : 0;

	if (position) *position = vec3_lerp(w->route[i].position, w->route[j].position, t);
	if (up) *up = vec3_normalize_or(vec3_lerp(route_point_up(w, i), route_point_up(w, j), t), w->gravity_up);
	if (direction) *direction = vec3_normalize_or(vec3_lerp(w->route_dir[i], w->route_dir[j], t), w->route_dir[i]);
	if (width) *width = lerpf(w->route[i].width, w->route[j].width, t);
}

// Nearest point on the route; searches around seg (or everywhere if seg < 0)
static int route_nearest(ag_world_t *w, vec3_t p, int seg, float *progress, float *distance) {
	int first = 0, count = w->route_segs;
	if (seg >= 0) {
		count = w->route_segs < 13 ? w->route_segs : 13;
		first = seg - count / 2;
	}

	int best_seg = 0;
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
		vec3_t ab = vec3_sub(w->route[route_next(w, i)].position, a);
		float ab_len_sq = vec3_dot(ab, ab);
		float t = ab_len_sq > 1e-9f ? clampf(vec3_dot(vec3_sub(p, a), ab) / ab_len_sq, 0, 1) : 0;
		float dist = vec3_len(vec3_sub(p, vec3_add(a, vec3_mulf(ab, t))));
		if (dist < best_dist) {
			best_dist = dist;
			best_seg = i;
			best_t = t;
		}
	}
	*progress = lerpf(w->route_dist[best_seg], w->route_dist[best_seg + 1], best_t);
	*distance = best_dist;
	return best_seg;
}


// Defaults ---------------------------------------------------------------------

void ag_world_desc_default(ag_world_desc_t *desc, float units_per_meter) {
	memset(desc, 0, sizeof(*desc));
	desc->units_per_meter = units_per_meter;
	desc->gravity = vec3(0, -25.0f * units_per_meter, 0);
	desc->fixed_dt = 1.0f / 120.0f;
	desc->max_substeps = 8;
	desc->right_handed = true;
	desc->route_closed = true;
	desc->auto_respawn = true;
	desc->respawn_delay = 1.5f;
	desc->respawn_speed = 0;
}

void ag_ship_params_default(ag_ship_params_t *p, float m) {
	*p = (ag_ship_params_t){
		.mass = 1.0f,
		.radius = 2.2f * m,
		.half_extents = vec3(1.6f * m, 0.6f * m, 3.6f * m),

		.top_speed = 145.0f * m,
		.thrust_accel = 60.0f * m,
		.drag_linear_share = 0.35f,
		.spool_up = 2.2f,
		.spool_down = 1.2f,
		.coast_drag = 1.6f,

		.turn_rate = 2.2f,
		.turn_rate_at_top_speed = 0.5f,
		.turn_accel = 11.0f,
		.turn_return = 9.0f,
		.steer_curve = 1.4f,
		.grip = 5.5f,
		.slip_scrub = 0.25f,

		.airbrake_yaw = 0.9f,
		.airbrake_drag = 0.35f,
		.airbrake_grip = 0.35f,
		.airbrake_turn_drag = 0.04f,
		.airbrake_gain = 6.0f,
		.airbrake_falloff = 10.0f,

		.sideshift_speed = 16.0f * m,
		.sideshift_time = 0.2f,
		.sideshift_max_time = 0.4f,
		.sideshift_cooldown = 0.5f,
		.double_tap_window = 0.25f,

		.barrel_roll_time = 0.55f,
		.barrel_roll_cost = 10.0f,
		.barrel_roll_boost_accel = 60.0f * m,
		.barrel_roll_boost_time = 0.8f,
		.gesture_window = 0.5f,

		.dive_slope_bonus = 1.0f,

		.maglock_magnet = 600.0f * m,
		.maglock_grip = 2.0f,

		.energy_max = 100.0f,
		.wall_damage = 0.25f / m,
		.wall_damage_min = 3.0f * m,
		.wall_scrape_damage = 4.0f,
		.landing_damage = 0.3f / m,
		.pit_recharge = 25.0f,
		.ship_damage = 0.12f / m,
		.ship_damage_min = 4.0f * m,
		.hyperthrust_accel = 30.0f * m,
		.hyperthrust_drain = 25.0f,
		.hyperthrust_min = 0.25f,

		.hover_height = 1.2f * m,
		.hover_stiffness = 140.0f,
		.hover_damping = 0.75f,
		.hover_probe_length = 6.0f * m,
		.magnet = 90.0f * m,
		.track_gravity = 25.0f * m,
		.slope_gravity = 0.6f,
		.align_rate = 10.0f,

		.air_pitch_rate = 1.6f,
		.air_weathervane = 1.0f,
		.air_level_rate = 2.0f,
		.air_grip = 0.25f,
		.air_drag = 0.6f,

		.wall_restitution = 0.25f,
		.wall_friction = 0.6f,
		.wall_graze = 0.05f,
		.wall_align = 0.6f,
		.wall_scrape_drag = 0.6f,

		.landing_speed_min = 15.0f * m,
		.landing_speed_loss = 0.8f,
		.ship_hit_spin = 0.05f / m,

		.boost_pad_accel = 70.0f * m,
		.boost_pad_time = 0.6f,
		.boost_pad_kick = 6.0f * m,
		.draft_length = 40.0f * m,
		.draft_width = 3.5f * m,
		.draft_drag_reduction = 0.15f,

		.start_boost_min = 0.62f,
		.start_boost_max = 0.70f,
		.start_boost_time = 1.0f,
		.start_stall_above = 0.9f,
		.start_stall_time = 1.0f,

		.bank_max = 0.5f,
		.bank_rate = 6.0f,
	};
}

void ag_ship_params_class(ag_ship_params_t *p, ag_speed_class_t speed_class, float m) {
	static const float top_speed_kmh[] = {420, 522, 657, 793};
	ag_ship_params_default(p, m);
	int c = speed_class < AG_CLASS_VENOM ? AG_CLASS_VENOM : (speed_class > AG_CLASS_PHANTOM ? AG_CLASS_PHANTOM : speed_class);

	// Speeds and accelerations scale with the class; steering with its square
	// root (faster classes lean on the airbrakes more); downforce with its
	// square, since crests and mag-strips need speed^2 / radius
	float k = top_speed_kmh[c] / 3.6f * m / p->top_speed;
	float k_sqrt = sqrtf(k);
	p->top_speed *= k;
	p->thrust_accel *= k;
	p->boost_pad_accel *= k;
	p->boost_pad_kick *= k;
	p->barrel_roll_boost_accel *= k;
	p->hyperthrust_accel *= k;
	p->sideshift_speed *= k_sqrt;
	p->turn_rate *= k_sqrt;
	p->turn_accel *= k_sqrt;
	p->airbrake_yaw *= k_sqrt;
	p->magnet *= k * k;
	p->maglock_magnet *= k * k;
}

void ag_ai_params_difficulty(ag_ai_params_t *ai, ag_ai_difficulty_t difficulty) {
	static const ag_ai_params_t levels[] = {
		[AG_AI_NOVICE] = {.skill = 0.35f, .speed_scale = 0.92f, .racing_line = 0.4f, .start_reaction = 0.5f, .tricks = 0.0f, .aggression = 0.0f},
		[AG_AI_PILOT]  = {.skill = 0.6f,  .speed_scale = 0.96f, .racing_line = 0.6f, .start_reaction = 0.3f, .tricks = 0.35f, .aggression = 0.2f},
		[AG_AI_EXPERT] = {.skill = 0.85f, .speed_scale = 1.0f,  .racing_line = 0.75f, .start_reaction = 0.15f, .tricks = 0.8f, .aggression = 0.5f},
		[AG_AI_ELITE]  = {.skill = 1.0f,  .speed_scale = 1.03f, .racing_line = 0.85f, .start_reaction = 0.08f, .tricks = 1.0f, .aggression = 0.8f},
	};
	int d = difficulty < AG_AI_NOVICE ? AG_AI_NOVICE : (difficulty > AG_AI_ELITE ? AG_AI_ELITE : difficulty);
	*ai = levels[d];
}

void ag_ai_params_default(ag_ai_params_t *ai) {
	ag_ai_params_difficulty(ai, AG_AI_EXPERT);
}


// World ------------------------------------------------------------------------

ag_world_t *ag_world_create(const ag_world_desc_t *desc) {
	ag_world_t *w = desc->alloc ? desc->alloc(sizeof(ag_world_t), desc->alloc_user) : malloc(sizeof(ag_world_t));
	if (!w) {
		return NULL;
	}
	memset(w, 0, sizeof(*w));
	w->desc = *desc;
	w->desc.route = NULL;
	if (w->desc.fixed_dt <= 0) {
		w->desc.fixed_dt = 1.0f / 120.0f;
	}
	if (w->desc.max_substeps <= 0) {
		w->desc.max_substeps = 8;
	}
	if (w->desc.units_per_meter <= 0) {
		w->desc.units_per_meter = 1;
	}
	w->hand = desc->right_handed ? -1.0f : 1.0f;
	w->meter = w->desc.units_per_meter;
	w->gravity_up = vec3_normalize_or(vec3_mulf(desc->gravity, -1), vec3(0, 1, 0));

	route_init(w, desc->route, desc->route_len);
	return w;
}

void ag_world_destroy(ag_world_t *w) {
	if (!w) {
		return;
	}
	world_free(w, w->route);
	world_free(w, w->route_dir);
	world_free(w, w->route_dist);
	world_free(w, w);
}

float ag_world_alpha(ag_world_t *w) {
	return clampf(w->accumulator / w->desc.fixed_dt, 0, 1);
}


// Craft frame ------------------------------------------------------------------

static vec3_t ship_side(ship_t *sh) {
	return vec3_cross(sh->s.up, sh->s.forward);
}

static void ship_orthonormalize(ship_t *sh, vec3_t fallback_up) {
	sh->s.up = vec3_normalize_or(sh->s.up, fallback_up);
	vec3_t fwd = vec3_on_plane(sh->s.forward, sh->s.up);
	if (vec3_len(fwd) < 1e-4f) {
		// Forward got parallel to up; pick anything perpendicular
		fwd = vec3_on_plane(fabsf(sh->s.up.x) < 0.9f ? vec3(1, 0, 0) : vec3(0, 0, 1), sh->s.up);
	}
	sh->s.forward = vec3_normalize_or(fwd, vec3(0, 0, 1));
}

// Rotate the frame so that up approaches target_up by fraction t (0..1)
static void ship_align_up(ship_t *sh, vec3_t target_up, float t) {
	vec3_t axis = vec3_cross(sh->s.up, target_up);
	float s = vec3_len(axis);
	float angle = atan2f(s, vec3_dot(sh->s.up, target_up));
	if (s < 1e-6f || angle < 1e-6f) {
		return;
	}
	axis = vec3_mulf(axis, 1.0f / s);
	sh->s.forward = vec3_rotate(sh->s.forward, axis, angle * t);
	sh->s.up = vec3_rotate(sh->s.up, axis, angle * t);
}

// Rotate the frame so that forward approaches target_fwd by fraction t (0..1)
static void ship_align_forward(ship_t *sh, vec3_t target_fwd, float t) {
	vec3_t axis = vec3_cross(sh->s.forward, target_fwd);
	float s = vec3_len(axis);
	float angle = atan2f(s, vec3_dot(sh->s.forward, target_fwd));
	if (s < 1e-6f || angle < 1e-6f) {
		return;
	}
	axis = vec3_mulf(axis, 1.0f / s);
	sh->s.forward = vec3_rotate(sh->s.forward, axis, angle * t);
	sh->s.up = vec3_rotate(sh->s.up, axis, angle * t);
}

static void ship_update_public(ag_world_t *w, ship_t *sh) {
	vec3_t side = ship_side(sh);
	sh->s.right = vec3_mulf(side, w->hand);
	sh->s.yaw_rate = sh->yaw_rate * w->hand;
	sh->s.speed = vec3_len(sh->s.velocity);
}


// Ground & walls ---------------------------------------------------------------

static bool world_raycast(ag_world_t *w, vec3_t origin, vec3_t dir, float max_dist, ag_hit_t *hit) {
	if (!w->desc.raycast) {
		return false;
	}
	return w->desc.raycast(w->desc.user, origin, dir, max_dist, hit);
}

static ground_t ship_probe_ground(ag_world_t *w, ship_t *sh) {
	ag_ship_params_t *p = &sh->params;
	ground_t g = {0};

	vec3_t up = sh->s.up;
	vec3_t side = ship_side(sh);
	vec3_t down = vec3_mulf(up, -1);
	float lift = p->hover_height * 0.75f;
	float ray_len = lift + p->hover_height + p->hover_probe_length;

	// Front left, front right, back left, back right
	const float px[4] = {-0.75f, 0.75f, -0.75f, 0.75f};
	const float pz[4] = {0.65f, 0.65f, -0.65f, -0.65f};

	vec3_t points[4];
	vec3_t normal_sum = vec3(0, 0, 0);
	vec3_t point_sum = vec3(0, 0, 0);
	float best_dist = INFINITY;

	for (int i = 0; i < 4; i++) {
		vec3_t origin = vec3_add(sh->s.position, vec3_add(
			vec3_mulf(side, px[i] * p->half_extents.x),
			vec3_mulf(sh->s.forward, pz[i] * p->half_extents.z)
		));
		origin = vec3_add(origin, vec3_mulf(up, lift));

		ag_hit_t hit;
		if (
			world_raycast(w, origin, down, ray_len, &hit) &&
			!(hit.surface_flags & AG_SURFACE_WALL) &&
			vec3_dot(hit.normal, up) > 0.35f
		) {
			points[i] = hit.position;
			normal_sum = vec3_add(normal_sum, hit.normal);
			point_sum = vec3_add(point_sum, hit.position);
			g.flags |= hit.surface_flags;
			if (hit.distance < best_dist) {
				best_dist = hit.distance;
				g.material = hit.material;
			}
			g.hits++;
		}
	}

	if (g.hits == 0) {
		return g;
	}

	vec3_t normal = vec3_normalize_or(normal_sum, up);
	if (g.hits == 4) {
		// Fit a plane through the 4 hits; smooths over triangle seams
		vec3_t fit = vec3_cross(vec3_sub(points[1], points[2]), vec3_sub(points[0], points[3]));
		fit = vec3_normalize_or(fit, normal);
		if (vec3_dot(fit, normal) < 0) {
			fit = vec3_mulf(fit, -1);
		}
		normal = vec3_normalize_or(vec3_add(normal, fit), normal);
	}

	vec3_t mean_point = vec3_mulf(point_sum, 1.0f / g.hits);
	g.normal = normal;
	g.height = vec3_dot(vec3_sub(sh->s.position, mean_point), normal);
	g.grounded = true;
	return g;
}

static void ship_damage(ag_world_t *w, int id, ship_t *sh, float amount, vec3_t pos) {
	ag_ship_state_t *s = &sh->s;
	if (!w->desc.use_energy || amount <= 0) {
		return;
	}
	s->energy = maxf(s->energy - amount, 0);
	if (amount >= 1) {
		event_push(w, AG_EVENT_DAMAGE, id, AG_INVALID_SHIP, amount, s->energy, pos, s->up, 0);
	}
	if (s->energy <= 0 && !sh->energy_depleted) {
		sh->energy_depleted = true;
		event_push(w, AG_EVENT_ENERGY_DEPLETED, id, AG_INVALID_SHIP, 0, 0, pos, s->up, 0);
	}
}

static void ship_wall_response(ag_world_t *w, int id, ship_t *sh, vec3_t wall_normal, vec3_t plane_n, ag_hit_t *hit) {
	ag_ship_params_t *p = &sh->params;
	vec3_t v = sh->s.velocity;
	float vn = vec3_dot(v, wall_normal);

	// Scraping along a wall costs speed and energy for as long as it lasts
	float dt = w->desc.fixed_dt;
	vec3_t scrape = vec3_on_plane(vec3_on_plane(v, wall_normal), plane_n);
	v = vec3_sub(v, vec3_mulf(scrape, minf(p->wall_scrape_drag * dt, 1)));
	ship_damage(w, id, sh, p->wall_scrape_damage * dt, hit->position);
	sh->s.velocity = v;
	if (vn >= 0) {
		return;
	}

	float speed = vec3_len(v);
	float sin_a = speed > 1e-4f ? clampf(-vn / speed, 0, 1) : 1;
	ship_damage(w, id, sh, p->wall_damage * (-vn - p->wall_damage_min), hit->position);

	// Remove (and bounce) the velocity into the wall
	v = vec3_sub(v, vec3_mulf(wall_normal, vn * (1 + p->wall_restitution)));

	// Friction scales with the impact angle; scraping along is lossless
	if (sin_a > p->wall_graze) {
		float loss = p->wall_friction * (sin_a - p->wall_graze) / (1 - p->wall_graze);
		vec3_t along = vec3_on_plane(vec3_on_plane(v, wall_normal), plane_n);
		v = vec3_sub(v, vec3_mulf(along, loss));
	}
	sh->s.velocity = v;

	// Turn the nose along the wall instead of spinning out
	vec3_t fwd = vec3_normalize_or(vec3_on_plane(sh->s.forward, plane_n), sh->s.forward);
	if (vec3_dot(fwd, wall_normal) < 0) {
		vec3_t slide = vec3_on_plane(vec3_on_plane(fwd, wall_normal), plane_n);
		slide = vec3_normalize_or(slide, fwd);
		float angle = vec3_signed_angle(fwd, slide, plane_n);
		float turn = angle * clampf(p->wall_align * sin_a * 2.0f, 0, 1);
		sh->s.forward = vec3_rotate(sh->s.forward, plane_n, turn);
		sh->s.up = vec3_rotate(sh->s.up, plane_n, turn);
	}
	sh->yaw_rate *= 1.0f - 0.6f * sin_a;

	if (-vn > 1.0f * w->meter && sh->wall_event_cooldown <= 0) {
		sh->wall_event_cooldown = 0.15f;
		event_push(w, AG_EVENT_WALL_HIT, id, AG_INVALID_SHIP, -vn, sin_a, hit->position, wall_normal, hit->material);
	}
}

static bool hit_is_wall(ag_hit_t *hit, vec3_t plane_n) {
	return (hit->surface_flags & AG_SURFACE_WALL) || vec3_dot(hit->normal, plane_n) < 0.5f;
}

static void ship_collide_walls(ag_world_t *w, int id, ship_t *sh, vec3_t prev_position, vec3_t plane_n) {
	ag_ship_params_t *p = &sh->params;
	float hx = p->half_extents.x;
	float hz = p->half_extents.z;

	// Continuous check along the motion, so fast craft can't tunnel
	vec3_t move = vec3_sub(sh->s.position, prev_position);
	float move_len = vec3_len(move);
	if (move_len > 1e-4f) {
		vec3_t dir = vec3_mulf(move, 1.0f / move_len);
		ag_hit_t hit;
		if (world_raycast(w, prev_position, dir, move_len + hx, &hit) && hit_is_wall(&hit, plane_n)) {
			vec3_t wn = vec3_normalize_or(vec3_on_plane(hit.normal, plane_n), vec3_mulf(dir, -1));
			float back = (move_len + hx) - hit.distance;
			float pen = back * maxf(0.25f, -vec3_dot(dir, wn));
			if (pen > 0) {
				sh->s.position = vec3_add(sh->s.position, vec3_mulf(wn, pen));
				ship_wall_response(w, id, sh, wn, plane_n, &hit);
			}
		}
	}

	// Probe around the hull ellipse
	vec3_t fwd = vec3_normalize_or(vec3_on_plane(sh->s.forward, plane_n), sh->s.forward);
	vec3_t side = vec3_cross(plane_n, fwd);

	typedef struct { vec3_t normal; float pen; ag_hit_t hit; } contact_t;
	contact_t contacts[8];
	int contacts_len = 0;

	for (int i = 0; i < 8; i++) {
		float a = i * (float)M_PI * 0.25f;
		float ca = cosf(a), sa = sinf(a);
		vec3_t dir = vec3_add(vec3_mulf(fwd, ca), vec3_mulf(side, sa));
		float ext = 1.0f / sqrtf((ca / hz) * (ca / hz) + (sa / hx) * (sa / hx));

		ag_hit_t hit;
		if (!world_raycast(w, sh->s.position, dir, ext, &hit) || !hit_is_wall(&hit, plane_n)) {
			continue;
		}
		vec3_t wn = vec3_normalize_or(vec3_on_plane(hit.normal, plane_n), vec3_mulf(dir, -1));
		float pen = (ext - hit.distance) * maxf(0.25f, -vec3_dot(dir, wn));
		if (pen <= 0) {
			continue;
		}

		// Insert sorted, deepest first
		int k = contacts_len++;
		while (k > 0 && contacts[k - 1].pen < pen) {
			contacts[k] = contacts[k - 1];
			k--;
		}
		contacts[k] = (contact_t){wn, pen, hit};
	}

	vec3_t pushed = vec3(0, 0, 0);
	vec3_t responded[8];
	int responded_len = 0;
	for (int i = 0; i < contacts_len; i++) {
		contact_t *c = &contacts[i];
		float remaining = c->pen - vec3_dot(pushed, c->normal);
		if (remaining > 0) {
			vec3_t push = vec3_mulf(c->normal, remaining);
			sh->s.position = vec3_add(sh->s.position, push);
			pushed = vec3_add(pushed, push);
		}

		// One velocity response per distinct wall
		bool seen = false;
		for (int j = 0; j < responded_len; j++) {
			if (vec3_dot(responded[j], c->normal) > 0.9f) {
				seen = true;
			}
		}
		if (!seen) {
			responded[responded_len++] = c->normal;
			ship_wall_response(w, id, sh, c->normal, plane_n, &c->hit);
		}
	}
}


// Craft step -------------------------------------------------------------------

static void ship_ai_input(ag_world_t *w, int id, ship_t *sh, float dt, ag_input_t *in);

static void world_learn_pad(ag_world_t *w, float progress, float lateral) {
	if (!w->route) {
		return;
	}
	for (int i = 0; i < w->pads_len; i++) {
		float d = fabsf(w->pads[i].progress - progress);
		if (w->desc.route_closed) {
			d = minf(d, w->route_length - d);
		}
		if (d < 15 * w->meter && fabsf(w->pads[i].lateral - lateral) < 4 * w->meter) {
			return;
		}
	}
	if (w->pads_len < (int)(sizeof(w->pads) / sizeof(w->pads[0]))) {
		w->pads[w->pads_len].progress = progress;
		w->pads[w->pads_len].lateral = lateral;
		w->pads_len++;
	}
}

static void ship_step(ag_world_t *w, int id, ship_t *sh, float dt) {
	ag_ship_params_t *p = &sh->params;
	ag_ship_state_t *s = &sh->s;

	ag_input_t in = sh->input;
	if (sh->is_ai) {
		ship_ai_input(w, id, sh, dt, &in);
	}
	in.steer = clampf(in.steer, -1, 1);
	in.pitch = clampf(in.pitch, -1, 1);
	in.thrust = clampf(in.thrust, 0, 1);
	in.airbrake_left = clampf(in.airbrake_left, 0, 1);
	in.airbrake_right = clampf(in.airbrake_right, 0, 1);
	if (s->locked) {
		in.steer = in.pitch = in.airbrake_left = in.airbrake_right = 0;
		in.sideshift = 0;
		in.barrel_roll = in.hyperthrust = false;
	}

	// Sideshift requests: a double tap on an airbrake, or the explicit input
	float now = w->time;
	int shift_request = 0, shift_source = 0;
	bool left_down = in.airbrake_left > 0.5f, right_down = in.airbrake_right > 0.5f;
	if (left_down && !sh->left_was_down) {
		if (now - sh->left_tap_time < p->double_tap_window) {
			shift_request = shift_source = -1;
			sh->left_tap_time = -1e9f;
		}
		else {
			sh->left_tap_time = now;
		}
	}
	if (right_down && !sh->right_was_down) {
		if (now - sh->right_tap_time < p->double_tap_window) {
			shift_request = shift_source = 1;
			sh->right_tap_time = -1e9f;
		}
		else {
			sh->right_tap_time = now;
		}
	}
	sh->left_was_down = left_down;
	sh->right_was_down = right_down;

	int shift_input = in.sideshift > 0 ? 1 : (in.sideshift < 0 ? -1 : 0);
	if (shift_input != 0 && shift_input != sh->prev_sideshift_input) {
		shift_request = shift_input;
		shift_source = 0;
	}
	sh->prev_sideshift_input = shift_input;

	bool roll_request = in.barrel_roll && !sh->prev_barrel_roll_input;
	sh->prev_barrel_roll_input = in.barrel_roll;

	if (sh->sideshift_cooldown > 0) {
		sh->sideshift_cooldown -= dt;
	}
	if (shift_request && s->sideshift_time <= 0 && sh->sideshift_cooldown <= 0 && sh->was_grounded) {
		sh->sideshift_dir = shift_request * (int)w->hand;
		sh->sideshift_source = shift_source;
		sh->sideshift_elapsed = 0;
		s->sideshift_time = p->sideshift_max_time;
		event_push(w, AG_EVENT_SIDESHIFT, id, AG_INVALID_SHIP, 0, (float)shift_request, s->position, s->up, 0);
	}
	if (s->sideshift_time > 0) {
		// A tap shifts for sideshift_time; holding the second tap goes further
		sh->sideshift_elapsed += dt;
		int user_dir = sh->sideshift_dir * (int)w->hand;
		bool held = sh->sideshift_source == 0
			? shift_input == user_dir
			: (sh->sideshift_source < 0 ? left_down : right_down);
		if (sh->sideshift_elapsed >= p->sideshift_max_time || (sh->sideshift_elapsed >= p->sideshift_time && !held)) {
			s->sideshift_time = 0;
			sh->sideshift_cooldown = p->sideshift_cooldown;
		}
		else {
			s->sideshift_time = p->sideshift_max_time - sh->sideshift_elapsed;
		}
	}
	// Airbrakes deploy over time, so the taps of a sideshift barely act
	float brake_l = in.airbrake_left, brake_r = in.airbrake_right;
	if (s->sideshift_time > 0) {
		brake_l = brake_r = 0;
		sh->brake_left = sh->brake_right = 0;
	}
	sh->brake_left = approachf(sh->brake_left, brake_l, (brake_l > sh->brake_left ? p->airbrake_gain : p->airbrake_falloff) * dt);
	sh->brake_right = approachf(sh->brake_right, brake_r, (brake_r > sh->brake_right ? p->airbrake_gain : p->airbrake_falloff) * dt);
	in.airbrake_left = sh->brake_left;
	in.airbrake_right = sh->brake_right;

	// Engine
	float spool = in.thrust > s->engine ? p->spool_up : p->spool_down;
	s->engine = approachf(s->engine, in.thrust, spool * dt);

	if (s->boost_time > 0) {
		s->boost_time = maxf(s->boost_time - dt, 0);
	}
	if (s->stall_time > 0) {
		s->stall_time = maxf(s->stall_time - dt, 0);
	}
	sh->wall_event_cooldown -= dt;
	sh->ship_event_cooldown -= dt;

	float thrust_scale = sh->thrust_scale * (sh->is_ai ? sh->ai_thrust_scale : 1.0f);
	if (s->stall_time > 0) {
		thrust_scale *= 0.25f;
	}
	if (s->locked) {
		thrust_scale = 0;
	}

	// Airbrakes in internal (side) terms
	float brake_side = w->hand > 0 ? in.airbrake_right : in.airbrake_left;
	float brake_other = w->hand > 0 ? in.airbrake_left : in.airbrake_right;
	float steer = in.steer * w->hand;

	// Both airbrakes brake; a single one mostly turns and costs little speed
	float both_brakes = minf(in.airbrake_left, in.airbrake_right);
	float brake_drag = p->airbrake_drag * both_brakes +
		p->airbrake_turn_drag * (in.airbrake_left + in.airbrake_right - 2 * both_brakes);

	// Hyperthrust burns energy for speed
	float hyper_accel = 0;
	s->hyperthrusting = false;
	if (w->desc.use_energy && in.hyperthrust && s->energy > p->hyperthrust_min * p->energy_max) {
		hyper_accel = p->hyperthrust_accel;
		s->energy = maxf(s->energy - p->hyperthrust_drain * dt, 0);
		s->hyperthrusting = true;
	}

	float top = p->top_speed;
	float c_lin = p->thrust_accel * p->drag_linear_share / top;
	float c_quad = p->thrust_accel * (1 - p->drag_linear_share) / (top * top);

	vec3_t prev_position = s->position;
	ground_t g = ship_probe_ground(w, sh);
	vec3_t yaw_axis;
	vec3_t plane_n;

	if (g.grounded) {
		vec3_t n = g.normal;
		plane_n = yaw_axis = n;
		float vn = vec3_dot(s->velocity, n);
		vec3_t vt = vec3_on_plane(s->velocity, n);

		if (!sh->was_grounded) {
			if (-vn > 2.0f * w->meter) {
				event_push(w, AG_EVENT_LANDED, id, AG_INVALID_SHIP, -vn, 0, s->position, n, g.material);
			}
			s->air_time = 0;

			// Coming down hard costs speed (and energy)
			float excess = -vn - p->landing_speed_min;
			float speed_t = vec3_len(vt);
			if (excess > 0 && speed_t > 1e-3f) {
				vt = vec3_mulf(vt, maxf(speed_t - p->landing_speed_loss * excess, 0) / speed_t);
				ship_damage(w, id, sh, p->landing_damage * excess, s->position);
			}

			// A barrel roll pays off only if it was completed before landing
			if (sh->roll_dir != 0) {
				bool clean = s->barrel_roll >= 1;
				if (clean) {
					s->boost_time = maxf(s->boost_time, p->barrel_roll_boost_time);
					sh->boost_accel = p->barrel_roll_boost_accel;
					sh->roll_angle = 0;
					sh->prev_roll_angle -= 2 * (float)M_PI * sh->roll_dir; // Same orientation, no unwinding
				}
				else {
					sh->roll_angle = remainderf(sh->roll_angle, 2 * (float)M_PI);
				}
				event_push(w, AG_EVENT_BARREL_ROLL, id, AG_INVALID_SHIP, 0, clean ? 1 : 0, s->position, n, g.material);
				sh->roll_dir = 0;
				s->barrel_roll = 0;
			}
		}
		sh->flicks = 0;
		sh->last_flick = 0;

		bool maglock = (g.flags & AG_SURFACE_MAGLOCK) != 0;
		s->maglocked = maglock;

		// Hover spring; downforce is capped by the magnet, or by plain gravity
		// on jump ramps so the craft flies off the lip
		float k = p->hover_stiffness;
		float c = 2.0f * p->hover_damping * sqrtf(k);
		float accel_n = k * (p->hover_height - g.height) - c * vn;
		float min_accel = -p->magnet;
		if (maglock) {
			min_accel = -p->maglock_magnet;
		}
		else if (g.flags & AG_SURFACE_NO_MAGNET) {
			min_accel = -fabsf(vec3_dot(w->desc.gravity, n));
		}
		accel_n = maxf(accel_n, min_accel);
		vn += accel_n * dt;

		// Never sink into the surface
		float min_height = p->hover_height * 0.3f;
		if (g.height < min_height) {
			s->position = vec3_add(s->position, vec3_mulf(n, min_height - g.height));
			if (vn < 0) {
				vn *= -0.1f;
			}
		}

		// Velocity follows the heading without losing speed; airbraking on
		// the inside of a turn lowers the grip and lets the craft drift
		vec3_t fwd = vec3_normalize_or(vec3_on_plane(s->forward, n), s->forward);
		float speed = vec3_len(vt);
		float drift = (steer > 0 ? brake_side : (steer < 0 ? brake_other : 0)) * fabsf(steer);
		float grip = p->grip * lerpf(1, p->airbrake_grip, drift) * (maglock ? p->maglock_grip : 1);
		if (speed > 1e-3f) {
			vec3_t dir = vec3_mulf(vt, 1.0f / speed);
			vec3_t side = vec3_cross(n, fwd);
			float slip = fabsf(vec3_dot(dir, side));
			dir = vec3_normalize_or(vec3_lerp(dir, fwd, blend(grip, dt)), fwd);
			speed *= 1.0f - p->slip_scrub * slip * dt;
			vt = vec3_mulf(dir, speed);
		}

		if ((g.flags & AG_SURFACE_PIT) && w->desc.use_energy) {
			s->energy = minf(s->energy + p->pit_recharge * dt, p->energy_max);
			sh->energy_depleted = s->energy <= 0;
		}

		// Boost pads
		if (g.flags & AG_SURFACE_BOOST) {
			if (!sh->on_boost_pad) {
				world_learn_pad(w, s->route_progress, s->lateral_offset * w->hand);
				vt = vec3_add(vt, vec3_mulf(fwd, p->boost_pad_kick));
				event_push(w, AG_EVENT_BOOST_PAD, id, AG_INVALID_SHIP, 0, 0, s->position, n, g.material);
			}
			sh->on_boost_pad = true;
			s->boost_time = maxf(s->boost_time, p->boost_pad_time);
			sh->boost_accel = p->boost_pad_accel;
		}
		else {
			sh->on_boost_pad = false;
		}

		// Thrust, boost and hills; mag-strips cancel the hills
		vec3_t accel = vec3_mulf(fwd, p->thrust_accel * s->engine * thrust_scale + hyper_accel);
		if (s->boost_time > 0) {
			accel = vec3_add(accel, vec3_mulf(fwd, sh->boost_accel));
		}
		float gravity_mag = vec3_len(w->desc.gravity);
		if (gravity_mag > 1e-6f && !maglock) {
			vec3_t slope = vec3_mulf(vec3_on_plane(w->desc.gravity, n), p->track_gravity / gravity_mag * p->slope_gravity);
			accel = vec3_add(accel, slope);

			// Nose down on descents to pick up more speed
			float downhill = vec3_dot(slope, fwd);
			if (downhill > 0 && in.pitch < 0) {
				accel = vec3_add(accel, vec3_mulf(fwd, downhill * p->dive_slope_bonus * -in.pitch));
			}
		}

		// Drag, integrated implicitly so it can't overshoot
		float drag_mult = lerpf(p->coast_drag, 1, s->engine) * (1 - s->draft * p->draft_drag_reduction);
		float drag = (c_lin + c_quad * speed) * drag_mult + brake_drag;
		vt = vec3_mulf(vec3_add(vt, vec3_mulf(accel, dt)), 1.0f / (1 + drag * dt));

		if (s->locked) {
			vt = vec3_mulf(vt, 1 - blend(8, dt));
		}

		s->velocity = vec3_add(vt, vec3_mulf(n, vn));
		s->ground_normal = n;
		s->ground_height = g.height;
	}
	else {
		vec3_t gup = w->gravity_up;
		plane_n = gup;
		yaw_axis = s->up;
		s->air_time += dt;
		sh->on_boost_pad = false;
		s->maglocked = false;

		// Barrel roll: steering left-right-left (or right-left-right) quickly
		if (sh->roll_dir == 0) {
			int flick = in.steer > 0.6f ? 1 : (in.steer < -0.6f ? -1 : 0);
			int roll_user_dir = in.steer < 0 ? -1 : 1;
			if (flick != 0 && flick != sh->last_flick) {
				if (sh->flicks == 0 || now - sh->first_flick_time > p->gesture_window) {
					sh->flicks = 0;
					sh->first_flick_time = now;
				}
				sh->flicks++;
				sh->last_flick = flick;
				if (sh->flicks >= 3) {
					roll_request = true;
					roll_user_dir = flick;
				}
			}
			bool affordable = !w->desc.use_energy || s->energy >= p->barrel_roll_cost;
			if (roll_request && affordable) {
				if (w->desc.use_energy) {
					s->energy -= p->barrel_roll_cost;
				}
				sh->roll_dir = (float)(roll_user_dir * (int)w->hand);
				sh->flicks = 0;
				s->barrel_roll = 1e-4f;
				event_push(w, AG_EVENT_BARREL_ROLL_START, id, AG_INVALID_SHIP, 0, (float)roll_user_dir, s->position, s->up, 0);
			}
		}
		if (sh->roll_dir != 0) {
			s->barrel_roll = minf(s->barrel_roll + dt / p->barrel_roll_time, 1);
			sh->roll_angle = s->barrel_roll * 2 * (float)M_PI * sh->roll_dir;
		}

		// Limited air control over the horizontal flight direction
		vec3_t vh = vec3_on_plane(s->velocity, gup);
		vec3_t fwd_h = vec3_on_plane(s->forward, gup);
		float speed_h = vec3_len(vh);
		if (speed_h > 1e-3f && vec3_len(fwd_h) > 0.1f) {
			vec3_t dir = vec3_mulf(vh, 1.0f / speed_h);
			dir = vec3_normalize_or(vec3_lerp(dir, vec3_normalize_or(fwd_h, dir), blend(p->grip * p->air_grip, dt)), dir);
			s->velocity = vec3_add(vec3_mulf(dir, speed_h), vec3_mulf(gup, vec3_dot(s->velocity, gup)));
		}

		vec3_t accel = vec3_add(w->desc.gravity, vec3_mulf(s->forward, p->thrust_accel * s->engine * thrust_scale + hyper_accel));
		if (s->boost_time > 0) {
			accel = vec3_add(accel, vec3_mulf(s->forward, sh->boost_accel));
		}
		float speed = vec3_len(s->velocity);
		float drag = ((c_lin + c_quad * speed) + brake_drag) * p->air_drag;
		s->velocity = vec3_mulf(vec3_add(s->velocity, vec3_mulf(accel, dt)), 1.0f / (1 + drag * dt));

		s->ground_normal = gup;
		s->ground_height = INFINITY;
	}

	// Yaw
	float speed_plane = vec3_len(vec3_on_plane(s->velocity, plane_n));
	float speed_frac = clampf(speed_plane / top, 0, 1);
	float turn_max = p->turn_rate * lerpf(1, p->turn_rate_at_top_speed, speed_frac);
	float steer_amount = powf(fabsf(steer), p->steer_curve);
	float brake_yaw = (brake_side - brake_other) * p->airbrake_yaw * clampf(speed_plane / (0.3f * top), 0, 1);
	float yaw_target = signf(steer) * steer_amount * turn_max + brake_yaw;
	if (!g.grounded) {
		yaw_target *= 0.6f;
	}

	if (fabsf(steer) > 0.01f || fabsf(brake_yaw) > 1e-3f) {
		float accel = p->turn_accel;
		if (sh->yaw_rate * yaw_target < 0) {
			accel *= 2; // Counter steering is quicker
		}
		sh->yaw_rate = approachf(sh->yaw_rate, yaw_target, accel * dt);
	}
	else {
		sh->yaw_rate *= 1 - blend(p->turn_return, dt);
	}
	if (s->locked) {
		sh->yaw_rate = 0;
	}

	float yaw = sh->yaw_rate * dt;
	s->forward = vec3_rotate(s->forward, yaw_axis, yaw);
	s->up = vec3_rotate(s->up, yaw_axis, yaw);

	// Orientation
	if (g.grounded) {
		ship_align_up(sh, g.normal, blend(p->align_rate, dt));
		sh->pitch_rate = 0;
	}
	else {
		sh->pitch_rate = approachf(sh->pitch_rate, in.pitch * p->air_pitch_rate, 8 * dt);
		vec3_t side = ship_side(sh);
		s->forward = vec3_rotate(s->forward, side, -sh->pitch_rate * dt);
		s->up = vec3_rotate(s->up, side, -sh->pitch_rate * dt);

		if (vec3_len(s->velocity) > 0.05f * top) {
			ship_align_forward(sh, vec3_normalize_or(s->velocity, s->forward), blend(p->air_weathervane, dt));
		}

		// Level the roll (not the pitch)
		vec3_t level_up = vec3_normalize_or(vec3_on_plane(w->gravity_up, s->forward), s->up);
		ship_align_up(sh, level_up, blend(p->air_level_rate, dt));
	}
	ship_orthonormalize(sh, w->gravity_up);

	// Visual bank into turns and airbrakes
	float bank_target = clampf(sh->yaw_rate / p->turn_rate, -1, 1) * p->bank_max;
	s->bank = lerpf(s->bank, bank_target, blend(p->bank_rate, dt));

	if (sh->roll_dir == 0) {
		sh->roll_angle *= 1 - blend(10, dt); // Settle after a failed roll
	}

	// Integrate; sideshifts move the craft without touching its velocity
	s->position = vec3_add(s->position, vec3_mulf(s->velocity, dt));
	if (s->sideshift_time > 0) {
		vec3_t fwd_plane = vec3_normalize_or(vec3_on_plane(s->forward, plane_n), s->forward);
		vec3_t side = vec3_cross(plane_n, fwd_plane);
		s->position = vec3_add(s->position, vec3_mulf(side, sh->sideshift_dir * p->sideshift_speed * dt));
	}
	ship_collide_walls(w, id, sh, prev_position, plane_n);

	sh->was_grounded = g.grounded;
	s->grounded = g.grounded;

	if (g.flags & AG_SURFACE_KILL) {
		s->out_of_bounds = true;
	}
}


// Craft vs. craft --------------------------------------------------------------

// Off-center hits (towards the nose or tail) spin the craft
static void ship_hit_spin(ship_t *sh, vec3_t toward_other, vec3_t delta_velocity) {
	float hz = sh->params.half_extents.z;
	vec3_t offset = vec3_mulf(sh->s.forward, vec3_dot(toward_other, sh->s.forward) * hz);
	float torque = vec3_dot(vec3_cross(offset, delta_velocity), sh->s.up) / hz;
	sh->yaw_rate += sh->params.ship_hit_spin * torque;
}

static void ships_collide(ag_world_t *w) {
	for (int a = 0; a < AG_MAX_SHIPS; a++) {
		ship_t *sa = &w->ships[a];
		if (!sa->active) {
			continue;
		}
		for (int b = a + 1; b < AG_MAX_SHIPS; b++) {
			ship_t *sb = &w->ships[b];
			if (!sb->active) {
				continue;
			}
			vec3_t d = vec3_sub(sb->s.position, sa->s.position);
			float dist = vec3_len(d);
			float min_dist = sa->params.radius + sb->params.radius;
			if (dist >= min_dist || dist < 1e-6f) {
				continue;
			}

			vec3_t n = vec3_mulf(d, 1.0f / dist);
			float inv_a = 1.0f / maxf(sa->params.mass, 1e-3f);
			float inv_b = 1.0f / maxf(sb->params.mass, 1e-3f);
			float inv_sum = inv_a + inv_b;

			float pen = min_dist - dist;
			sa->s.position = vec3_sub(sa->s.position, vec3_mulf(n, pen * inv_a / inv_sum));
			sb->s.position = vec3_add(sb->s.position, vec3_mulf(n, pen * inv_b / inv_sum));

			float v_rel = vec3_dot(vec3_sub(sb->s.velocity, sa->s.velocity), n);
			if (v_rel >= 0) {
				continue;
			}
			float j = -(1 + 0.35f) * v_rel / inv_sum;
			vec3_t dv_a = vec3_mulf(n, -j * inv_a);
			vec3_t dv_b = vec3_mulf(n, j * inv_b);
			sa->s.velocity = vec3_add(sa->s.velocity, dv_a);
			sb->s.velocity = vec3_add(sb->s.velocity, dv_b);
			ship_hit_spin(sa, n, dv_a);
			ship_hit_spin(sb, vec3_mulf(n, -1), dv_b);

			vec3_t hit_point = vec3_add(sa->s.position, vec3_mulf(n, sa->params.radius));
			ship_damage(w, a, sa, sa->params.ship_damage * (-v_rel - sa->params.ship_damage_min), hit_point);
			ship_damage(w, b, sb, sb->params.ship_damage * (-v_rel - sb->params.ship_damage_min), hit_point);

			if (-v_rel > 1.0f * w->meter && sa->ship_event_cooldown <= 0 && sb->ship_event_cooldown <= 0) {
				sa->ship_event_cooldown = sb->ship_event_cooldown = 0.15f;
				vec3_t contact = vec3_add(sa->s.position, vec3_mulf(n, sa->params.radius));
				event_push(w, AG_EVENT_SHIP_HIT, a, b, -v_rel, 0, contact, n, 0);
			}
		}
	}
}

static void ships_update_draft(ag_world_t *w) {
	for (int a = 0; a < AG_MAX_SHIPS; a++) {
		ship_t *sa = &w->ships[a];
		if (!sa->active) {
			continue;
		}
		float draft = 0;
		for (int b = 0; b < AG_MAX_SHIPS; b++) {
			ship_t *sb = &w->ships[b];
			if (b == a || !sb->active || sb->s.speed < 0.3f * sb->params.top_speed) {
				continue;
			}
			vec3_t d = vec3_sub(sb->s.position, sa->s.position);
			float along = vec3_dot(d, sa->s.forward);
			if (along <= 0 || along > sa->params.draft_length || vec3_dot(sa->s.forward, sb->s.forward) < 0.7f) {
				continue;
			}
			float lateral = vec3_len(vec3_sub(d, vec3_mulf(sa->s.forward, along)));
			if (lateral >= sa->params.draft_width) {
				continue;
			}
			float strength = (1 - along / sa->params.draft_length) * (1 - lateral / sa->params.draft_width);
			draft = maxf(draft, strength);
		}
		sa->s.draft = draft;
	}
}


// Race -------------------------------------------------------------------------

static void ship_update_race(ag_world_t *w, int id, ship_t *sh, float dt) {
	ag_ship_state_t *s = &sh->s;
	ag_ship_params_t *p = &sh->params;

	if (!s->locked) {
		s->lap_time += dt;
	}

	if (w->route) {
		float progress, dist;
		sh->route_seg = route_nearest(w, s->position, sh->route_seg, &progress, &dist);

		if (w->desc.route_closed) {
			float delta = progress - s->route_progress;
			if (delta < -w->route_length * 0.5f) {
				s->lap++;
				if (s->lap > sh->max_lap) {
					sh->max_lap = s->lap;
					if (s->lap >= 2) {
						s->last_lap_time = s->lap_time;
						if (s->best_lap_time <= 0 || s->lap_time < s->best_lap_time) {
							s->best_lap_time = s->lap_time;
						}
						event_push(w, AG_EVENT_LAP, id, AG_INVALID_SHIP, 0, s->lap_time, s->position, s->up, 0);
					}
					s->lap_time = 0;
				}
			}
			else if (delta > w->route_length * 0.5f) {
				s->lap--;
			}
			s->route_progress = progress;
			s->total_progress = (s->lap - 1) * w->route_length + progress;
		}
		else {
			s->route_progress = progress;
			s->total_progress = progress;
			if (progress >= w->route_length - 1e-3f && sh->max_lap < 2) {
				sh->max_lap = s->lap = 2;
				s->last_lap_time = s->best_lap_time = s->lap_time;
				event_push(w, AG_EVENT_LAP, id, AG_INVALID_SHIP, 0, s->lap_time, s->position, s->up, 0);
			}
		}

		vec3_t r_pos, r_dir, r_up;
		float r_width;
		ag_route_sample(w, progress, &r_pos, &r_dir, &r_up, &r_width);
		vec3_t r_side = vec3_cross(r_up, r_dir);
		vec3_t rel = vec3_sub(s->position, r_pos);
		float lateral = vec3_dot(rel, r_side);

		// Invisible walls along the route edges
		float limit = r_width * 0.5f - p->half_extents.x;
		if (w->desc.route_walls && limit > 0 && fabsf(lateral) > limit) {
			float side = signf(lateral);
			vec3_t wall_normal = vec3_mulf(r_side, -side);
			s->position = vec3_add(s->position, vec3_mulf(wall_normal, fabsf(lateral) - limit));
			ag_hit_t hit = {
				.position = vec3_add(r_pos, vec3_mulf(r_side, side * r_width * 0.5f)),
				.normal = wall_normal,
				.surface_flags = AG_SURFACE_WALL,
			};
			ship_wall_response(w, id, sh, wall_normal, r_up, &hit);
			lateral = side * limit;
		}
		s->lateral_offset = lateral * w->hand;

		if (s->grounded && !s->out_of_bounds && fabsf(lateral) < r_width * 0.5f + p->half_extents.x) {
			sh->last_valid_progress = progress;
			sh->last_valid_lateral = lateral;
		}

		// Wrong way
		vec3_t v_plane = vec3_on_plane(s->velocity, r_up);
		float speed = vec3_len(v_plane);
		if (speed > 0.1f * p->top_speed && vec3_dot(v_plane, r_dir) < -0.3f * speed) {
			sh->wrong_way_time += dt;
		}
		else {
			sh->wrong_way_time = 0;
			if (s->wrong_way && vec3_dot(v_plane, r_dir) > 0) {
				s->wrong_way = false;
				event_push(w, AG_EVENT_WRONG_WAY, id, AG_INVALID_SHIP, 0, 0, s->position, s->up, 0);
			}
		}
		if (sh->wrong_way_time > 1.5f && !s->wrong_way) {
			s->wrong_way = true;
			event_push(w, AG_EVENT_WRONG_WAY, id, AG_INVALID_SHIP, 0, 1, s->position, s->up, 0);
		}

		// Way off the route, or far below it
		float far = r_width * 0.5f + 70.0f * w->meter;
		if (dist > far || vec3_dot(rel, r_up) < -40.0f * w->meter) {
			s->out_of_bounds = true;
		}
	}

	if (s->air_time > 6.0f) {
		s->out_of_bounds = true;
	}

	// Stuck against something
	if (!s->locked && s->engine > 0.5f && s->speed < 0.02f * p->top_speed) {
		sh->stuck_time += dt;
	}
	else {
		sh->stuck_time = 0;
	}

	bool can_respawn = w->route && (sh->is_ai || w->desc.auto_respawn);
	if (s->out_of_bounds) {
		if (sh->out_of_bounds_time == 0) {
			event_push(w, AG_EVENT_OUT_OF_BOUNDS, id, AG_INVALID_SHIP, 0, 0, s->position, s->up, 0);
		}
		sh->out_of_bounds_time += dt;
		if (can_respawn && sh->out_of_bounds_time >= w->desc.respawn_delay) {
			ag_ship_respawn(w, id);
		}
	}
	else if (can_respawn && sh->stuck_time > 3.0f) {
		ag_ship_respawn(w, id);
	}
}

static void ships_update_ranks(ag_world_t *w) {
	for (int a = 0; a < AG_MAX_SHIPS; a++) {
		ship_t *sa = &w->ships[a];
		if (!sa->active) {
			continue;
		}
		int rank = 1;
		for (int b = 0; b < AG_MAX_SHIPS; b++) {
			ship_t *sb = &w->ships[b];
			if (b != a && sb->active && (
				sb->s.total_progress > sa->s.total_progress ||
				(sb->s.total_progress == sa->s.total_progress && b < a)
			)) {
				rank++;
			}
		}
		sa->s.rank = rank;
	}
}


// AI ---------------------------------------------------------------------------

static float route_ahead(ag_world_t *w, float from, float to) {
	float d = to - from;
	if (w->desc.route_closed) {
		d = fmodf(d, w->route_length);
		if (d < -w->route_length * 0.5f) d += w->route_length;
		if (d > w->route_length * 0.5f) d -= w->route_length;
	}
	return d;
}

static void ship_ai_input(ag_world_t *w, int id, ship_t *sh, float dt, ag_input_t *in) {
	ag_ship_state_t *s = &sh->s;
	ag_ship_params_t *p = &sh->params;
	ag_ai_params_t *ai = &sh->ai;
	float m = w->meter;
	float skill = clampf(ai->skill, 0, 1);
	float tricks = clampf(ai->tricks, 0, 1);
	float aggression = clampf(ai->aggression, 0, 1);
	*in = (ag_input_t){0};
	sh->ai_thrust_scale = ai->speed_scale;

	// Rev to the start boost window during the countdown
	if (s->locked) {
		float target = (p->start_boost_min + p->start_boost_max) * 0.5f;
		in->thrust = s->engine < target ? 1 : 0;
		sh->ai_start_timer = ai->start_reaction;
		return;
	}
	if (!w->route) {
		in->thrust = 1;
		return;
	}
	if (sh->ai_start_timer > 0) {
		sh->ai_start_timer -= dt;
		in->thrust = s->engine < p->start_boost_max ? 1 : 0;
		return;
	}

	float speed = s->speed;
	float progress = s->route_progress;
	vec3_t r_pos, r_dir, r_up;
	float r_width;
	ag_route_sample(w, progress, &r_pos, &r_dir, &r_up, &r_width);
	float half_width = r_width * 0.5f;
	float max_lateral = maxf(0, half_width - p->half_extents.x * 2.5f);
	float my_lateral = s->lateral_offset * w->hand;
	float clear = p->radius * 2.6f;

	// Speed plan: the slowest corner within braking distance limits us.
	// Skilled drivers lean on the airbrakes for rotation and corner faster.
	float steer_rate = p->turn_rate;
	float brake_rate = p->airbrake_yaw * (0.25f + 0.45f * skill);
	float steer_falloff = steer_rate * (1 - p->turn_rate_at_top_speed) / p->top_speed;
	float brake_decel = (p->airbrake_drag + p->thrust_accel * p->drag_linear_share / p->top_speed) * p->top_speed * 0.4f;
	float margin = 0.75f + 0.15f * skill;
	float step = 10 * m;
	float horizon = minf(maxf(60 * m, speed * speed / (2 * brake_decel) + 40 * m), 400 * m);
	// Crests: going over faster than the magnet can hold means flying off
	// (and maybe over the walls). Never slower than crest_floor, so real
	// jumps still get taken at speed.
	float magnet = s->maglocked ? p->maglock_magnet : p->magnet;
	float crest_floor = p->top_speed * 0.7f;

	float target_speed = INFINITY;
	vec3_t dir_a = r_dir;
	vec3_t pos_a, pos_b;
	ag_route_sample(w, progress, &pos_a, NULL, NULL, NULL);
	ag_route_sample(w, progress + step, &pos_b, NULL, NULL, NULL);
	for (float d = step; d <= horizon; d += step) {
		vec3_t dir_b, up_b, pos_c;
		ag_route_sample(w, progress + d, NULL, &dir_b, &up_b, NULL);
		ag_route_sample(w, progress + d + step, &pos_c, NULL, NULL, NULL);
		float curvature = fabsf(vec3_signed_angle(dir_a, dir_b, up_b)) / step;
		dir_a = dir_b;

		float allowed_at = INFINITY;
		if (curvature > 1e-5f) {
			allowed_at = (steer_rate + brake_rate) / (curvature + steer_falloff) * margin;
		}

		vec3_t v1 = vec3_sub(pos_b, pos_a), v2 = vec3_sub(pos_c, pos_b);
		float l1 = vec3_len(v1), l2 = vec3_len(v2);

		// The smoothed directions blur tight S-bends; the raw positions don't
		vec3_t h1 = vec3_on_plane(v1, up_b), h2 = vec3_on_plane(v2, up_b);
		if (vec3_len(h1) > 1e-3f && vec3_len(h2) > 1e-3f) {
			float raw = fabsf(vec3_signed_angle(h1, h2, up_b)) / step;
			if (raw > curvature) {
				curvature = raw;
				allowed_at = (steer_rate + brake_rate) / (curvature + steer_falloff) * margin;
			}
		}

		if (l1 > 1e-3f && l2 > 1e-3f) {
			float crest = (vec3_dot(v1, up_b) / l1 - vec3_dot(v2, up_b) / l2) / step;
			if (crest > 1e-5f) {
				allowed_at = minf(allowed_at, maxf(sqrtf(magnet / crest) * margin, crest_floor));
			}
		}
		pos_a = pos_b;
		pos_b = pos_c;

		if (allowed_at < INFINITY) {
			float allowed = sqrtf(allowed_at * allowed_at + 2 * brake_decel * maxf(d - step, 0));
			target_speed = minf(target_speed, allowed);
		}
	}
	bool straight = target_speed > speed * 1.15f;

	// Lateral target: preferred offset plus cutting into upcoming corners
	vec3_t dir_near, dir_far;
	ag_route_sample(w, progress + 10 * m + 0.3f * speed, NULL, &dir_near, NULL, NULL);
	ag_route_sample(w, progress + 10 * m + 0.3f * speed + maxf(40 * m, speed), NULL, &dir_far, NULL, NULL);
	float turn_ahead = vec3_signed_angle(dir_near, dir_far, r_up);
	float turn_now = vec3_signed_angle(r_dir, dir_near, r_up);
	float cut = clampf(turn_ahead / 0.6f, -1, 1) * ai->racing_line * 0.6f * half_width;
	if (turn_now * turn_ahead < 0 && fabsf(turn_now) > 0.1f && fabsf(turn_ahead) > 0.1f) {
		cut *= 0.2f; // S-bend: stay central rather than swinging across
	}
	float lateral = ai->line_offset * 0.5f * half_width + cut;

	// Hunt boost pads other craft have found
	if (tricks > 0) {
		float best = INFINITY;
		float reach = 10 * m + speed * (0.5f + 0.7f * tricks);
		for (int i = 0; i < w->pads_len; i++) {
			float ahead = route_ahead(w, progress, w->pads[i].progress);
			float shift = fabsf(w->pads[i].lateral - my_lateral);
			if (ahead > 8 * m && ahead < reach && ahead < best && shift < ahead * 0.25f && fabsf(w->pads[i].lateral) <= max_lateral + p->half_extents.x) {
				best = ahead;
				lateral = lerpf(lateral, w->pads[i].lateral, tricks);
			}
		}
	}

	// Traffic: draft and overtake craft ahead, keep clear of craft alongside,
	// block craft trying to pass
	int shift_dir = 0; // Internal side sign
	bool battle = false; // Someone to pass or hold off
	for (int b = 0; b < AG_MAX_SHIPS; b++) {
		ship_t *o = &w->ships[b];
		if (b == id || !o->active || o->s.out_of_bounds) {
			continue;
		}
		float ahead = o->s.total_progress - s->total_progress;
		float o_lateral = o->s.lateral_offset * w->hand;
		float gap = my_lateral - o_lateral;
		float away = gap >= 0 ? 1 : -1;
		if (fabsf(o_lateral + away * clear) > max_lateral + p->half_extents.x) {
			away = -away; // No room on that side
		}
		bool alongside = fabsf(ahead) < (p->half_extents.z + o->params.half_extents.z) * 1.2f;

		if (fabsf(ahead) < 60 * m) {
			battle = true;
		}

		if (ahead > 0 && ahead < p->draft_length * 0.9f && straight && tricks > 0 && o->s.speed > speed * 0.9f) {
			if (ahead > 18 * m) {
				lateral = lerpf(lateral, o_lateral, tricks * 0.8f); // Tuck into the slipstream
			}
			else {
				lateral = o_lateral + away * clear; // Slingshot past
				if (fabsf(gap) < clear * 0.8f && tricks > 0.3f) {
					shift_dir = (int)away;
				}
			}
		}
		else if (alongside || (ahead > 0 && ahead < 30 * m && o->s.speed < speed)) {
			if (fabsf(gap) < clear) {
				lateral = o_lateral + away * clear;
				float closing = speed - o->s.speed;
				if (!alongside && ahead < 12 * m && closing > 10 * m && tricks > 0.2f) {
					shift_dir = (int)away; // Dodge
				}
			}
		}
		else if (ahead < 0 && ahead > -25 * m && o->s.speed > speed + 2 * m && aggression > 0) {
			lateral = lerpf(lateral, o_lateral, aggression * 0.6f); // Block
		}
	}
	lateral = clampf(lateral, -max_lateral, max_lateral);
	sh->ai_lateral = approachf(sh->ai_lateral, lateral, 8 * m * dt);

	// Steer towards a look-ahead point; shorter in tight bends so it doesn't
	// cut across them
	float look_ahead = 12 * m + 0.35f * speed;
	vec3_t t_pos, t_dir, t_up;
	ag_route_sample(w, progress + look_ahead, NULL, &t_dir, NULL, NULL);
	float bend = fabsf(vec3_signed_angle(r_dir, t_dir, r_up));
	look_ahead = maxf(look_ahead / (1 + bend), 10 * m);
	ag_route_sample(w, progress + look_ahead, &t_pos, &t_dir, &t_up, NULL);

	// Aim past the target lateral to cancel sideways drift relative to the
	// route, so the craft doesn't carry momentum into the far wall
	float lateral_speed = vec3_dot(s->velocity, vec3_cross(r_up, r_dir));
	float predicted = my_lateral + lateral_speed * 0.35f;
	float aim = sh->ai_lateral + clampf((sh->ai_lateral - predicted) * 0.6f, -half_width, half_width);
	t_pos = vec3_add(t_pos, vec3_mulf(vec3_cross(t_up, t_dir), aim));

	vec3_t n = s->grounded ? s->ground_normal : s->up;
	vec3_t fwd = vec3_normalize_or(vec3_on_plane(s->forward, n), s->forward);
	vec3_t to_target = vec3_normalize_or(vec3_on_plane(vec3_sub(t_pos, s->position), n), fwd);
	float angle = vec3_signed_angle(fwd, to_target, n);

	// Steer by where we're actually going, so sliding doesn't cause overshoot
	vec3_t travel = vec3_on_plane(s->velocity, n);
	vec3_t travel_dir = vec3_len(travel) > 0.1f * p->top_speed ? vec3_normalize_or(travel, fwd) : fwd;
	float travel_angle = vec3_signed_angle(travel_dir, to_target, n);
	float steer = clampf(travel_angle * (2.0f + skill) - sh->yaw_rate * 0.25f, -1, 1);

	// Throttle off just above the target speed, both airbrakes when well above it
	float over = target_speed < INFINITY ? (speed - target_speed) / target_speed : -1;
	in->thrust = clampf(1 - (over + 0.02f) * 25, 0, 1);
	float slow_down = clampf((over - 0.03f) * 8, 0, 1);

	// Inside airbrake when the bend needs more rotation than steering gives,
	// or when way off the heading
	vec3_t dir_turn;
	float turn_dist = maxf(15 * m, speed * 0.25f);
	ag_route_sample(w, progress + turn_dist, NULL, &dir_turn, NULL, NULL);
	float yaw_needed = speed * fabsf(vec3_signed_angle(r_dir, dir_turn, r_up)) / turn_dist;
	float steer_cap = steer_rate * (1 - (1 - p->turn_rate_at_top_speed) * minf(speed / p->top_speed, 1));
	float rotate = clampf((yaw_needed * 1.1f - steer_cap * 0.9f) / maxf(p->airbrake_yaw, 1e-3f), 0, 1) * (0.5f + 0.5f * skill);
	float turn_in = clampf((fabsf(angle) - 0.3f) * 2, 0, 1);
	float inside = maxf(rotate, turn_in);
	float brake_side = slow_down, brake_other = slow_down;
	if (steer > 0) {
		brake_side = maxf(brake_side, inside);
	}
	else {
		brake_other = maxf(brake_other, inside);
	}

	// Energy reserve: tricks that cost energy (or risk contact) need a margin,
	// since only the host can recharge it (pickups, pit lanes)
	float reserve = w->desc.use_energy ? s->energy / maxf(p->energy_max, 1e-3f) : 1;

	// Sideshift: one step of input starts a tap
	if (shift_dir != 0 && s->grounded && reserve > 0.4f) {
		in->sideshift = shift_dir * (int)w->hand;
	}

	// Barrel roll when the jump is long enough to finish it
	bool energy_ok = !w->desc.use_energy || (reserve > 0.7f && s->energy > p->barrel_roll_cost);
	if (!s->grounded && tricks > 0.3f && sh->roll_dir == 0 && energy_ok) {
		ag_hit_t hit;
		vec3_t down = vec3_mulf(w->gravity_up, -1);
		float g = vec3_len(w->desc.gravity);
		if (g > 0 && world_raycast(w, s->position, down, 300 * m, &hit)) {
			float h = maxf(hit.distance - p->hover_height, 0);
			float vz = vec3_dot(s->velocity, w->gravity_up);
			float t_land = (vz + sqrtf(vz * vz + 2 * g * h)) / g;
			in->barrel_roll = t_land > p->barrel_roll_time * 1.25f;
		}
	}

	// Hyperthrust on straights to win a battle, while the shield can spare it
	if (w->desc.use_energy && tricks > 0.5f) {
		sh->ai_hyperthrust_cooldown -= dt;
		if (
			!sh->ai_hyperthrust && sh->ai_hyperthrust_cooldown <= 0 && battle && straight &&
			speed > p->top_speed * 0.9f && reserve > 0.9f - 0.15f * aggression
		) {
			sh->ai_hyperthrust = true;
		}
		if (sh->ai_hyperthrust && (!straight || reserve < 0.75f)) {
			sh->ai_hyperthrust = false;
			sh->ai_hyperthrust_cooldown = 20;
		}
		in->hyperthrust = sh->ai_hyperthrust;
	}

	// Rubber banding against the leading human
	if (ai->catchup > 0) {
		float leader = -INFINITY;
		for (int b = 0; b < AG_MAX_SHIPS; b++) {
			if (w->ships[b].active && !w->ships[b].is_ai) {
				leader = maxf(leader, w->ships[b].s.total_progress);
			}
		}
		if (leader > -INFINITY) {
			float behind = clampf((leader - s->total_progress) / (150 * m), -1, 1);
			sh->ai_thrust_scale *= 1 + ai->catchup * behind;
		}
	}

	in->steer = steer * w->hand;
	in->airbrake_right = w->hand > 0 ? brake_side : brake_other;
	in->airbrake_left = w->hand > 0 ? brake_other : brake_side;
}


// Step -------------------------------------------------------------------------

static void world_substep(ag_world_t *w, float dt) {
	for (int i = 0; i < AG_MAX_SHIPS; i++) {
		ship_t *sh = &w->ships[i];
		if (sh->active) {
			sh->prev_position = sh->s.position;
			sh->prev_forward = sh->s.forward;
			sh->prev_up = sh->s.up;
			sh->prev_bank = sh->s.bank;
			sh->prev_roll_angle = sh->roll_angle;
		}
	}

	ships_update_draft(w);

	for (int i = 0; i < AG_MAX_SHIPS; i++) {
		if (w->ships[i].active) {
			ship_step(w, i, &w->ships[i], dt);
		}
	}

	ships_collide(w);

	for (int i = 0; i < AG_MAX_SHIPS; i++) {
		if (w->ships[i].active) {
			ship_update_race(w, i, &w->ships[i], dt);
			ship_update_public(w, &w->ships[i]);
		}
	}

	ships_update_ranks(w);
	w->time += dt;
}

int ag_world_step(ag_world_t *w, float frame_dt) {
	float fixed = w->desc.fixed_dt;
	w->accumulator += clampf(frame_dt, 0, fixed * w->desc.max_substeps);

	int steps = 0;
	while (w->accumulator >= fixed && steps < w->desc.max_substeps) {
		world_substep(w, fixed);
		w->accumulator -= fixed;
		steps++;
	}
	if (w->accumulator >= fixed) {
		w->accumulator = fixed * 0.999f; // Can't keep up; drop time
	}
	return steps;
}


// Craft API --------------------------------------------------------------------

static ship_t *get_ship(ag_world_t *w, ag_ship_id_t id) {
	if (id < 0 || id >= AG_MAX_SHIPS || !w->ships[id].active) {
		return NULL;
	}
	return &w->ships[id];
}

static void ship_reset_motion(ship_t *sh) {
	sh->s.velocity = vec3(0, 0, 0);
	sh->yaw_rate = 0;
	sh->pitch_rate = 0;
	sh->prev_position = sh->s.position;
	sh->prev_forward = sh->s.forward;
	sh->prev_up = sh->s.up;
	sh->prev_bank = sh->s.bank = 0;
	sh->roll_dir = 0;
	sh->roll_angle = sh->prev_roll_angle = 0;
	sh->s.barrel_roll = 0;
	sh->s.sideshift_time = 0;
}

ag_ship_id_t ag_ship_add(ag_world_t *w, const ag_ship_params_t *params, vec3_t position, vec3_t forward) {
	for (int i = 0; i < AG_MAX_SHIPS; i++) {
		ship_t *sh = &w->ships[i];
		if (sh->active) {
			continue;
		}
		memset(sh, 0, sizeof(*sh));
		sh->active = true;
		sh->params = *params;
		sh->thrust_scale = 1;
		sh->ai_thrust_scale = 1;
		sh->route_seg = -1;
		sh->left_tap_time = sh->right_tap_time = -1e9f;
		sh->s.energy = params->energy_max;
		ag_ship_teleport(w, i, position, forward);
		return i;
	}
	return AG_INVALID_SHIP;
}

void ag_ship_remove(ag_world_t *w, ag_ship_id_t id) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->active = false;
	}
}

void ag_ship_set_params(ag_world_t *w, ag_ship_id_t id, const ag_ship_params_t *params) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->params = *params;
		sh->s.energy = minf(sh->s.energy, params->energy_max);
	}
}

void ag_ship_set_input(ag_world_t *w, ag_ship_id_t id, const ag_input_t *input) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->input = *input;
	}
}

void ag_ship_set_ai(ag_world_t *w, ag_ship_id_t id, const ag_ai_params_t *ai) {
	ship_t *sh = get_ship(w, id);
	if (!sh) {
		return;
	}
	sh->is_ai = ai != NULL;
	if (ai) {
		sh->ai = *ai;
		sh->ai_lateral = sh->s.lateral_offset * w->hand;
	}
	sh->ai_thrust_scale = 1;
}

const ag_ship_state_t *ag_ship_state(ag_world_t *w, ag_ship_id_t id) {
	ship_t *sh = get_ship(w, id);
	return sh ? &sh->s : NULL;
}

void ag_ship_get_pose(ag_world_t *w, ag_ship_id_t id, ag_pose_t *pose) {
	ship_t *sh = get_ship(w, id);
	if (!sh) {
		return;
	}
	float t = ag_world_alpha(w);
	vec3_t fwd = vec3_normalize_or(vec3_lerp(sh->prev_forward, sh->s.forward, t), sh->s.forward);
	vec3_t up = vec3_normalize_or(vec3_lerp(sh->prev_up, sh->s.up, t), sh->s.up);
	up = vec3_normalize_or(vec3_on_plane(up, fwd), sh->s.up);
	float bank = lerpf(sh->prev_bank, sh->s.bank, t) + lerpf(sh->prev_roll_angle, sh->roll_angle, t);

	// Positive bank tilts the up vector towards the side of the turn
	up = vec3_rotate(up, fwd, -bank);

	pose->position = vec3_lerp(sh->prev_position, sh->s.position, t);
	pose->forward = fwd;
	pose->up = up;
	pose->right = vec3_mulf(vec3_cross(up, fwd), w->hand);
}

// Find the surface below a point and put the craft at hover height
static void ship_snap_to_ground(ag_world_t *w, ship_t *sh) {
	ag_hit_t hit;
	vec3_t up = sh->s.up;
	float reach = 10 * sh->params.half_extents.z;
	vec3_t origin = vec3_add(sh->s.position, vec3_mulf(up, sh->params.half_extents.z));
	if (world_raycast(w, origin, vec3_mulf(up, -1), reach, &hit) && !hit_is_wall(&hit, up)) {
		sh->s.up = hit.normal;
		sh->s.position = vec3_add(hit.position, vec3_mulf(hit.normal, sh->params.hover_height));
		ship_orthonormalize(sh, w->gravity_up);
	}
}

void ag_ship_teleport(ag_world_t *w, ag_ship_id_t id, vec3_t position, vec3_t forward) {
	ship_t *sh = get_ship(w, id);
	if (!sh) {
		return;
	}
	sh->s.position = position;
	sh->s.forward = forward;
	sh->s.up = w->gravity_up;
	ship_orthonormalize(sh, w->gravity_up);
	ship_snap_to_ground(w, sh);
	ship_reset_motion(sh);

	if (w->route) {
		float progress, dist;
		sh->route_seg = route_nearest(w, sh->s.position, -1, &progress, &dist);
		sh->s.route_progress = progress;
		sh->last_valid_progress = progress;
		sh->last_valid_lateral = 0;
		sh->s.total_progress = w->desc.route_closed ? (sh->s.lap - 1) * w->route_length + progress : progress;
	}
	ship_update_public(w, sh);
}

static void ship_place_on_route(ag_world_t *w, ship_t *sh, float progress, float lateral, float speed) {
	vec3_t pos, dir, up;
	ag_route_sample(w, progress, &pos, &dir, &up, NULL);
	pos = vec3_add(pos, vec3_mulf(vec3_cross(up, dir), lateral));
	pos = vec3_add(pos, vec3_mulf(up, sh->params.hover_height));

	sh->s.position = pos;
	sh->s.forward = dir;
	sh->s.up = up;
	ship_orthonormalize(sh, w->gravity_up);
	ship_snap_to_ground(w, sh);
	ship_reset_motion(sh);
	sh->s.velocity = vec3_mulf(sh->s.forward, speed);

	sh->route_seg = route_seg_at(w, route_wrap(w, progress));
	sh->s.route_progress = route_wrap(w, progress);
	sh->ai_lateral = lateral;
	ship_update_public(w, sh);
}

void ag_ship_place_on_grid(ag_world_t *w, ag_ship_id_t id, int slot) {
	ship_t *sh = get_ship(w, id);
	if (!sh || !w->route) {
		return;
	}
	int row = slot / 2;
	int column = slot % 2;
	float spacing = sh->params.half_extents.z * 5;
	float behind = (row + 1) * spacing + column * spacing * 0.5f;

	float width;
	ag_route_sample(w, 0, NULL, NULL, NULL, &width);
	float lateral = (column == 0 ? -0.25f : 0.25f) * width * w->hand;

	if (w->desc.route_closed) {
		// Just behind the line; crossing it starts lap 1
		sh->s.lap = 0;
		sh->max_lap = 0;
		ship_place_on_route(w, sh, w->route_length - behind, lateral, 0);
		sh->s.total_progress = sh->s.route_progress - w->route_length;
	}
	else {
		sh->s.lap = 1;
		sh->max_lap = 1;
		ship_place_on_route(w, sh, (AG_MAX_SHIPS / 2 + 1) * spacing - behind, lateral, 0);
		sh->s.total_progress = sh->s.route_progress;
	}
	sh->last_valid_progress = sh->s.route_progress;
	sh->last_valid_lateral = lateral;
	sh->s.lap_time = 0;
	sh->s.velocity = vec3(0, 0, 0);
	ship_update_public(w, sh);
}

void ag_ship_set_locked(ag_world_t *w, ag_ship_id_t id, bool locked) {
	ship_t *sh = get_ship(w, id);
	if (!sh || sh->s.locked == locked) {
		return;
	}
	sh->s.locked = locked;
	if (locked) {
		return;
	}

	ag_ship_params_t *p = &sh->params;
	sh->s.lap_time = 0;
	if (sh->s.engine >= p->start_boost_min && sh->s.engine <= p->start_boost_max) {
		sh->s.boost_time = p->start_boost_time;
		sh->boost_accel = p->boost_pad_accel * 0.8f;
		event_push(w, AG_EVENT_START_BOOST, id, AG_INVALID_SHIP, 0, sh->s.engine, sh->s.position, sh->s.up, 0);
	}
	else if (sh->s.engine > p->start_stall_above) {
		sh->s.stall_time = p->start_stall_time;
		event_push(w, AG_EVENT_START_STALL, id, AG_INVALID_SHIP, 0, sh->s.engine, sh->s.position, sh->s.up, 0);
	}
}

static bool route_has_ground(ag_world_t *w, ship_t *sh, float progress, float lateral) {
	vec3_t pos, dir, up;
	ag_route_sample(w, progress, &pos, &dir, &up, NULL);
	pos = vec3_add(pos, vec3_mulf(vec3_cross(up, dir), lateral));
	float reach = sh->params.half_extents.z * 3;
	ag_hit_t hit;
	return world_raycast(w, vec3_add(pos, vec3_mulf(up, reach)), vec3_mulf(up, -1), reach * 2, &hit) &&
		!hit_is_wall(&hit, up) && !(hit.surface_flags & AG_SURFACE_KILL);
}

// Like a rescue drone: if there's a gap right ahead of the last valid
// position (a missed jump), put the craft down on the far side of it
static float route_respawn_progress(ag_world_t *w, ship_t *sh, float from, float lateral) {
	float m = w->meter;
	float step = 5 * m;
	float d = 0;
	bool gap = false;
	for (; d <= 60 * m; d += step) {
		if (!route_has_ground(w, sh, from + d, lateral)) {
			gap = true;
			break;
		}
	}
	if (!gap) {
		return from;
	}
	float solid = 0;
	for (; d <= 400 * m; d += step) {
		solid = route_has_ground(w, sh, from + d, lateral) ? solid + step : 0;
		if (solid >= 20 * m) {
			return route_wrap(w, from + d);
		}
	}
	return from;
}

void ag_ship_respawn(ag_world_t *w, ag_ship_id_t id) {
	ship_t *sh = get_ship(w, id);
	if (!sh || !w->route) {
		return;
	}
	float width;
	ag_route_sample(w, sh->last_valid_progress, NULL, NULL, NULL, &width);
	float lateral = clampf(sh->last_valid_lateral, -width * 0.25f, width * 0.25f);
	float speed = w->desc.respawn_speed * sh->params.top_speed;
	float progress = route_respawn_progress(w, sh, sh->last_valid_progress, lateral);

	int lap = sh->s.lap;
	ship_place_on_route(w, sh, progress, lateral, speed);
	sh->last_valid_progress = sh->s.route_progress;
	sh->s.lap = lap;
	sh->s.total_progress = w->desc.route_closed
		? (lap - 1) * w->route_length + sh->s.route_progress
		: sh->s.route_progress;

	sh->s.out_of_bounds = false;
	sh->s.air_time = 0;
	sh->s.boost_time = 0;
	sh->s.stall_time = 0;
	sh->out_of_bounds_time = 0;
	sh->stuck_time = 0;
	sh->was_grounded = true;
	event_push(w, AG_EVENT_RESPAWNED, id, AG_INVALID_SHIP, 0, 0, sh->s.position, sh->s.up, 0);
}

void ag_ship_apply_impulse(ag_world_t *w, ag_ship_id_t id, vec3_t delta_velocity) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->s.velocity = vec3_add(sh->s.velocity, delta_velocity);
	}
}

void ag_ship_apply_spin(ag_world_t *w, ag_ship_id_t id, float delta_yaw_rate) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->yaw_rate += delta_yaw_rate * w->hand;
	}
}

void ag_ship_boost(ag_world_t *w, ag_ship_id_t id, float accel, float duration) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->boost_accel = accel;
		sh->s.boost_time = maxf(sh->s.boost_time, duration);
	}
}

void ag_ship_set_thrust_scale(ag_world_t *w, ag_ship_id_t id, float scale) {
	ship_t *sh = get_ship(w, id);
	if (sh) {
		sh->thrust_scale = maxf(scale, 0);
	}
}

float ag_ship_add_energy(ag_world_t *w, ag_ship_id_t id, float amount) {
	ship_t *sh = get_ship(w, id);
	if (!sh) {
		return 0;
	}
	ag_ship_state_t *s = &sh->s;
	s->energy = clampf(s->energy + amount, 0, sh->params.energy_max);
	if (s->energy > 0) {
		sh->energy_depleted = false;
	}
	else if (!sh->energy_depleted) {
		sh->energy_depleted = true;
		event_push(w, AG_EVENT_ENERGY_DEPLETED, id, AG_INVALID_SHIP, 0, 0, s->position, s->up, 0);
	}
	return s->energy;
}

ag_ship_id_t ag_ship_find_target(ag_world_t *w, ag_ship_id_t id, float max_distance, float max_angle) {
	ship_t *sh = get_ship(w, id);
	if (!sh) {
		return AG_INVALID_SHIP;
	}
	float min_cos = cosf(max_angle);
	float best = max_distance;
	ag_ship_id_t target = AG_INVALID_SHIP;
	for (int i = 0; i < AG_MAX_SHIPS; i++) {
		ship_t *o = &w->ships[i];
		if (i == id || !o->active || o->s.out_of_bounds) {
			continue;
		}
		vec3_t d = vec3_sub(o->s.position, sh->s.position);
		float dist = vec3_len(d);
		if (dist < 1e-3f || dist >= best || vec3_dot(d, sh->s.forward) < dist * min_cos) {
			continue;
		}
		best = dist;
		target = i;
	}
	return target;
}
