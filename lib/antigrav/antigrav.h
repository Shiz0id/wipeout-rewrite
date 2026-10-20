/*
	antigrav - anti-gravity racing physics, derived from the wipEout rewrite

	A standalone C99 library (no dependencies besides libm). The host engine
	owns all geometry, rendering, audio and weapons; antigrav owns the craft
	physics, race progress and AI drivers.

	The host provides:
	  - a raycast callback against its static world geometry (no ships)
	  - optionally, a route: a list of waypoints along the racing line, used for
	    lap counting, race positions, respawning and the AI drivers

	All lengths are in host world units. ag_ship_params_default() takes a
	units_per_meter scale so the defaults (tuned in meters) fit any engine.
	The coordinate system can be Y-up, Z-up or anything else; "down" is
	defined solely by ag_world_desc_t.gravity.

	Typical frame:
		ag_ship_set_input(world, player, &input);   // human ships only
		ag_world_step(world, frame_dt);              // fixed sub-steps inside
		while (ag_world_poll_event(world, &ev)) { ...play sounds, fx... }
		ag_ship_get_pose(world, ship, &pose);        // interpolated, for drawing
*/

#ifndef ANTIGRAV_H
#define ANTIGRAV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AG_MAX_SHIPS 32
#define AG_MAX_EVENTS 256
#define AG_INVALID_SHIP -1

typedef struct {
	float x, y, z;
} ag_vec3_t;


// World geometry ---------------------------------------------------------------

// Surface flags the raycast callback may report for a hit. Anything with a
// steep normal is treated as a wall automatically; AG_SURFACE_WALL forces it.
enum {
	AG_SURFACE_WALL      = 1 << 0, // Always treat as wall, regardless of slope
	AG_SURFACE_BOOST     = 1 << 1, // Boost pad
	AG_SURFACE_NO_MAGNET = 1 << 2, // Jump ramps: no downforce, the craft flies off the lip
	AG_SURFACE_KILL      = 1 << 3, // Out of bounds (pits, water); triggers a respawn
};

typedef struct {
	ag_vec3_t position;
	ag_vec3_t normal;        // Unit length, facing the ray origin
	float distance;          // Along the ray, in world units
	uint32_t surface_flags;  // AG_SURFACE_*
	uint32_t material;       // Passed through untouched in events (for sfx/fx)
} ag_hit_t;

// Cast a ray against static world geometry. dir is unit length. Return true and
// fill hit with the nearest hit within max_distance. Must not hit the craft
// themselves; craft vs. craft collisions are handled by antigrav.
typedef bool (*ag_raycast_func_t)(void *user, ag_vec3_t origin, ag_vec3_t dir, float max_distance, ag_hit_t *hit);

typedef struct {
	ag_vec3_t position;  // On the drivable surface, along the racing line
	ag_vec3_t up;        // Surface up; zero vector = opposite of gravity
	float width;         // Full drivable width at this point
} ag_route_point_t;


// World ------------------------------------------------------------------------

typedef struct {
	ag_raycast_func_t raycast;
	void *user;              // Passed to raycast

	float units_per_meter;   // World scale; used by the AI and for event thresholds
	ag_vec3_t gravity;       // Gravity while airborne, e.g. (0, -25, 0) m/s^2 for Y-up meters
	float fixed_dt;          // Physics step; default 1/120
	int max_substeps;        // Max steps per ag_world_step(); default 8

	// Defines which way "right" is, for steering and airbrakes. With
	// right_handed, right = forward x up; otherwise right = up x forward
	// (e.g. Unity, Unreal-style left handed systems).
	bool right_handed;

	// Optional route; copied. Points should be spaced roughly 5-20m apart.
	const ag_route_point_t *route;
	int route_len;
	bool route_closed;       // Circuit (laps) vs. point-to-point

	// For open maps without barriers: invisible walls along the route edges
	// keep craft within the route width. They behave like real walls (events,
	// scraping, bouncing).
	bool route_walls;

	// Respawn craft automatically after they went out of bounds or got stuck.
	// If false, only AG_EVENT_OUT_OF_BOUNDS is emitted and the host calls
	// ag_ship_respawn() itself. AI craft always respawn automatically.
	bool auto_respawn;
	float respawn_delay;     // Seconds; default 1.0

	// Optional allocator; malloc/free when NULL
	void *(*alloc)(size_t size, void *user);
	void (*free)(void *ptr, void *user);
	void *alloc_user;
} ag_world_desc_t;

typedef struct ag_world_t ag_world_t;

void ag_world_desc_default(ag_world_desc_t *desc, float units_per_meter);
ag_world_t *ag_world_create(const ag_world_desc_t *desc);
void ag_world_destroy(ag_world_t *world);

// Advance the simulation by frame_dt seconds, in fixed steps. Returns the
// number of steps taken.
int ag_world_step(ag_world_t *world, float frame_dt);

// Interpolation factor between the last two physics states, 0..1
float ag_world_alpha(ag_world_t *world);


// Craft parameters -------------------------------------------------------------

typedef struct {
	// Body
	float mass;                // Relative; only matters for craft vs. craft hits
	float radius;              // Collision sphere for craft vs. craft
	ag_vec3_t half_extents;    // x: half width, y: half height, z: half length (walls)

	// Engine
	float top_speed;           // Equilibrium speed at full thrust on flat ground
	float thrust_accel;        // Acceleration from standstill at full thrust
	float drag_linear_share;   // 0..1: share of linear vs. quadratic drag at top speed
	float spool_up;            // Engine response when opening the throttle (1/s)
	float spool_down;          // Engine response when closing it (1/s)
	float coast_drag;          // Drag multiplier with the throttle closed

	// Steering
	float turn_rate;           // Max yaw rate at low speed (rad/s)
	float turn_rate_at_top_speed; // Multiplier for turn_rate at top speed (0..1)
	float turn_accel;          // How fast the yaw rate builds up (rad/s^2)
	float turn_return;         // Yaw rate decay without steering (1/s)
	float steer_curve;         // Analog response exponent (1 = linear)
	float grip;                // How fast the velocity follows the heading (1/s)
	float slip_scrub;          // Speed lost while sliding sideways (0..1)

	// Airbrakes
	float airbrake_yaw;        // Extra yaw rate at full airbrake (rad/s)
	float airbrake_drag;       // Drag at full airbrake, both sides (1/s)
	float airbrake_grip;       // Grip multiplier when drifting (airbrake + steer same side)

	// Hover
	float hover_height;        // Target height above the surface
	float hover_stiffness;     // Spring stiffness (1/s^2)
	float hover_damping;       // Damping ratio (1 = critical)
	float hover_probe_length;  // How far below the hover height the ground is found
	float magnet;              // Max downforce keeping the craft on the surface
	float track_gravity;       // Gravity along the surface normal while grounded
	float slope_gravity;       // 0..1: how much hills slow you down / speed you up
	float align_rate;          // How fast the craft aligns to the surface (1/s)

	// Air
	float air_pitch_rate;      // Pitch rate from input while airborne (rad/s)
	float air_weathervane;     // Nose follows the flight path (1/s)
	float air_level_rate;      // Roll levels out while airborne (1/s)
	float air_grip;            // Grip multiplier while airborne
	float air_drag;            // Drag multiplier while airborne

	// Walls
	float wall_restitution;    // Bounce off the wall (0..1)
	float wall_friction;       // Speed lost on a head-on hit (0..1)
	float wall_graze;          // sin(angle) below which a hit is a lossless scrape
	float wall_align;          // How much a hit turns the nose along the wall (0..1)

	// Boost & drafting
	float boost_pad_accel;
	float boost_pad_time;
	float boost_pad_kick;      // Instant speed added when hitting a pad
	float draft_length;        // Slipstream reach behind other craft
	float draft_width;
	float draft_drag_reduction; // 0..1 at point blank

	// Race start: holding the engine in a window when released gives a boost,
	// over-revving stalls the engine
	float start_boost_min;     // Engine fraction 0..1
	float start_boost_max;
	float start_boost_time;
	float start_stall_above;
	float start_stall_time;

	// Cosmetic
	float bank_max;            // Max visual bank angle (rad)
	float bank_rate;           // (1/s)
} ag_ship_params_t;

void ag_ship_params_default(ag_ship_params_t *params, float units_per_meter);


// AI ---------------------------------------------------------------------------

typedef struct {
	float skill;               // 0..1: cornering precision and margin
	float speed_scale;         // Multiplier for thrust (e.g. 0.9..1.05)
	float line_offset;         // -1..1: preferred lateral offset, fraction of half width
	float racing_line;         // 0..1: how much to cut corners
	float catchup;             // 0..: thrust bonus/malus relative to the human leader (rubber banding)
	float start_reaction;      // Seconds before the AI opens the throttle at race start
} ag_ai_params_t;

void ag_ai_params_default(ag_ai_params_t *params);


// Craft ------------------------------------------------------------------------

typedef int ag_ship_id_t;

typedef struct {
	float steer;               // -1..1, positive = right
	float pitch;               // -1..1, positive = nose up (airborne only)
	float thrust;              // 0..1
	float airbrake_left;       // 0..1
	float airbrake_right;      // 0..1
} ag_input_t;

typedef struct {
	// Physics
	ag_vec3_t position;
	ag_vec3_t velocity;
	ag_vec3_t forward, up, right; // Physics frame (no visual bank)
	float speed;
	float yaw_rate;
	float engine;              // 0..1 spooled throttle
	bool grounded;
	ag_vec3_t ground_normal;
	float ground_height;
	float air_time;
	float boost_time;
	float draft;               // 0..1 slipstream strength
	float stall_time;
	float bank;                // Visual bank angle

	// Race
	bool locked;               // Held at the start line
	int lap;                   // 0 before crossing the line for the first time
	float lap_time;
	float last_lap_time;
	float best_lap_time;       // 0 if none
	float route_progress;      // Distance along the route in this lap
	float total_progress;      // Including laps; for race positions
	float lateral_offset;      // From the route center, positive = right
	int rank;                  // 1 = leading
	bool wrong_way;
	bool out_of_bounds;
} ag_ship_state_t;

typedef struct {
	ag_vec3_t position;
	ag_vec3_t forward, up, right; // Includes visual bank, interpolated
} ag_pose_t;

// Returns AG_INVALID_SHIP if AG_MAX_SHIPS is exceeded.
ag_ship_id_t ag_ship_add(ag_world_t *world, const ag_ship_params_t *params, ag_vec3_t position, ag_vec3_t forward);
void ag_ship_remove(ag_world_t *world, ag_ship_id_t ship);
void ag_ship_set_params(ag_world_t *world, ag_ship_id_t ship, const ag_ship_params_t *params);

// Human input; ignored for AI craft
void ag_ship_set_input(ag_world_t *world, ag_ship_id_t ship, const ag_input_t *input);

// Hand the craft to an AI driver, or back to the human with NULL
void ag_ship_set_ai(ag_world_t *world, ag_ship_id_t ship, const ag_ai_params_t *ai);

const ag_ship_state_t *ag_ship_state(ag_world_t *world, ag_ship_id_t ship);
void ag_ship_get_pose(ag_world_t *world, ag_ship_id_t ship, ag_pose_t *pose);

// Place the craft on the start grid behind the first route point. slot 0 is
// pole position.
void ag_ship_place_on_grid(ag_world_t *world, ag_ship_id_t ship, int slot);
void ag_ship_teleport(ag_world_t *world, ag_ship_id_t ship, ag_vec3_t position, ag_vec3_t forward);

// Locked craft hover in place and can rev their engine; unlocking applies the
// start boost/stall rule. Use it for the countdown.
void ag_ship_set_locked(ag_world_t *world, ag_ship_id_t ship, bool locked);

// Back on the route at the last valid position
void ag_ship_respawn(ag_world_t *world, ag_ship_id_t ship);

// Hooks for host weapons and pickups
void ag_ship_apply_impulse(ag_world_t *world, ag_ship_id_t ship, ag_vec3_t delta_velocity);
void ag_ship_apply_spin(ag_world_t *world, ag_ship_id_t ship, float delta_yaw_rate); // rad/s, positive = right
void ag_ship_boost(ag_world_t *world, ag_ship_id_t ship, float accel, float duration);
void ag_ship_set_thrust_scale(ag_world_t *world, ag_ship_id_t ship, float scale);   // e.g. 0.5 while hit by an EMP


// Route ------------------------------------------------------------------------

float ag_route_length(ag_world_t *world);

// Turn a few hand placed control points into a smooth route (centripetal
// Catmull-Rom spline) with a point every `spacing` world units; width and up
// are interpolated. Writes at most out_capacity points to out and returns the
// number of points the whole route needs, so call it with out = NULL first to
// size the buffer. Returns 0 for fewer than 2 control points.
int ag_route_resample(const ag_route_point_t *controls, int controls_len, bool closed, float spacing, ag_route_point_t *out, int out_capacity);
// Sample the route at a distance along it. Any output may be NULL.
void ag_route_sample(ag_world_t *world, float distance, ag_vec3_t *position, ag_vec3_t *direction, ag_vec3_t *up, float *width);


// Events -----------------------------------------------------------------------

typedef enum {
	AG_EVENT_WALL_HIT,         // intensity: impact speed
	AG_EVENT_SHIP_HIT,         // other: the other craft; intensity: impact speed
	AG_EVENT_LANDED,           // intensity: vertical impact speed
	AG_EVENT_BOOST_PAD,
	AG_EVENT_START_BOOST,
	AG_EVENT_START_STALL,
	AG_EVENT_LAP,              // value: lap time of the completed lap; lap in state
	AG_EVENT_WRONG_WAY,        // value: 1 when it starts, 0 when it ends
	AG_EVENT_OUT_OF_BOUNDS,
	AG_EVENT_RESPAWNED,
} ag_event_type_t;

typedef struct {
	ag_event_type_t type;
	ag_ship_id_t ship;
	ag_ship_id_t other;
	float intensity;
	float value;
	ag_vec3_t position;
	ag_vec3_t normal;
	uint32_t material;
} ag_event_t;

bool ag_world_poll_event(ag_world_t *world, ag_event_t *event);

#ifdef __cplusplus
}
#endif

#endif
