/*
	podracer - Star Wars Episode I: Racer handling as a standalone C99 library

	A port of the podracer flight model from the Racer reverse engineering
	project (github.com/tim-tim707/SW_RACER_RE), wrapped in the same host
	contract as antigrav: the host supplies a raycast against its static world
	and a route along the course; podracer simulates the pods, their race
	progress and the AI, and hands back poses and events.

	Ported from decompiled code (constants and formulas as in the game):
	  - throttle integrator and the speed curve thrust*maxSpeed/(accel+thrust)
	  - coast/air brake decay, start boost window, boost curve, engine heat
	    (0..100) with overheating engines blowing up
	  - traction blend from anti-skid, surface grip and sliding
	  - turning: squared steering, turn response with 5x counter steer,
	    nose up/down trimming throttle and turn rate
	  - surface tags: fast, slow, sand trap, ice, lava, fall, magnet (wall
	    riding), zero-g, swamp, side
	  - hover via the fall accumulator, landings, gliding, slope slide
	  - per-engine damage, steering pull from damaged sides, crash death
	    (impact speed > 325 with a speed drop > 140 at >= 200)
	  - AI pacing (pace-setter rubber banding, rank random walk), spline
	    autopilot steering, proximity avoidance and blocking
	  - pod stat upgrades and the original 23 pod stat table (read from the
	    player's own swep1rcr.exe)

	Reconstructed (not decompiled in the source project yet, or replacing
	data a host route doesn't have), marked RECONSTRUCTED in the code: wall
	collision response, pod vs. pod collision, boost charging, repair while
	driving, the effect of manual tilt on turning, and the AI's corner speed
	planning and curvature feed-forward (the game's AI splines carry these).

	Units: podracer works in Racer units internally (pod speeds ~400-650).
	ag... types are shared with antigrav so one raycast and route serve both.
*/

#ifndef PODRACER_H
#define PODRACER_H

#include "../antigrav/antigrav.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PR_MAX_PODS 24
#define PR_MAX_EVENTS 256
#define PR_NUM_STOCK_PODS 23
#define PR_INVALID_POD -1


// Surfaces ---------------------------------------------------------------------

// Racer's track surface reactions, reported in ag_hit_t.surface_flags. They
// live above antigrav's bits so one raycast can tag surfaces for both.
enum {
	PR_SURFACE_FAST    = 1 << 8,   // Speed bonus (+200), e.g. Ando Prime ice paths, tubes
	PR_SURFACE_SLOW    = 1 << 9,   // Speed x0.75 (sand, gravel); kills boost in zero-g
	PR_SURFACE_SWST    = 1 << 10,  // Sand trap: speed x0.1, kills boost
	PR_SURFACE_SLIP    = 1 << 11,  // Ice: grip x0.2
	PR_SURFACE_LAVA    = 1 << 12,  // Drains engine temperature 20/s (overheats)
	PR_SURFACE_FALL    = 1 << 13,  // Pit: respawn on contact
	PR_SURFACE_MAGNET  = 1 << 14,  // Gravity follows the surface (wall rides, tubes)
	PR_SURFACE_ZON     = 1 << 15,  // Enter zero-g: pitch input pitches the pod
	PR_SURFACE_ZOFF    = 1 << 16,  // Leave zero-g
	PR_SURFACE_SWAMP   = 1 << 17,
	PR_SURFACE_SIDE    = 1 << 18,  // Banked side: AI tilts into it
	PR_SURFACE_WALL    = 1 << 19,  // Force wall treatment
};


// Pod stats --------------------------------------------------------------------

// The game's raw per-pod handling values, in the game's order and units.
typedef struct {
	float anti_skid;             // Traction (upgrade 0): grip, 0..1, higher = less skid
	float turn_response;         // Turning (upgrade 1): how fast the turn rate builds
	float max_turn_rate;         // Turn rate cap
	float acceleration;          // Acceleration (upgrade 2): speed curve denominator, LOWER = quicker
	float max_speed;             // Top Speed (upgrade 3): asymptotic top speed
	float air_brake_inv;         // Air Brake (upgrade 4): brake decay, LOWER = stronger
	float deceleration_interval; // Coast deceleration with the throttle off
	float boost_thrust;          // Boost power
	float heat_rate;             // Engine temperature drain while boosting
	float cool_rate;             // Cooling (upgrade 5): temperature recovery
	float hover_height;          // Ride height
	float repair_rate;           // Repair (upgrade 6)
	float bump_mass;             // Pod vs. pod collision mass
	float damage_immunity;       // Damage MULTIPLIER (higher = more fragile)
	float intersect_radius;      // Collision radius
} pr_pod_stats_t;

typedef enum {
	PR_UPGRADE_TRACTION,
	PR_UPGRADE_TURNING,
	PR_UPGRADE_ACCELERATION,
	PR_UPGRADE_TOP_SPEED,
	PR_UPGRADE_AIR_BRAKE,
	PR_UPGRADE_COOLING,
	PR_UPGRADE_REPAIR,
	PR_NUM_UPGRADES,
} pr_upgrade_t;

// A middle-of-the-field pod for when the original stats aren't available
void pr_pod_stats_default(pr_pod_stats_t *stats);

// Read stock pod `pod` (0..22, game order; see pr_pod_name) from the player's
// own copy of swep1rcr.exe (PC, the patched retail build). Returns false if
// the file doesn't look like that executable.
bool pr_pod_stats_from_exe(const void *exe, size_t exe_size, int pod, pr_pod_stats_t *stats);
const char *pr_pod_name(int pod);

// Apply a parts shop upgrade the way the game does: level 0..5, health 0..1
// (part condition). Call once per category on the stock stats.
void pr_pod_stats_apply_upgrade(pr_pod_stats_t *stats, pr_upgrade_t category, int level, float health);

// The garage's 0..1 stat bars (traction, turning, acceleration, top speed,
// air brake, cooling, repair)
void pr_pod_stats_bars(const pr_pod_stats_t *stats, float bars[PR_NUM_UPGRADES]);


// World ------------------------------------------------------------------------

typedef struct {
	ag_raycast_func_t raycast;  // Static geometry only; host units
	void *user;

	float units_per_racer_unit; // Host units per Racer unit; e.g. ~0.45 for meters
	ag_vec3_t gravity_dir;      // Down; default (0, -1, 0)
	bool right_handed;          // As in antigrav: which way "right" is
	float fixed_dt;             // Default 1/60
	int max_substeps;           // Default 8

	const ag_route_point_t *route; // Host units; copied
	int route_len;
	bool route_closed;

	// AI: base speed multiplier (the game's track level x AI speed setting,
	// 0.85..1.15) and field spread. Each assigned rank holds station
	// 50 * ai_spread Racer units behind the one ahead; the game uses 20..40
	// on its full-length tracks (~30k units a lap). Scale it down with the
	// lap length for short courses.
	float ai_level;
	float ai_spread;

	// Distance based catch-up boost for human pods behind the leading human
	// (the game's splitscreen catch-up, max 1.25x)
	bool human_catchup;

	void *(*alloc)(size_t size, void *user);
	void (*free)(void *ptr, void *user);
	void *alloc_user;
} pr_world_desc_t;

typedef struct pr_world_t pr_world_t;

void pr_world_desc_default(pr_world_desc_t *desc, float units_per_racer_unit);
pr_world_t *pr_world_create(const pr_world_desc_t *desc);
void pr_world_destroy(pr_world_t *world);
int pr_world_step(pr_world_t *world, float frame_dt);
float pr_world_alpha(pr_world_t *world);


// Pods -------------------------------------------------------------------------

typedef int pr_pod_id_t;

typedef struct {
	float steer;            // -1..1, positive = right
	float pitch;            // -1..1, positive = nose DOWN (more throttle, less turning; charges boost)
	float throttle;         // 0..1 (1 = full thrust)
	bool brake;             // Air brake
	bool tilt_left;         // Lean (above 200 speed)
	bool tilt_right;
	bool slide;             // Slide: less grip for tighter, sliding turns
	bool boost;             // Fire a charged boost (re-pressing full throttle also works)
	bool repair;            // Hold to repair damaged engines while driving
} pr_input_t;

typedef enum {
	PR_BOOST_NOT_READY,
	PR_BOOST_CHARGING,
	PR_BOOST_READY,
} pr_boost_state_t;

typedef struct {
	// Motion (host units)
	ag_vec3_t position;
	ag_vec3_t velocity;
	ag_vec3_t forward, up, right;
	float speed;                // Racer units/s, as the speedometer shows
	float turn_rate;            // Degrees/s
	float tilt;                 // Visual bank (degrees), includes manual tilt
	float ground_distance;      // Racer units; large when airborne
	bool airborne;
	bool magnet;                // Riding a magnet surface
	bool zero_g;

	// Engines
	float engine_temp;          // 100 = cool, 0 = overheated
	pr_boost_state_t boost_state;
	bool boosting;
	float engine_damage[6];     // 0..1 per part: left top/mid/bottom, right top/mid/bottom
	bool engine_on_fire[6];
	bool repairing;
	bool dead;                  // Exploded, waiting for respawn

	// Race
	int lap;
	float lap_time;
	float last_lap_time;
	float best_lap_time;
	float route_progress;       // Host units along the route
	float total_progress;
	int rank;
	float pace_multiplier;      // Applied speed multiplier (AI pacing / catch-up)
} pr_pod_state_t;

typedef struct {
	float level;                // Pace multiplier base relative to the world's ai_level
	int rank_baseline;          // Assigned finishing position (1 = win)
	bool pace_setter;           // The track favorite that chases the leading human
} pr_ai_params_t;

pr_pod_id_t pr_pod_add(pr_world_t *world, const pr_pod_stats_t *stats, ag_vec3_t position, ag_vec3_t forward);
void pr_pod_remove(pr_world_t *world, pr_pod_id_t pod);
void pr_pod_set_stats(pr_world_t *world, pr_pod_id_t pod, const pr_pod_stats_t *stats);
void pr_pod_set_input(pr_world_t *world, pr_pod_id_t pod, const pr_input_t *input);
void pr_pod_set_ai(pr_world_t *world, pr_pod_id_t pod, const pr_ai_params_t *ai); // NULL = human
const pr_pod_state_t *pr_pod_state(pr_world_t *world, pr_pod_id_t pod);
void pr_pod_get_pose(pr_world_t *world, pr_pod_id_t pod, ag_pose_t *pose);
void pr_pod_place_on_grid(pr_world_t *world, pr_pod_id_t pod, int slot);
void pr_pod_set_locked(pr_world_t *world, pr_pod_id_t pod, bool locked);   // Countdown
void pr_pod_respawn(pr_world_t *world, pr_pod_id_t pod);

// Host hooks
void pr_pod_damage(pr_world_t *world, pr_pod_id_t pod, int engine, float amount);  // e.g. Tusken shots
void pr_pod_apply_impulse(pr_world_t *world, pr_pod_id_t pod, ag_vec3_t delta_velocity);


// Events -----------------------------------------------------------------------

typedef enum {
	PR_EVENT_WALL_HIT,          // intensity: impact speed
	PR_EVENT_POD_HIT,           // other, intensity
	PR_EVENT_LANDED,            // intensity: landing speed ('HitBotm')
	PR_EVENT_START_BOOST,
	PR_EVENT_BOOST_READY,
	PR_EVENT_BOOST,
	PR_EVENT_OVERHEAT,          // value: engine part that caught fire
	PR_EVENT_ENGINE_DAMAGED,    // value: engine part; intensity: its damage 0..1
	PR_EVENT_EXPLODED,
	PR_EVENT_RESPAWNED,
	PR_EVENT_LAP,               // value: lap time
	PR_EVENT_SURFACE,           // value: the new surface flags under the pod
} pr_event_type_t;

typedef struct {
	pr_event_type_t type;
	pr_pod_id_t pod;
	pr_pod_id_t other;
	float intensity;
	float value;
	ag_vec3_t position;
} pr_event_t;

bool pr_world_poll_event(pr_world_t *world, pr_event_t *event);

#ifdef __cplusplus
}
#endif

#endif
