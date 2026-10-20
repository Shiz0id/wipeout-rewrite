# antigrav

Anti-gravity racing physics as a standalone C99 library, derived from the
wipEout rewrite's craft code and rebuilt to run on any engine's geometry.

- **No dependencies** besides libm; two files: `antigrav.h`, `antigrav.c`
- **Your geometry**: works on arbitrary map meshes through one raycast callback;
  loops, banks, hills, jumps. Optional `antigrav_mesh` provides a BVH raycaster
  over triangle soups when the engine's own raycast isn't convenient
- **Open maps**: invisible walls along the route edges keep craft on course
  where the map has no barriers; sparse control points become a smooth route
- **Fixed timestep** (120 Hz by default) with interpolated poses; results are
  identical at any frame rate
- **Handling**: grip that keeps your speed through turns, airbrake drifting,
  boosting past top speed, slipstream drafting, start boost / stall
- **Walls you can scrape**: grazing a wall costs almost nothing, a head-on hit
  costs everything; the nose turns along the wall instead of spinning out
- **AI drivers** on the same physics as the player, with speed planning, racing
  lines, overtaking and rubber banding
- **Race logic**: laps, lap times, positions, wrong way detection, respawns
- **Hooks for your weapons** (impulses, spin, boost, thrust scale) and an event
  queue for audio and effects

The host engine keeps everything else: models, rendering, audio, weapons,
pickups, UI and cameras. See `INTEGRATION.md` for a step-by-step wiring and
verification checklist.


## Building

Add the directory to your CMake project:

```cmake
add_subdirectory(path/to/antigrav)
target_link_libraries(your_game PRIVATE antigrav)
```

Or just compile `antigrav.c` (and optionally `antigrav_mesh.c`) as C (C99;
MSVC 2019+ in C mode) with the rest of your sources. The headers are C++
compatible.

Tests: `make test` or `cmake -S . -B build && cmake --build build && ctest --test-dir build`.


## Quick start

```cpp
#include "antigrav.h"

// 1. Raycast against static map geometry (not the craft). host_* stands for
//    whatever the engine provides.
static bool raycast(void *user, ag_vec3_t origin, ag_vec3_t dir, float max_dist, ag_hit_t *hit) {
	host_ray_hit_t h;
	if (!host_raycast_static(user, origin, dir, max_dist, &h)) {
		return false;
	}
	hit->position = h.point;
	hit->normal = h.normal;          // unit length, facing the ray origin
	hit->distance = h.distance;
	hit->material = h.material_id;
	hit->surface_flags = 0;
	if (h.material_flags & HOST_MAT_BOOST)   hit->surface_flags |= AG_SURFACE_BOOST;
	if (h.material_flags & HOST_MAT_JUMP)    hit->surface_flags |= AG_SURFACE_NO_MAGNET;
	if (h.material_flags & HOST_MAT_KILL)    hit->surface_flags |= AG_SURFACE_KILL;
	if (h.material_flags & HOST_MAT_BARRIER) hit->surface_flags |= AG_SURFACE_WALL;
	return true;
}

// 2. Create the world with your route
ag_world_desc_t desc;
ag_world_desc_default(&desc, UNITS_PER_METER);
desc.raycast = raycast;
desc.user = host_world;
desc.route = route_points;      // copied
desc.route_len = route_count;
desc.route_closed = true;
desc.route_walls = true;        // open map without barriers
ag_world_t *world = ag_world_create(&desc);

// 3. Add craft
ag_ship_params_t params;
ag_ship_params_default(&params, UNITS_PER_METER);
ag_ship_id_t player = ag_ship_add(world, &params, start_pos, start_dir);
ag_ship_place_on_grid(world, player, 7);

for (int i = 0; i < 7; i++) {
	ag_ship_id_t bot = ag_ship_add(world, &params, start_pos, start_dir);
	ag_ship_place_on_grid(world, bot, i);
	ag_ai_params_t ai;
	ag_ai_params_default(&ai);
	ai.skill = 0.6f + 0.05f * i;
	ai.line_offset = random_range(-0.5f, 0.5f);
	ag_ship_set_ai(world, bot, &ai);
}

// 4. Countdown: lock everyone, unlock on GO (revving into the window = boost)
for (each ship) ag_ship_set_locked(world, ship, true);
...
for (each ship) ag_ship_set_locked(world, ship, false);

// 5. Every frame
ag_input_t in = {0};
in.steer = pad.left_x;              // -1..1, + = right
in.thrust = pad.right_trigger;      // 0..1
in.airbrake_left = pad.l1;          // 0..1
in.airbrake_right = pad.r1;
in.pitch = pad.left_y;              // nose up/down while airborne
ag_ship_set_input(world, player, &in);

ag_world_step(world, frame_dt);

ag_event_t ev;
while (ag_world_poll_event(world, &ev)) {
	handle_event(ev);               // see Events below
}

for (each ship) {
	ag_pose_t pose;
	ag_ship_get_pose(world, ship, &pose);   // interpolated, with visual bank
	draw_ship(ship, pose.position, pose.forward, pose.up, pose.right);
}
```


## Hooking up maps

### Geometry

antigrav never sees your meshes. It asks for raycasts:

- **Ground**: 4 rays straight down from the hull corners, every step
- **Walls**: 8 rays around the hull at hover height, plus one along the motion
  so fast craft can't pass through thin walls
- **Spawning**: one ray down to find the surface

That's 13 short rays per craft per step, about 1,600 rays per second per craft
at 120 Hz. Use your physics engine's static collision layer, and leave craft,
pickups and triggers out of it.

Rules for map geometry:

- Anything steeper than ~60° relative to the craft is a **wall**; tag barriers
  with `AG_SURFACE_WALL` if they're shallower than that
- Walls must stand taller than the hover height plus the craft's half height
  (~2 m with defaults), or craft will float over them
- Ground normals must face up out of the track; one-sided collision is fine
- Large maps are fine: at 4 km from the origin float precision is still
  sub-millimeter in meters. If the engine uses very small units (e.g.
  centimeters), keep the map centered on the origin

### Mesh collision (optional)

`antigrav_mesh.h` builds a BVH over one or more triangle meshes (with
optional transforms, per part or per triangle surface flags and materials)
and provides `ag_mesh_raycast`, which plugs straight into `desc.raycast`:

```c
ag_mesh_part_t part = {
	.positions = vertices, .vertex_count = vertex_count,
	.indices = indices, .triangle_count = triangle_count,
};
ag_mesh_t *mesh = ag_mesh_create(&part, 1);
desc.raycast = ag_mesh_raycast;
desc.user = mesh;
```

About 80 bytes per triangle and roughly 2 million short rays per second on
one core; feed it the geometry around the course rather than a whole map.

### Surface flags

| Flag | Use for |
|---|---|
| `AG_SURFACE_BOOST` | Boost pads: an instant kick plus extra acceleration while on the pad and for `boost_pad_time` after |
| `AG_SURFACE_NO_MAGNET` | The last stretch before a jump: removes downforce so craft fly off the lip instead of following the ramp down |
| `AG_SURFACE_KILL` | Water, pits, lava: out of bounds, then a respawn |
| `AG_SURFACE_WALL` | Force wall behavior on shallow barriers, curbs or glass |

`material` is passed through untouched in events, so wall hits and landings can
pick the right sounds and sparks.

### The route

The route is a polyline along the circuit, authored per map. You can place a
few control points at the bends and let `ag_route_resample()` turn them into
a smooth, evenly spaced route. antigrav uses it for laps, positions, wrong way
detection, respawning and the AI. Physics works without it, but then there are
no laps and no AI.

- One point every **5-20 m**, more densely in tight turns
- `position`: on the surface along the middle of the drivable area
- `width`: full drivable width at that point; the AI keeps a margin from it
- `up`: the surface up vector for banked turns, loops and wall rides; leave it
  zero on regular ground
- Point 0 is the **start/finish line**; `ag_ship_place_on_grid()` lines craft up
  behind it in two columns
- Branching tracks: route the main line; craft on a shortcut still get progress
  from the nearest point (don't combine shortcuts with `route_walls`)

### Open maps

Maps built for other games rarely have barriers along a race course. Set
`desc.route_walls` and the route edges (`width / 2` from the route line)
become invisible walls that behave like real ones: craft scrape along them,
bounce off them and emit `AG_EVENT_WALL_HIT`. Real walls inside the corridor
still work as usual.


## Events

| Event | Fields | Typical use |
|---|---|---|
| `AG_EVENT_WALL_HIT` | `intensity` = impact speed, `value` = sin of the impact angle, `position`, `normal`, `material` | Scrape/crash sound by intensity, sparks, camera shake, damage |
| `AG_EVENT_SHIP_HIT` | `other`, `intensity` | Bump sound, damage |
| `AG_EVENT_LANDED` | `intensity` = vertical speed | Landing thud, camera dip |
| `AG_EVENT_BOOST_PAD` | | Boost sound, FOV kick |
| `AG_EVENT_START_BOOST` / `AG_EVENT_START_STALL` | `value` = engine level | Start feedback |
| `AG_EVENT_LAP` | `value` = lap time; read `lap` from the state | Lap HUD; race over at `lap > num_laps` |
| `AG_EVENT_WRONG_WAY` | `value` 1 = starts, 0 = ends | HUD warning |
| `AG_EVENT_OUT_OF_BOUNDS` | | Fade out, rescue drone |
| `AG_EVENT_RESPAWNED` | | Fade in |

The queue holds 256 events; poll it every frame.


## Weapons and pickups

antigrav doesn't know about weapons; the host applies their effects to craft:

```c
// Missile hit: knock back, spin and choke the engine for a moment
ag_ship_apply_impulse(world, target, scale(missile_dir, 25 * m));
ag_ship_apply_spin(world, target, random_sign() * 3.0f);
ag_ship_set_thrust_scale(world, target, 0.3f);   // reset to 1 after 1.5s

// Turbo pickup
ag_ship_boost(world, ship, 90 * m, 1.5f);

// Blaster hit
ag_ship_apply_impulse(world, target, scale(shot_dir, 8 * m));
```

For projectiles, use the craft's `position`, `velocity` and `forward` from
`ag_ship_state()`. `rank` and `total_progress` tell you who's ahead for homing
weapons, and `lateral_offset` tells you where on the track each craft is.


## Tuning

All defaults are in `ag_ship_params_default()`, tuned in meters. Numbers from
the test report with defaults:

| | |
|---|---|
| 0-100 m/s | 2.5 s |
| Top speed | 145 m/s (522 km/h); ~190 m/s with boost pads and drafting |
| Turn radius at top speed | 97 m full steer; 26 m drifting with the airbrake (losing half the speed) |
| Wall hit at 10° / 30° / 90° | keeps 97% / 73% / 8% of the speed |
| AI average race speed | ~88% of top speed on the test circuit |

What to change for what you feel:

| Feel | Parameters |
|---|---|
| Faster overall | `top_speed`, `thrust_accel` (raise them together) |
| Punchier launch | `thrust_accel`, `spool_up`, lower `drag_linear_share` |
| Speed bleeds off when coasting | `coast_drag` |
| Turns quicker / slower | `turn_rate`, `turn_rate_at_top_speed` |
| Steering responsiveness | `turn_accel` (how fast it bites), `turn_return` (how fast it lets go), `steer_curve` (analog precision near center) |
| Less floaty / more planted | raise `grip` |
| Drifts longer | lower `airbrake_grip`, raise `airbrake_yaw` |
| Airbrakes slow you more | `airbrake_drag` |
| Walls more / less punishing | `wall_friction`, `wall_graze`, `wall_restitution`; `wall_align` controls how much a hit straightens you out |
| Sticks to the track over crests | `magnet` |
| Hills matter more | `slope_gravity` |
| Bigger boosts | `boost_pad_accel`, `boost_pad_time`, `boost_pad_kick` |
| Stronger slipstream | `draft_drag_reduction`, `draft_length` |
| Jumps hang longer | lower `desc.gravity` |

Craft classes and teams are just different parameter sets, and
`ag_ship_set_params()` can change them at any time (e.g. for upgrades).

AI difficulty: `skill` (cornering margin and precision), `speed_scale` (raw
pace), `racing_line` (corner cutting), `catchup` (e.g. 0.05 for 5% rubber banding
against the leading human).


## Sense of speed is half camera

The physics gives you the speed; a few host-side tricks make it feel like it:

- Widen the FOV with `speed / top_speed`, plus a kick while `boost_time > 0`
- Let the chase camera lag behind on acceleration and catch up on braking
- A small high-frequency shake that grows with speed; a bigger one on wall hits
- Engine pitch from `engine` and `speed`; wind noise from `speed`
- Lean the camera with the pose's bank (`pose.up`)
- Speed lines or motion blur above ~70% of top speed
- Track-side objects close to the racing line (pillars, signs, arches) show off
  speed much more than open space does


## Origins

The hover, magnet, thrust/drag and airbrake concepts come from the PSX
wipEout craft code (`src/wipeout/ship*.c` in this repository). antigrav swaps
its fixed-point, frame-rate dependent integration and section/face track
format for a generic, fixed-step model, and replaces the AI's fake physics
with real drivers.
