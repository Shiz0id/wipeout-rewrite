# Integrating antigrav into a host engine

A checklist for wiring antigrav into an existing engine (written for the
Selotape integration, but nothing here is engine specific). Read `README.md`
for what the library does; this file covers how to hook it up and how to
verify each step.

antigrav is self-contained C99. It owns craft physics, race progress and AI.
The host owns everything else: rendering, audio, input, weapons, pickups, UI,
cameras, networking and map loading.


## 0. Vendor the files

Copy `lib/antigrav/` into the host tree. Required: `antigrav.h`, `antigrav.c`.
Optional: `antigrav_mesh.h`, `antigrav_mesh.c` (only if you use step 3B).
Compile them as C99; there are no other dependencies besides libm. Either use
the bundled `CMakeLists.txt` (`add_subdirectory`) or add the `.c` files to the
host build directly.

Before touching the engine, build and run the tests once to confirm the
toolchain: `make test` in `lib/antigrav` must end in `PASSED`.


## 1. Units and axes

Three facts about the host decide the setup:

| Question | Where it goes |
|---|---|
| How many world units is one meter? | `ag_world_desc_default(&desc, units_per_meter)` and `ag_ship_params_default(&params, units_per_meter)`. Every default length, speed and acceleration scales with it. |
| Which way is up? | `desc.gravity` points down, e.g. `(0, -25 * upm, 0)` for Y-up or `(0, 0, -25 * upm)` for Z-up. Nothing else in antigrav assumes an axis. |
| Is the engine right handed? | `desc.right_handed`. Verify it in step 7: with `steer = +1` the craft must turn toward its visual right. If it turns left, flip the flag. |

If the host's vector type isn't `{float x, y, z}`, write two tiny conversion
functions; antigrav only exchanges `ag_vec3_t` by value.


## 2. World lifetime

- Create one `ag_world_t` per race, after the map is loaded:
  `ag_world_create(&desc)`. Destroy it when the race ends.
- The world isn't thread safe. Call everything from the thread that runs game
  logic; it's cheap (about 70 µs per step for 8 craft, including raycasts).
- The route (step 4) is copied at creation; the host's buffer can be freed
  afterwards.


## 3. Collision: pick A or B

antigrav needs one function that casts a ray against **static** world geometry:

```c
bool raycast(void *user, ag_vec3_t origin, ag_vec3_t dir, float max_distance, ag_hit_t *hit);
```

Contract:
- `dir` is unit length; return the **nearest** hit within `max_distance`
- `hit->normal` is unit length and faces the ray origin (flip it if needed)
- `hit->distance` is along the ray in world units; `hit->position` is the point
- Must **not** hit the craft themselves, other vehicles, characters,
  projectiles, pickups or triggers. Craft vs. craft is handled inside antigrav.
- Rays are short (< 15 m with defaults, except the one-off spawn snap), about
  1,600 per craft per second

### A. Use the host's raycast

Wrap the engine's existing ray query against its static collision layer.
Map materials to surface flags (step 5) and pass the material id through in
`hit->material`.

### B. Use antigrav_mesh

If the engine's raycast is awkward to call (wrong layer filtering, too slow,
not reachable from game code), build a dedicated BVH from triangle data:

```c
ag_mesh_part_t parts[N];   // one per collision mesh / instance
parts[i] = (ag_mesh_part_t){
	.positions = vertex_floats, .position_stride = vertex_size_in_bytes,
	.vertex_count = vertex_count,
	.indices = indices_u32, .triangle_count = triangle_count,
	.transform = instance_matrix,   // 4x4, column-major unless .row_major
	.surface_flags = flags_for_this_part,
	.material = material_id,
};
ag_mesh_t *mesh = ag_mesh_create(parts, N);
desc.raycast = ag_mesh_raycast;
desc.user = mesh;
...
ag_mesh_destroy(mesh);     // after ag_world_destroy
```

- Memory is ~80 bytes per triangle. For big maps, only include geometry near
  the course (e.g. instances whose bounds are within ~50 m of a route point).
- Terrain heightfields: triangulate the cells near the course into a part,
  or skip the mesh for terrain and write a raycast that first checks the
  heightfield, then calls `ag_mesh_raycast` for props, and returns the nearer
  hit.
- Triangles are double sided, so winding doesn't matter.


## 4. The route

The route is required for laps, positions, respawning, route walls and AI.
Battlefront maps have no race course, so each race needs one authored per map.

1. Pick control points along the course: on the ground, along the middle of
   the drivable corridor, one at every bend (every 50-200 m is fine). Record
   `position`, `width` (corridor width) and `up` (zero unless banked, a loop
   or a wall ride).
2. Store them as data per map/course (JSON, a Lua table, whatever the host
   uses for level data).
3. Densify at load time:

```c
int n = ag_route_resample(controls, control_count, true, 10 * upm, NULL, 0);
ag_route_point_t *route = malloc(sizeof(*route) * n);
ag_route_resample(controls, control_count, true, 10 * upm, route, n);
desc.route = route;
desc.route_len = n;
desc.route_closed = true;     // false for point-to-point
desc.route_walls = true;      // open maps: invisible walls at the route edges
```

Point 0 is the start/finish line, and the grid lines up behind it, so place
the first control point where the start line should be, on a straight.

Tip: an in-engine "record route" debug mode (drive or walk the course and
drop a control point every few seconds) makes authoring fast.


## 5. Surface flags

From the host's material or collision flags, set:

| Flag | When |
|---|---|
| `AG_SURFACE_WALL` | Barriers that aren't steep (glass, low railings, curbs) |
| `AG_SURFACE_BOOST` | Boost pads (can be decals placed for the racing mode) |
| `AG_SURFACE_NO_MAGNET` | The last few meters before a jump lip |
| `AG_SURFACE_KILL` | Water, lava, death pits, out of map volumes with geometry |

Everything else needs no flag: walkable ground is ground, steep geometry is
a wall.


## 6. Craft

```c
ag_ship_params_t params;
ag_ship_params_default(&params, upm);
// Tune per vehicle class: see the Tuning table in README.md
ag_ship_id_t id = ag_ship_add(world, &params, pos, forward);
ag_ship_place_on_grid(world, id, slot);   // slot 0 = pole
```

AI craft: `ag_ship_set_ai(world, id, &ai)` after `ag_ai_params_default(&ai)`.
Give each AI a slightly different `skill`, `speed_scale` and `line_offset` so
they don't drive in a line.

Collision sizes are in `params.radius` (craft vs. craft) and
`params.half_extents` (walls). Match them to the host's vehicle model.


## 7. Per frame

```c
// Input (human craft only)
ag_input_t in = {
	.steer = stick_x,                 // -1..1, + = right
	.thrust = accelerate,             // 0..1
	.airbrake_left = left_brake,      // 0..1
	.airbrake_right = right_brake,    // 0..1
	.pitch = stick_y,                 // nose up/down in the air
};
ag_ship_set_input(world, player, &in);

ag_world_step(world, frame_dt);       // once per frame; fixed sub-steps inside

ag_event_t ev;
while (ag_world_poll_event(world, &ev)) { /* audio, fx, HUD; see README */ }

for each craft:
	ag_pose_t pose;
	ag_ship_get_pose(world, id, &pose);   // interpolated; includes visual bank
	// Build the render transform from pose.right, pose.up, pose.forward and
	// pose.position, in the host's matrix convention. If the vehicle model's
	// forward axis isn't the host's forward, add a fixed model rotation.
```

Race state for HUD, weapons and camera: `ag_ship_state(world, id)`; it holds
speed, lap, lap times, rank, wrong way, boost and grounded state.


## 8. Race flow

1. Spawn all craft on the grid, `ag_ship_set_locked(world, id, true)`
2. Countdown: players can rev; revving into the window gives a start boost,
   over-revving stalls
3. On GO: `ag_ship_set_locked(world, id, false)` for all
4. `AG_EVENT_LAP`: a lap is complete when `state->lap` goes up; with N laps,
   a craft has finished when `state->lap > N`
5. Out of bounds respawns automatically (`desc.auto_respawn`), or handle
   `AG_EVENT_OUT_OF_BOUNDS` and call `ag_ship_respawn()` yourself


## 9. Weapons

The host's weapons stay the host's. Apply their effects through:
`ag_ship_apply_impulse`, `ag_ship_apply_spin`, `ag_ship_boost` and
`ag_ship_set_thrust_scale`. Read positions and velocities from
`ag_ship_state()` to aim and to spawn projectiles. Examples are in README.md.


## 10. Verify, in this order

| Step | Expect | If not |
|---|---|---|
| Spawn on flat ground, no input | Craft settles ~1.2 m (scaled) above the ground and stays still | Floats away or falls through: raycast normal/distance wrong, or units_per_meter wrong |
| Full thrust on a long straight | 0-100 m/s in ~2.5 s, levels off at `top_speed` | Way off: units_per_meter wrong |
| Steer +1 | Turns toward the craft's right | Flip `desc.right_handed` |
| Drive into a wall at a shallow angle | Scrapes along, keeps most of its speed, `AG_EVENT_WALL_HIT` | Passes through: raycast doesn't hit that geometry, or it's lower than ~2 m |
| Drive over hills and ramps | Follows the ground, flies off crests at speed, lands smoothly | Snaps or jitters: raycast hitting the craft itself or other dynamic objects |
| Leave the route width (route walls on) | Bounces off an invisible wall | Route width or position wrong; debug-draw the route |
| Start a race with AI | All AI complete laps without getting stuck | Debug-draw the route; check it follows the drivable surface and widths fit |
| Same race at 30 and 144 fps | Same lap times | Something is stepping antigrav with a fixed dt or more than once per frame |

Debug drawing that pays off: the route polyline with its width, the four
ground ray hits under each craft, and wall hit events as markers.


## API at a glance

| Call | Purpose |
|---|---|
| `ag_world_desc_default`, `ag_world_create`, `ag_world_destroy` | World lifetime |
| `ag_world_step`, `ag_world_alpha`, `ag_world_poll_event` | Per frame |
| `ag_ship_params_default`, `ag_ship_add`, `ag_ship_remove`, `ag_ship_set_params` | Craft |
| `ag_ship_set_input`, `ag_ship_set_ai`, `ag_ai_params_default` | Control |
| `ag_ship_state`, `ag_ship_get_pose` | Read back |
| `ag_ship_place_on_grid`, `ag_ship_teleport`, `ag_ship_set_locked`, `ag_ship_respawn` | Race flow |
| `ag_ship_apply_impulse`, `ag_ship_apply_spin`, `ag_ship_boost`, `ag_ship_set_thrust_scale` | Weapon and pickup effects |
| `ag_route_resample`, `ag_route_length`, `ag_route_sample` | Route |
| `ag_mesh_create`, `ag_mesh_raycast`, `ag_mesh_destroy` | Optional mesh collision |
