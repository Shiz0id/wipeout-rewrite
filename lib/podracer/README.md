# podracer

Star Wars Episode I: Racer's pod handling as a standalone C99 library, ported
from the [SW_RACER_RE](https://github.com/tim-tim707/SW_RACER_RE) decompilation
and wrapped in the same host contract as `../antigrav`. One raycast callback
and one route can drive both handling mods, so a Selo game mode can offer
either and let the player pick.

- **One file pair** (`podracer.h`, `podracer.c`), libm only. It includes
  `../antigrav/antigrav.h` for the shared host types (`ag_vec3_t`, `ag_hit_t`,
  `ag_raycast_func_t`, `ag_route_point_t`, `ag_pose_t`); antigrav itself
  doesn't have to be linked
- **Fixed timestep** (60 Hz) with interpolated poses, bit-identical at any
  frame rate
- **The game's numbers**: the thrust integrator and speed curve, coast and air
  brake decay, boost curve and engine heat, traction blend, squared steering
  with 5x counter steer, nose up/down trimming, hover via the fall
  accumulator, landings and gliding, surface tags, per-engine damage and
  steering pull, the crash death rule, AI pacing and autopilot steering, parts
  shop upgrades and stat bars
- **The original 23 pods**: `pr_pod_stats_from_exe` reads the stock handling
  table out of the player's own `swep1rcr.exe`

The host keeps everything else: models, the cables and engine animation,
audio, weapons, UI and cameras.


## Ported vs. reconstructed

The decompilation is incomplete, so every function in `podracer.c` is tagged:

| | |
|---|---|
| **PORTED** (formulas and constants as in the game, with addresses) | `UpdateSpeed`, `ApplyBoost`, `ApplyTraction`, `UpdateHeat`, `UpdatePlayerControl` (steering, pitch trims, throttle, start boost, damage pull), `UpdateSurfaceTag`, `ApplySlopeSteering`, `ApplyGravity`, death speed thresholds, `TakeDamage`, `GetDamagedEngineSides`, `GetEngineDamagePenalty`, `Tilt`, `AI` (pacing, rank random walk, pace-setter and the human tether), `AutopilotSteer` / `ApplyPodProximityForce`, `CalculateUpgradedStat`, `ComputeStatBars`, `Decelerator` |
| **RECONSTRUCTED** (not decompiled yet, rebuilt to behave like the game) | wall collision and scraping, pod vs. pod contact, boost charging (hold full throttle nose down ~1s, then press boost or re-press throttle), repair while driving, lean tightening turns, airborne nose-drop details |
| **Replaced** (the game reads these from its track data, which a host route doesn't have) | AI corner speed planning and curvature feed-forward (the game's AI splines carry speed hints and are themselves the racing line) |

Where a reconstruction makes a call, it's documented at that spot in the
code. As SW_RACER_RE decompiles more (`UpdateControlAndMove`,
`UpdatePhysicsContact`, `CollideTrack`, `BoostCharge`, `Repair`), those
functions can be swapped for ports.


## Units

podracer works in **Racer units** internally: pod speeds run 450–650 and the
stat table is in those units. Set `units_per_racer_unit` to the number of host
units per Racer unit. With the speedometer reading as mph, a Racer unit is
about 0.45 m, so a meters-based host uses `0.45`.

Everything crossing the API in a position or velocity is in **host units**:
raycasts, the route, `pr_pod_state_t::position`/`velocity`, poses, events and
impulses. Speeds and distances that are game stats stay in Racer units:
`speed`, `ground_distance`, `turn_rate` (degrees/s) and engine temperature.

Racer is tuned for its own big courses. Mind these numbers when choosing the
scale:

- **Gravity** is 960 Racer units/s² (`gravityScale` 32, the game's heavy
  "stuck to the track" fall), about 430 m/s² at 0.45. Pods drop fast off
  crests and gaps, as in the game.
- **Turning** tops out at `max_turn_rate * 1.5` degrees/s of yaw (the game
  applies yaw at 1.5x). At top speed that's a radius of about 80 m at 0.45, so
  tight courses need wide tracks or a smaller scale.
- **Ride height** is `hover_height` Racer units (6 by default, 2.7 m), so
  walls should be taller than antigrav's. Racer's own walls are tall and
  invisible.


## Quick start

```c
#include "podracer.h"

static bool raycast(void *user, ag_vec3_t origin, ag_vec3_t dir, float max_dist, ag_hit_t *hit) {
	// Static map geometry only, host units. Map host materials to PR_SURFACE_*
	// flags in hit->surface_flags (see below).
	return host_raycast(user, origin, dir, max_dist, hit);
}

pr_world_desc_t desc;
pr_world_desc_default(&desc, 0.45f);  // host units (meters) per Racer unit
desc.raycast = raycast;
desc.user = host_world;
desc.route = route_points;            // same ag_route_point_t array antigrav takes
desc.route_len = route_count;
desc.route_closed = true;
desc.ai_spread = 6;                   // see "AI" below
pr_world_t *world = pr_world_create(&desc);

// Stock pod stats, from the player's own swep1rcr.exe if it's there
pr_pod_stats_t stats;
if (!pr_pod_stats_from_exe(exe_bytes, exe_size, 0 /* Anakin */, &stats)) {
	pr_pod_stats_default(&stats);
}
pr_pod_stats_apply_upgrade(&stats, PR_UPGRADE_TOP_SPEED, 3, 1.0f); // optional

pr_pod_id_t player = pr_pod_add(world, &stats, spawn_pos, spawn_fwd);
pr_pod_place_on_grid(world, player, 0);
for (int i = 1; i < 8; i++) {
	pr_pod_id_t ai = pr_pod_add(world, &stats, spawn_pos, spawn_fwd);
	pr_ai_params_t params = {.level = 1, .rank_baseline = i + 1, .pace_setter = i == 1};
	pr_pod_set_ai(world, ai, &params);
	pr_pod_place_on_grid(world, ai, i);
}
// Lock everyone for the countdown; unlocking opens the 0.2s start boost window
for (int i = 0; i < 8; i++) pr_pod_set_locked(world, i, true);

// Per frame
pr_input_t in = {
	.steer = stick_x,            // -1..1, + = right
	.pitch = stick_y,            // -1..1, + = nose down (more throttle, charges boost)
	.throttle = trigger,         // 0..1; 1 = full thrust (needed for boosting)
	.brake = brake_button,
	.tilt_left = lb, .tilt_right = rb,
	.slide = slide_button,
	.boost = boost_button,
	.repair = repair_button,
};
pr_pod_set_input(world, player, &in);
pr_world_step(world, frame_dt);

ag_pose_t pose;
pr_pod_get_pose(world, player, &pose);  // interpolated, includes the visual bank
const pr_pod_state_t *s = pr_pod_state(world, player);  // speed, temp, damage, laps...

pr_event_t e;
while (pr_world_poll_event(world, &e)) { /* audio, effects, UI */ }
```


## Surface flags

The game tags track polygons with surface reactions. Map host materials onto
these flags in the raycast's `hit->surface_flags`. They sit at bits 8–19, so
they coexist with antigrav's `AG_SURFACE_*` bits 0–5 and one tagging pass can
serve both mods.

| Flag | Game behavior |
|---|---|
| `PR_SURFACE_FAST` | +200 speed, built up at 25/s (ice paths, speed strips) |
| `PR_SURFACE_SLOW` | Speed x0.75, eased in at 0.5/s (sand, gravel) |
| `PR_SURFACE_SWST` | Sand trap: speed x0.1 and no boost |
| `PR_SURFACE_SLIP` | Grip x0.2 (ice) |
| `PR_SURFACE_LAVA` | Drains engine temperature 20/s until the engines catch fire |
| `PR_SURFACE_FALL` | Respawn on touching it (put it on pit floors under gaps) |
| `PR_SURFACE_MAGNET` | Gravity and the pod's frame follow the surface (wall rides, tubes) |
| `PR_SURFACE_ZON` / `ZOFF` | Enter or leave zero-g, where pitch input pitches the pod |
| `PR_SURFACE_SIDE` | Banked side: AI leans into it |
| `PR_SURFACE_WALL` | Treat as wall even if it isn't steep |

Anything steeper than about 60° from the pod's up is a wall automatically.


## Handling notes for players

These are the game's mechanics, so experienced Racer players will expect
them:

- Full throttle (`throttle >= 0.99`) is a distinct state: analog throttle
  below it caps thrust. **Boost** needs full throttle: hold it with the nose
  down for about a second until `PR_EVENT_BOOST_READY`, then press boost or
  release and re-press the throttle within 0.3 s. Boosting drains engine
  temperature at `heat_rate`; at 0 a random engine part catches fire. Releasing
  the throttle or touching the air brake ends the boost.
- **Start boost**: press the throttle within 0.2 s of unlocking.
- **Nose down** (`pitch > 0`) adds throttle and trims turning; **nose up**
  turns harder and glides in the air.
- **Damage**: wall hits damage the engines on that side, scaled by
  `damage_immunity`. A part above 0.8 pulls the steering, and a burning or
  destroyed part caps throttle at half and disables tilt and boost. Hold
  repair to fix it while driving, which costs speed.
- **Death**: losing more than 140 speed in a hit at more than 325 explodes a
  pod going 200+ (respawn after 2–3 s). Glancing hits up to about 30° are
  survivable at top speed; 45° and up aren't. AI pods respawn instead of
  exploding.


## AI

AI pods run the same physics, steering on the route with the game's
autopilot and pacing on the game's rubber banding:

- The **pace-setter** (one per race, the track favorite) runs 1.06x while
  leading, 1.4x when the leading human is far ahead, and 1.1x otherwise.
- **Everyone else** holds a station behind the pace-setter by assigned rank.
  The rank wanders ±2 around `rank_baseline` every 8–18 s, and pace is clamped
  to 0.5–1.6x. Each rank's station is `50 * ai_spread` Racer units behind the
  one ahead. The game uses 20–40 on tracks of about 30k units a lap, so scale
  it to your lap length (the 3 km test circuit is about 6.7k units and uses 6).
- The two AI nearest the leading human pace on their gap to that human, so
  there's always someone to race.
- `ai_level` is the game's track level times the AI speed setting (0.9 /
  1.0 / 1.1); the game's per-track levels run 0.85–1.13.
- `human_catchup` is the splitscreen catch-up (up to 1.25x for trailing
  humans).


## Tests

`make test`, or `cmake -S . -B build && cmake --build build && ctest --test-dir build`.

The headless suite reuses antigrav's test geometry and BVH. It checks the
speed curve against the game's formula, turning rates and counter steer,
boost, heat, overheating and repair, start boost, each surface, cliff jumps
and gliding, wall angles (survive, explode, never tunnel), engine damage pull,
upgrades and bars, the exe loader on a synthetic PE, magnet wall rides, AI
races (solo, an 8-pod pack, and the game's own spread) and frame-rate
determinism. It's clean under ASan/UBSan.


## Integration checklist

1. Vendor `lib/podracer` next to `lib/antigrav`, which supplies the header
   types.
2. Pick `units_per_racer_unit` (0.45 for meters) and set `gravity_dir` and
   `right_handed` to match the engine.
3. Reuse the antigrav raycast. Add the `PR_SURFACE_*` mapping, at least
   `FALL` on pit floors and `WALL` where walls are low or shallow.
4. Reuse the antigrav route: center line, up vectors, width.
5. Load stats: `pr_pod_stats_from_exe` with the player's `swep1rcr.exe` if
   they have it (pods 0–22 in game order, `pr_pod_name`), otherwise
   `pr_pod_stats_default` and your own values. Apply upgrades once on load.
6. Each frame: set inputs, `pr_world_step`, `pr_pod_get_pose` for rendering,
   and poll events.
7. Verify in this order: the pod hovers at `hover_height` on flat ground; full
   throttle reaches about 95% of `max_speed` in 20 s; full right steering
   turns right; walls stop it without tunneling; AI laps the course without
   respawning.
