# swarm_formation — virtual-leader formation experiment

Four ArduCopters hold a square, a line or a triangle around a virtual leader that you drive from
a panel. It is an experiment on top of the existing stack, and it uses exactly the interface
handed to RSCL: `/droneN/odom` in, `/droneN/cmd_vel` out. Everything else (MAVROS, stream rates,
the shared frame, clamping and the command watchdog) is the stack's, unchanged.

```
formation_panel ──leader_cmd──> formation_leader ──formation_target──> formation_controller ×4
   (PyQt5)        (velocity)     integrates, shapes,                    u = v_d + sat(K e)
                                 checks separation                            │
                                                                     /droneN/cmd_vel
/droneN/odom (odom_relay, swarm_origin) ──────────────────────────────────>   │
                                                       command_bridge (clamp, watchdog)
                                                                              │
                                                  mavros/setpoint_velocity/cmd_vel_unstamped
```

| Node | Does |
| --- | --- |
| `formation_leader` | Integrates the leader's velocity (with speed and acceleration limits), computes each drone's target pose and velocity, morphs between shapes, rejects unsafe changes, keeps the formation inside the fence |
| `formation_controller` | One per drone (`-r __ns:=/droneN`). Position control law below. Commands only when enabled, in GUIDED, armed and above `min_altitude` |
| `formation_panel` | Leader buttons, shape selector, per-drone and ALL buttons (GUIDED, ARM, TAKEOFF, CTRL ON/OFF, LAND), LAND ALL, status |
| `gz_clock` | SITL only: Gazebo Harmonic's clock onto ROS `/clock`, for `use_sim_time` |
| `plot_errors` | `\|e\|` against time from a bag |

## The control law, and why it converges

In GUIDED, ArduCopter closes its own velocity loop, so seen from here each drone is a single
integrator, **ṗ = u**. With the error **e = p_d − p**:

```
u = v_d + v_max · tanh(K‖e‖ / v_max) · e/‖e‖        (horizontal and vertical saturated apart)
```

Take **V = ½ eᵀe**. Since ė = v_d − u:

```
V̇ = eᵀ(v_d − u) = −‖e‖ · v_max · tanh(K‖e‖ / v_max) < 0     for every e ≠ 0
```

so the error converges to zero. Near the target, tanh(x) ≈ x and V̇ ≈ −K‖e‖² = −2K·V, which is
exponential convergence at rate K. Far from it, the correction is capped at v_max, so a large error
never asks for a large speed. The saturation acts on the norm, not per axis, so the command keeps
pointing at the target.

**v_d is the feedforward.** For an integrator plant it plays the role a D term would, so there is
no D term on measured velocity. Without it the drones would trail a moving leader by v_L/K
(0.4 m at 0.3 m/s); with it, measured cruise error is about 1 cm.

**Authority.** The whole command is also clamped to v_max (and `command_bridge` clamps it
again). The proof above holds as long as **‖v_d‖ < v_max**, which leaves the feedback room to
act. v_d is the leader's speed plus ω × r, so the leader limits (0.3 m/s, 0.15 rad/s) are set well
below v_max = 1 m/s: 0.3 + 0.15 × 4.5 m = 0.98 m/s at the ends of a 3 m line turning at full rate.
If you widen the formation or speed up the leader, keep that sum below v_max.

**Integral (optional, off).** `ki > 0` adds `ki·∫e dt`, for a steady wind or a velocity-loop
offset. Anti-windup: it integrates only while the output is not saturated, and its contribution
is capped at `i_max`. In still air with the feedforward, steady-state error is already zero.

**Yaw.** Every drone takes the leader's heading. Yaw rate = ω_L + sat(K_ψ · wrap(ψ_L − ψ)), with
the error wrapped to [−π, π] so a drone never turns the long way round. It goes out in
`angular.z`, which MAVROS sends as `yaw_rate` in SET_POSITION_TARGET_LOCAL_NED. **Verified on
ArduCopter 4.7.2 SITL**: after a 109° leader turn, all four drones' yaw matched the leader's to 0.1°.

## Frames

Everything is in **`swarm_origin`**, the shared ENU frame `odom_relay` publishes `/droneN/odom`
in. It computes each drone's offset from its own EKF origin using GPS (see the top-level README),
so this package needs no per-instance offsets of its own. The controllers ignore odometry in any
other frame (for example `odom_relay` in passthrough mode).

Two consequences found in SITL:

- **Restarting `odom_relay` can move the frame.** By default it adopts drone1's *current* fix as
  the origin. Restarted with drone1 airborne 6 m away, the frame jumped 6.7 m, and before the fix
  below the controllers carried the whole swarm 6.7 m with it. Now a pose jump larger than
  `max_pose_jump` (1 m; a drone cannot move that far between two messages) **disables the
  controller** and asks for a leader reset. Better still, **pin the origin** so a restart
  reproduces the same frame (in the field, the RTK base's surveyed position):
  ```bash
  ros2 run swarm_nodes odom_relay --ros-args -p origin_lat:=<lat> -p origin_lon:=<lon> -p origin_alt:=<alt>
  ```
- **`min_altitude` is z in `swarm_origin`**, which is ground level only if the origin was taken
  on the ground (or pinned there). Start `odom_relay` before takeoff.

## Formations

The leader frame is x forward, y left. **`spacing` s is the closest-neighbour distance in every
shape**, so `min_separation` means the same thing for all of them:

| Shape | Offsets d₁…d₄ |
| --- | --- |
| line | abreast across the heading: y = +1.5s, +0.5s, −0.5s, −1.5s |
| square | side s: front-left, back-left, back-right, front-right |
| triangle | four drones, three corners, so **the fourth sits on the centroid**. Corners at distance s from it (side √3·s) keep the closest pair at s. Two corners forward, one behind |

Targets: **p_i = p_L + R(ψ_L)d_i** and **v_i = v_L + ω_L × (R(ψ_L)d_i) + R(ψ_L)ḋ_i**. The last term
is only non-zero during a shape change, and keeps the feedforward exact then.

**Transitions** interpolate each d_i along a straight line with smoothstep (3u² − 2u³) over
`transition_time`, so they start and end with zero velocity. Because every offset moves on a
straight line over the same interval, each pair's separation is linear in the interpolation
variable and **its minimum is computed exactly**. A change whose shape or path would come closer
than `min_separation` is **rejected** with the reason, and nothing changes. The slot order was
chosen so that every pair of shapes morphs with at least 0.707·s clearance (2.12 m at s = 3 m).

The leader stops one formation radius short of the fence, so the whole formation stays inside it
and no two targets get clamped onto the same wall point. The controller enforces the fence too,
as a backstop.

## ArduPilot specifics, and where each is handled

| Requirement | Where |
| --- | --- |
| One SYSID and one MAVROS per vehicle | Already in `swarm_bringup`: SITL `--auto-sysid` (1–4) on TCP 5760–5790, field sysids 19/17/32/31 over each ESP8266 |
| Stream rates | `stream_setup` (in the MAVROS launch) requests position at 25 Hz via `set_message_interval` |
| GUIDED → arm → takeoff service → wait → velocities | Panel buttons. The controller refuses to enable below `min_altitude`, because a velocity setpoint cancels a GUIDED takeoff in progress |
| Fixed-rate setpoints | Controller at 20 Hz while enabled; `command_bridge` keeps streaming zeros after that |
| Land with LAND, not zero z | LAND / LAND ALL buttons. Entering LAND disables the controller |

## Safety behaviour

| Situation | Response |
| --- | --- |
| Not enabled | No commands. Nothing is enabled, armed, taken off or mode-switched without a button press |
| Leaves GUIDED, or disarms | Controller disables itself; returning to GUIDED does **not** resume it |
| Odometry or target older than `stale_timeout` | Zero velocity |
| Odometry jumps > `max_pose_jump` | Controller disables itself |
| Enable requested more than `max_enable_error` from target | Refused (leader not reset where the swarm is) |
| Leader command stream stops | Leader decelerates to rest after `cmd_timeout` |
| Reset requested with a controller active | Refused (targets would jump) |

## Running it in SITL

Bring up the simulation as in [`docs/GAZEBO_RUNBOOK.md`](../../docs/GAZEBO_RUNBOOK.md) (Gazebo,
four SITL instances, `mavros_sitl_x4.launch.py`), then `odom_relay` **before takeoff** and
`command_bridge`. Then:

```bash
ros2 launch swarm_formation formation.launch.py use_sim_time:=true record:=true
```

`use_sim_time` matters in SITL: four lockstepped copters run slower than real time, and a leader
integrating wall-clock time would run ahead of drones flying in simulated time. Humble's
`ros_gz_bridge` is built for Gazebo Fortress and cannot see Harmonic's topics, so `gz_clock`
relays the clock with Harmonic's own Python bindings. In the field, leave it out
(`use_sim_time` defaults to false).

In the panel:

1. **ALL → GUIDED**, **ALL → ARM** (one confirmation), then **ALL → TAKEOFF** straight away.
   ArduCopter disarms if it sits armed too long without taking off.
2. Wait until all four reach altitude, then **RESET LEADER** (centroid and mean heading of the
   swarm). In SITL the drones spawn 3 m apart abreast, so the default `line` at 3 m matches the
   spawn layout and `|e|` reads about 0.
3. **ALL → CTRL ON**.
4. Pick a shape, **APPLY FORMATION**. **Hold** the arrow and yaw buttons, or the keys (arrows,
   PgUp/PgDn, Q/E, Space to stop), to move the leader. It accelerates gently to 0.3 m/s, so a
   short press moves it only centimetres; the line above the buttons shows where it is. Losing
   window focus stops it.
5. **ALL → LAND** (or **LAND ALL**). The controllers disable themselves on entering LAND.

The bag lands in the current directory; plot it with

```bash
ros2 run swarm_formation plot_errors rosbag2_<date> --out errors.png
```

## Tuning

| Parameter | Raise it and… | Default |
| --- | --- | --- |
| `k` (1/s) | the error dies faster (time constant 1/k), but the outer loop gets closer to ArduCopter's own velocity-loop bandwidth, where the integrator model stops holding. Only 0.8 has been tested; raise in small steps | 0.8 |
| `v_max_xy`, `v_max_z` | large errors close faster; the cap on the whole command. Keep them at or below `command_bridge`'s limits (1.0 / 0.5), which clamp anyway | 1.0 / 0.5 |
| `k_yaw` | heading converges faster; capped by `yaw_rate_max` | 0.8 |
| `ki` | steady wind offsets get removed; too high and it oscillates. If a constant offset appears outdoors, 0.05–0.1 is a cautious (untested) place to start | 0 |
| leader `max_speed_xy`, `max_yaw_rate` | the formation moves faster, but eats the feedback's authority (see above) | 0.3, 0.15 |
| `transition_time` | shape changes are gentler and tracking error during them shrinks | 6 s |

Start with defaults, watch `|e|` in the panel or the bag, and change one thing at a time.

## Measured in SITL (ArduCopter 4.7.2, Gazebo Harmonic, four X500 V2, 3 m spacing)

| Test | Result |
| --- | --- |
| Enable on the ground / before leader reset | refused, with the reason |
| Hand-over after reset, line matches spawn | `\|e\|` 0–2 cm |
| line → square → triangle → line, 6 s each | max `\|e\|` 0.09–0.26 m during, 0–2 cm after; closest approach 2.11 m (predicted 2.12) |
| square at 1.0 m spacing | rejected: drones would come within 0.95 m |
| Leader north at 0.3 m/s | max 0.16 m while accelerating, ~1 cm cruising |
| Leader yaw at 0.15 rad/s, 109° | outer drones max 0.37 m; all four yaws matched the leader's to 0.1° |
| `odom_relay` killed | controllers stream zero velocity, drones hold |
| `odom_relay` restarted, origin not pinned | frame moved 6.66 m → all four controllers disabled |
| `odom_relay` restarted, origin pinned | no jump, controllers kept going, `\|e\|` 1 cm |
| Leader pushed into the east fence | leader stopped at exactly 20.5 m (fence 25 − radius 4.5) |
| drone4 switched to BRAKE, then back to GUIDED | disabled on BRAKE, stayed disabled in GUIDED until re-enabled |
| LAND all four | controllers disabled on entering LAND; all four landed and disarmed |

Not tested yet: the integral term, wind, and anything on real aircraft.
