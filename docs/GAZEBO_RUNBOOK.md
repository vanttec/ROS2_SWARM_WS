# Gazebo Runbook — four-drone rehearsal

Fly the whole stack against four simulated X500s before touching real aircraft. Same nodes, same
topics, same commands as the field — only the transport changes.

For the team formation tutorial, follow steps 1–5, then [Formation and mochila replay](#formation-and-mochila-replay). The manual flight rehearsal uses the same first five steps.

**Use this when:** validating a change before a field day, onboarding someone to the stack, or
reproducing something that went wrong outdoors. Mistakes here cost nothing.

Field procedure is in [FIELD_RUNBOOK.md](FIELD_RUNBOOK.md); every command is in
[COMMANDS.md](COMMANDS.md). This document is the simulation-specific setup and the test list.

---

## What it proves

| | Why it matters |
| --- | --- |
| Four drones spawn **3 m apart** | check that shared-frame odometry preserves their spacing |
| `odom_relay` resolves four offsets | deliverable 4, without flying |
| `first_flight` arms four, and refuses a partial swarm | the sequence that puts aircraft in the air |
| Clamp, watchdog, `sequence_player`, Ctrl-C landing | every safety path, where a mistake is free |
| One bridge serving 4 × 20 Hz | throughput |

---

## Architecture — who runs where

```
CONTAINER (humble)                         HOST or CONTAINER
┌────────────────────────────────┐        ┌──────────────────────┐
│ gz sim    4 × ArduPilotPlugin  │<──UDP──│ sim_vehicle.py       │
│           ports 9002/12/22/32  │  FDM   │   4 × arducopter     │
└────────────────────────────────┘        └──────────┬───────────┘
┌────────────────────────────────┐                   │ MAVLink
│ MAVROS ×4  → odom_relay        │<──TCP 5760/70/80/90┘
│            → command_bridge    │
│            → dashboard         │
└────────────────────────────────┘
```

The container shares the host network namespace, so `127.0.0.1` means the same thing in both and
SITL can sit on either side.

| Instance | sysid | FDM out | FDM in | MAVLink TCP |
| --- | --- | --- | --- | --- |
| `-I0` | 1 | 9002 | 9003 | 5760 |
| `-I1` | 2 | 9012 | 9013 | 5770 |
| `-I2` | 3 | 9022 | 9023 | 5780 |
| `-I3` | 4 | 9032 | 9033 | 5790 |

## Before you start

Use a ROS 2 Humble environment with MAVROS, the MAVROS GeographicLib datasets, Gazebo Harmonic and an ArduPilot checkout with SITL build prerequisites installed. The first GUI run downloads the X500 visual meshes from Gazebo Fuel. The team's container is called `humble`. If using Distrobox, enter it with `distrobox enter humble`.

Open each terminal in this repository. Use the following to return to the workspace root from any directory inside the checkout:

```bash
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
```

Run SITL in the same container for the simplest setup. In the **SITL terminal**, set `ARDUPILOT_DIR` to your own checkout. The default below assumes it is a sibling of this workspace. Set it before changing directory. Build the binary in the environment where it will run.

```bash
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-../ardupilot}"
cd "$ARDUPILOT_DIR"
./waf configure --board sitl && ./waf copter
./build/sitl/bin/arducopter --help
```

For the numbered procedure, start each terminal in the workspace root again. If moving an existing ArduPilot build between host and container causes glibc or linker errors, clean its build products with `./waf distclean` there and repeat the build. Host-side SITL requires shared networking and a host-readable X500 parameter file.

## One-time setup

### 1. Build the ArduPilot↔Gazebo bridge — in the container

```bash
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && export GZ_VERSION=harmonic
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j$(nproc)
```

The launcher expects `build/libArduPilotPlugin.so` under `~/gz_ws/src/ardupilot_gazebo`. This external dependency uses the current user’s home directory. Open a workspace terminal for the next step.

### 2. Build the workspace

```bash
cd "$(git rev-parse --show-toplevel)"
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables swarm_nodes
```

With the default isolated install, the package share directory is
`install/swarm_bringup/share/swarm_bringup/` relative to the workspace root.
In the sourced container shell, check the installed assets that Gazebo will use:

```bash
share_dir="$(ros2 pkg prefix --share swarm_bringup)" && \
  ls "$share_dir/gazebo/worlds/x500_swarm.sdf" "$share_dir/gazebo/models/x500_v2/model.sdf"
```

---

After building, run `source install/setup.bash` from the workspace root in **each new ROS terminal**. Keep each long-running process open.

## Terminal layout

| # | Runs | Where |
| --- | --- | --- |
| **G** | `./tools/sim_gazebo.sh` | container |
| **S** | `sim_vehicle.py` | wherever `arducopter` runs |
| **T1** | `mavros_sitl_x4.launch.py` | container |
| **T3** | `odom_relay` | container |
| **T4** | `command_bridge` | container |
| **T5** | `dashboard` | container |
| **T6** | `first_flight` | container |
| **T7** | `teleop_velocity` / `sequence_player` | container |
| **Formation** | formation launch and panel | container |
| **Bag** | recorded mochila odometry | container |
| **T9** | free, for `ros2 topic echo` | container |

Numbers match the field runbook deliberately, so muscle memory transfers. There is no **T2**
(`rtcm_relay`) — SITL reports RTK Fixed without corrections.

---

## Procedure

### Step 1 — Gazebo first

**Gazebo must be running before SITL.** SITL sends to UDP 9002 and needs something listening.

**G:**
```bash
source install/setup.bash
./tools/sim_gazebo.sh                 # add --headless if the GUI misbehaves
```

```
world   : .../gazebo/worlds/x500_swarm.sdf
models  : .../gazebo/models
waiting for FDM ports.
FDM ports bound: 9002 9012 9022 9032

Gazebo is ready. Now start SITL.
```

The launcher returns when it sees the first matching UDP port. For four vehicles, confirm all four ports above with `ss -lun` before starting SITL. Gazebo keeps running after the launcher returns.

> **Do not skip to SITL because the window appeared.** Gazebo opens perfectly happily with zero
> vehicles in it — see [Troubleshooting](#troubleshooting).

### Step 2 — four SITL instances

**S:** Start in the workspace root. Resolve the parameter file here, then enter your ArduPilot checkout. Set `ARDUPILOT_DIR` if you keep it somewhere other than `../ardupilot`.
```bash
X500_PARAMS="$(realpath -e src/swarm_bringup/config/x500_v2.parm)" && \
cd "${ARDUPILOT_DIR:-../ardupilot}" && \
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --count 4 --auto-sysid -L CMAC --no-mavproxy \
    --add-param-file="$X500_PARAMS"
```

| Flag | Why |
| --- | --- |
| `--count 4 --auto-sysid` | four instances, sysids 1–4 |
| *(no `--auto-offset-line`)* | the **world** carries the 3 m spacing. An offset here shifts each GPS home without moving its model, so the picture and the published positions would disagree |
| `-L CMAC` | where on the globe the simulated field sits. Any name from `Tools/autotest/locations.txt`. Required if you ever add `--auto-offset-line`, which otherwise fails with `ValueError: location needed for auto-offset-line` |
| `--no-mavproxy` | leaves TCP 5760–5790 free for MAVROS |
| `--add-param-file=...x500_v2.parm` | your airframe's tuning |
| `-w` | add this once if you hit `PreArm: Motors: Check frame class and type` |

Each instance prints `Waiting for connection ....` — **that is normal**, they are waiting for
MAVROS. First run after a `distclean` recompiles and takes several minutes.

Confirm from the container:
```bash
ss -ltn | grep -E ':(5760|5770|5780|5790)\b'     # expect four
```

### Step 3 — MAVROS

**T1:**
```bash
ros2 launch swarm_bringup mavros_sitl_x4.launch.py
```

Per drone:
```
[mavros_node-1] [INFO] [drone1.mavros]: CON: Got HEARTBEAT, connected. FCU: ArduPilot
[stream_setup-2] [INFO] [stream_setup]: drone1: message 30 at 25 Hz
... 31, 32, 33
```

**Go / no-go:** four connected and stream requests acknowledged for each (message IDs 24, 30, 31, 32 and 33).

`num_drones:=1` connects to one instance only — useful for a quick smoke test.

### Step 4 — the shared frame

**T3:**
```bash
ros2 run swarm_nodes odom_relay --ros-args -p namespaces:="['drone1','drone2','drone3','drone4']"
```

```
swarm_origin adopted from drone1: -35.3632621, 149.1652374, 603.64 m
drone1: offset +0.00, +0.00, -0.19 m (spread over 25 samples: 0.00, 0.00, 0.00 m)
drone2: offset +0.00, +0.00, ...
drone3: offset +0.00, ...
drone4: offset +0.00, ...
all drones resolved; /droneN/odom now shares one frame
```

**T9 — the check that matters:**
```bash
for n in 1 2 3 4; do echo "drone$n:"; ros2 topic echo /drone$n/odom --once | grep -A3 position; done
```

**Go / no-go:** east ≈ **0, 3, 6, 9 m**, every message `frame_id: swarm_origin`. If all four
read near `(0,0,0)`, check each MAVROS local position and GPS fix before continuing. Small, similar **offsets** are expected when local odometry already includes the spawn displacement. Compare published positions to verify spacing.

### Step 5 — command bridge

**T4:** Run `ros2 run swarm_nodes command_bridge`. Choose one flight procedure below so only one source commands each drone.

## Formation and mochila replay

### Step 6 — launch the experiment and take off

In the **Formation** terminal:

```bash
ros2 launch swarm_formation formation.launch.py use_sim_time:=true record:=false
```

Gazebo supplies simulation time through the launch file’s `gz_clock` node. Set `record:=true` if you want to save the experiment outputs in the launch directory. The panel starts in **MANUAL** with the controllers off.

1. Check that all four columns show a flight mode. Click **ALL → GUIDED** and wait for all four to show **GUIDED**.
2. Click **ALL → ARM** and confirm. Once all four show **ARMED**, click **ALL → TAKEOFF** promptly. If only some arm, inspect each FCU’s message in T1 before retrying. An idle copter can automatically disarm on the ground.
3. Wait for all four to reach the selected altitude (default 3 m). Click **RESET LEADER to swarm**. Wait until the panel shows the leader’s position and heading, then click **ALL → CTRL ON**. If a controller reports “no fresh formation target”, check that the reset succeeded and targets are arriving before retrying. The default **line** at **3 m spacing** matches the spawn layout. Check for four controllers **ON** and numerical position errors.
4. For manual operation, hold the panel arrows to translate or the yaw buttons to turn. Select a shape and click **APPLY FORMATION** to change it.

### Step 7 — follow the recorded mochila

In the **Bag** terminal, from the workspace root:

```bash
bag=rosbags/rosbag_11_nov_2026_mochilartk_3
ros2 bag info "$bag"
ros2 bag play "$bag" --topics /mochila_rtk/odom
```

The example contains 2,806 messages on `/mochila_rtk/odom`. The topic filter leaves the four live simulated drones in control of their own odometry. Keep Gazebo as the only `/clock` source. During playback click **FOLLOW MOCHILA** and check that the panel source changes to **MOCHILA**. The leader follows recorded x, y and yaw while the formation holds its flight altitude.

If follow is refused, read the panel status. Reset the leader first, check that playback is active, and verify that the bag publishes odometry in `swarm_origin` inside the configured fence. When samples stop, the panel can show **MOCHILA (no data, holding)**. At the end of playback, select **MANUAL**. Click **LAND ALL**, wait for landing and disarming, then follow [Shutting down](#shutting-down).

## Alternative — manual flight rehearsal

Use steps 1–5, then open the optional dashboard in **T5** with `ros2 run swarm_nodes dashboard`. Run the following checks with the formation controllers off.

### Step 6 — fly it

**T6:**
```bash
ros2 run swarm_nodes first_flight --ros-args \
    -p confirm:=true -p namespaces:="['drone1','drone2','drone3','drone4']" \
    -p takeoff_altitude:=3.0
```

```
[INFO] [first_flight]: all drones connected
[INFO] [first_flight]: drone1: GUIDED          (then drone2, 3, 4)
[WARN] [first_flight]: ARMING 4 DRONE(S) in 10s -- Ctrl-C to abort
[INFO] [first_flight]: drone1: armed
[INFO] [first_flight]: drone1: taking off to 3.0 m
[INFO] [first_flight]: HANDOVER: the command bridge now has control.
```

Watch the Gazebo window: four copters spin up and climb. **The countdown is your abort window.**

### Step 7 — fly it by hand

**T7:**
```bash
ros2 run swarm_nodes teleop_velocity --ros-args -p drone_id:=drone1
```

```
[INFO] [teleop_velocity]: /drone1/cmd_vel at 20 Hz, 0.5 m/s, 0.5 rad/s
[WARN] [teleop_velocity]: axes are world ENU: w = EAST, a/d = north/south, NOT body-relative.
Requires command_bridge running -- it clamps these speeds and holds position when you stop
pressing keys.
```

| Key | Direction |
| --- | --- |
| `w` / `s` | **east** / west |
| `a` / `d` | south / **north** |
| `q` / `e` | down / up |
| `←` / `→` | yaw left / right |
| `Ctrl-C` | quit — the watchdog then takes over |

Tap `w` once: drone1 drifts **east** and stops when you release. **`w` is east, not forward** —
the axes are world ENU, so it does not matter which way the nose points. Watch it in Gazebo and
confirm it matches `/drone1/odom` climbing in `x`.

Needs `command_bridge` running in T4. Without it nothing reaches the drone — the symptom is keys
that do nothing, and `ros2 topic info /drone1/cmd_vel` showing `Subscription count: 0`.

Change the drone with `-p drone_id:=drone3`, or the speed with `-p speed:=0.3`. The bridge clamps
at 1.0 m/s regardless, so raising `speed` past that does nothing — by design.

### Step 8 — fly a scripted sequence

**T7** (quit the teleop first):
```bash
ros2 run swarm_nodes sequence_player --ros-args -p namespaces:="['drone1']"
```

```
[INFO] [sequence_player]: 6 steps, 85s total, on drone1 at 20 Hz
[INFO] [sequence_player]: step 2/6: 20.0s  x=0.5
[INFO] [sequence_player]: sequence complete: holding zero velocity
```

The default is a 10 m square that ends where it began. **If it does not come back to its starting
point, something is wrong with the frame or the velocity scaling** — that is what the shape is for.

Run it on all four at once with
`-p namespaces:="['drone1','drone2','drone3','drone4']"`.

> **In SITL the square takes longer than 85 s of wall clock.** Four copters in lockstep run at
> about 0.37× real time, so allow roughly four minutes.

---

## Test checklist

Work down it. Each line is one thing that can break in the field.

### Frame and telemetry
- [ ] Four drones read **0 / 3 / 6 / 9 m east**, all `frame_id: swarm_origin`
- [ ] `ros2 topic hz /drone1/odom` matches the local odometry input rate (25 Hz in simulated time, about 6 Hz on the wall clock at 0.24× real time)
- [ ] `odom_relay --ros-args -p publish_global_check:=true` → `/droneN/odom_global` agrees with
      `/droneN/odom` within centimetres (method A vs B)
- [ ] Dashboard shows four live columns, all green

### Command path
- [ ] `ros2 topic pub -r 20 /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}}"` →
      `cmd_vel_unstamped` reads **1.0**, never 5.0
- [ ] `{linear: {x: 3.0, y: 4.0}}` → **(0.6, 0.8)**, not (1.0, 1.0) — direction preserved
- [ ] `{linear: {x: .nan}}` → rejected, log says `command contains NaN or inf, ignored`
- [ ] Teleop `w` moves the drone **east**, not forward
- [ ] `sequence_player` square returns to its start — drift means a frame or scaling problem

### Safety paths — the ones worth rehearsing
- [ ] Ctrl-C the publisher → output goes to **zeros within 0.5 s** and keeps streaming
- [ ] Drone **stays in GUIDED** on the default `hold_mode` (correct, not a fault)
- [ ] With `-p hold_mode:=BRAKE`: mode flips to BRAKE at 2 s, then `send GUIDED to resume`
- [ ] **Ctrl-C `first_flight` → all four land.** The single most important line here
- [ ] `first_flight` without `confirm:=true` → refuses
- [ ] `first_flight` with a drone missing → arms **none** of them
- [ ] Kill `command_bridge` mid-flight → ArduPilot's `GUID_TIMEOUT` holds them after ~3 s

### Multi-drone
- [ ] Four drones through one bridge at 20 Hz, no rate drop
- [ ] `sequence_player` with all four namespaces moves them together
- [ ] Dashboard `LAND ALL` lands all four

---

## Shutting down

After the formation has landed and disarmed, stop bag playback and the formation launch with Ctrl-C. For the manual rehearsal, Ctrl-C T6 first and confirm landing. Then stop the command bridge, relay, MAVROS and SITL before stopping the detached Gazebo process.

```bash
ps -eo pid,args | grep '[g]z sim'      # then kill those PIDs
```

**Do not use `pkill -f "gz sim"`** — the pattern matches its own command line and kills the shell
running it. Same trap with `pkill -f arducopter`.

SITL writes `eeprom.bin`, `mav.tlog`, `logs/` and `terrain/` into whatever directory it was
launched from. They are in `.gitignore`; delete them freely.

---

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| Missing package share or world | Workspace overlay is missing or stale | Source `install/setup.bash` from the workspace root and run the installed asset check in setup |
| Relay waits for drone1 GPS | Relay has no usable fix on its subscribed topic | Check `ros2 topic echo /drone1/mavros/global_position/global --once --qos-reliability best_effort`. If the publisher uses a doubled `mavros/mavros` namespace, rebuild the current bringup and restart its launch |
| State flickers or heartbeats time out | Duplicate MAVROS launches or a slow/stalled sim | Check `pgrep -cx mavros_node` (four for this experiment) and `ros2 topic info /drone1/mavros/state` (one publisher). Inspect `gz topic -e -t /world/x500_swarm/stats -n 1`. At 0.24× real time, a one-second simulated heartbeat takes about four wall seconds |
| TAKEOFF failed | Check mode, arming response and subsequent disarm messages in T1 | Confirm the same drone is GUIDED and armed at takeoff. For `Gyros inconsistent`, rebuild the current bringup and restart Gazebo to load the corrected rotor gain. Keep pre-arm checks enabled |
| **Gazebo opens but is empty; FDM ports never bind; SITL hangs on `Waiting for connection`** | `GZ_SIM_RESOURCE_PATH` is missing our models dir. Log shows `Unable to find uri[model://x500_v2]` then `A model must have at least one link` | use `./tools/sim_gazebo.sh`, which sets it. **Gazebo starts happily with no vehicles — it looks fine** |
| `GLIBC_2.38 not found` running `arducopter` | built on the other side of the container boundary | rebuild there, or run SITL on the side it was built |
| `undefined reference to __isoc23_strtoul` when building | object files from two glibcs mixed | `./waf distclean` then configure and build again |
| SITL stuck on `Waiting for connection ....` | normal with `--no-mavproxy` — it wants MAVROS | start MAVROS, or drop the flag |
| MAVROS `VER: request timeout`, then `FCU don't support AUTOPILOT_VERSION` | two clients on one TCP port — SITL serves exactly one | `--no-mavproxy`, or MAVProxy `output add 127.0.0.1:1455N` plus `use_mavproxy:=true` |
| `PreArm: Motors: Check frame class and type` | `FRAME_CLASS=0` after a wipe | `param set FRAME_CLASS 1` and `reboot`, or restart SITL with `-w` |
| `PreArm: Need Position Estimate` | EKF has not settled | wait for `EKF3 IMU0 origin set` in the MAVROS log |
| All four read `(0,0,0)` despite 3 m spacing | offsets never resolved | check `frame_id`; read `odom_relay`'s log, it says why every 10 s |
| Gazebo GUI will not open under Wayland | Qt/podman | `./tools/sim_gazebo.sh --headless` — physics and the bridge work fine without a window |
| Gazebo dies when the launching shell exits | process group signalled | `setsid` it. The script already does |
| Sim runs in slow motion | lockstep — SITL and Gazebo advance together | expected under load, not a fault |

---

## Status of these tests

Honest record, so nobody mistakes a plan for a result.

| Verified | How |
| --- | --- |
| Gazebo launcher, model path, four FDM ports bound | **29 Oct, this setup** |
| Single drone: MAVROS connect, 25 Hz odom, arm, takeoff, teleop, clamp, watchdog, Ctrl-C land | earlier SITL sessions |
| Shared frame maths | GeographicLib to 3 nm; synthetic four-drone exact |
| Four shared-frame positions | Observed approximately 0, 3, 6 and 9 m east |
| Updated rotor gain | Idle gyro oscillation reproduced and removed in an isolated Gazebo comparison |
| Four-drone takeoff and formation enable, 26 Nov 2026 | All four accepted a 3 m takeoff. Each SITL flight log recorded a peak altitude of 3.32 m. The leader reset at 3.00 m in the shared frame and all four controllers reported enabled. |
| Mochila replay, same session | The player opened `rosbags/rosbag_11_nov_2026_mochilartk_3`. The panel confirmed follow and the leader switched to `MOCHILA`, then reported holding when samples stopped. |
| Landing after this replay | Unverified. The available flight logs end airborne. |
