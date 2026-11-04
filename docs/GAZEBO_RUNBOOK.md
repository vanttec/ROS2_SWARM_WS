# Gazebo Runbook — four-drone rehearsal

Fly the whole stack against four simulated X500s before touching real aircraft. Same nodes, same
topics, same commands as the field — only the transport changes.

**Use this when:** validating a change before a field day, onboarding someone to the stack, or
reproducing something that went wrong outdoors. Mistakes here cost nothing.

Field procedure is in [FIELD_RUNBOOK.md](FIELD_RUNBOOK.md); every command is in
[COMMANDS.md](COMMANDS.md). This document is the simulation-specific setup and the test list.

---

## What it proves

| | Why it matters |
| --- | --- |
| Four drones spawn **3 m apart, each with its own EKF origin** | the exact condition the shared frame has to fix |
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

### Where does SITL run?

`arducopter` is a compiled binary and only runs where it was built — the container has glibc 2.35,
the host 2.43, and a host build demands 2.38+.

```bash
~/ardupilot/build/sitl/bin/arducopter --help
```

| Result | Run SITL on | Note |
| --- | --- | --- |
| prints options | **here** | good |
| `GLIBC_2.38 not found` | **the other side** | it was built there |

**Container** is tidier — everything in one place, and MAVProxy works (your MAVProxy packages are
installed for the container's Python 3.10, not the host's 3.14). Rebuilding takes 10–20 minutes:

```bash
cd ~/ardupilot && ./waf distclean && ./waf configure --board sitl && ./waf copter
```

**`distclean` is not optional** when switching sides. Mixing object files from two glibcs gives
`undefined reference to __isoc23_strtoul` at link time.

**Host** needs no rebuild but means `--no-mavproxy`, since MAVProxy cannot run there.

---

## One-time setup

### 1. Build the ArduPilot↔Gazebo bridge — in the container

```bash
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && export GZ_VERSION=harmonic
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j$(nproc)
```

Done when `build/libArduPilotPlugin.so` exists.

### 2. Build the workspace

```bash
cd ~/Documents/VANTTEC/ROS2_SWARM_WS
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables swarm_nodes     # expect 10
```

This installs the worlds and the `x500_v2` model into `share/`, which the next step needs.

---

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
| **T9** | free, for `ros2 topic echo` | container |

Numbers match the field runbook deliberately, so muscle memory transfers. There is no **T2**
(`rtcm_relay`) — SITL reports RTK Fixed without corrections.

---

## Procedure

### Step 1 — Gazebo first

**Gazebo must be running before SITL.** SITL sends to UDP 9002 and needs something listening.

**G:**
```bash
cd ~/Documents/VANTTEC/ROS2_SWARM_WS && source install/setup.bash
./tools/sim_gazebo.sh                 # add --headless if the GUI misbehaves
```

```
world   : .../gazebo/worlds/x500_swarm.sdf
models  : .../gazebo/models
waiting for FDM ports.
FDM ports bound: 9002 9012 9022 9032

Gazebo is ready. Now start SITL.
```

The script exits only when the **FDM ports are actually bound**, so "ready" means ready. If they
never bind it tails the log and names the cause.

> **Do not skip to SITL because the window appeared.** Gazebo opens perfectly happily with zero
> vehicles in it — see [Troubleshooting](#troubleshooting).

### Step 2 — four SITL instances

**S:**
```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --count 4 --auto-sysid -L CMAC --no-mavproxy \
    --add-param-file=$HOME/Documents/VANTTEC/ROS2_SWARM_WS/src/swarm_bringup/config/x500_v2.parm
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

**Go / no-go:** four connected, four rates each.

`num_drones:=1` connects to one instance only — useful for a quick smoke test.

### Step 4 — the shared frame

**T3:**
```bash
ros2 run swarm_nodes odom_relay
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
read near `(0,0,0)` the shift was not applied — that is the bug this whole exercise exists to
catch.

### Step 5 — bridge and dashboard

**T4:** `ros2 run swarm_nodes command_bridge` &nbsp;&nbsp; **T5:** `ros2 run swarm_nodes dashboard`

Banner should read `shared frame resolved | RTK Fixed on all 4 | 0 armed`.
SITL reports fix type 6 without corrections, so the GPS rows go green on their own.

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
- [ ] `ros2 topic hz /drone1/odom` ≈ **25 Hz**, matching the input rate
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

In reverse order. Ctrl-C each terminal, T6 first so the drones land.

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
| **Four drones end to end** | **not yet run** |

The four-drone run is the gap this runbook exists to close. Everything above it has been seen
working; the last line has not.
