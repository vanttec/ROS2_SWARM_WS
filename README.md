# ROS2_SWARM_WS

Ground station for the **SWARM – RSCL outdoor formation experiment**: four Holybro X500 V2
quadcopters flown as a swarm, with RTK-grade pose streamed into ROS 2.

RSCL owns the leader–follower formation controller, which already works in simulation and indoors
under motion capture. This workspace is the **outdoor replacement for the MOCAP room**. The target
interface is deliberately small — the formation controller should only ever see
`/droneN/odom` (`nav_msgs/Odometry`) coming in and `/droneN/cmd_vel` (`geometry_msgs/Twist`) going
out, with MAVLink, WiFi, RTK and origin math hidden inside this stack.

The full specification is in [`RCSL Collab.pdf`](RCSL%20Collab.pdf) in the repo root.

**Flying it:** [`docs/FIELD_RUNBOOK.md`](docs/FIELD_RUNBOOK.md) is the procedure for a test day —
bring-up order, go/no-go at each stage, abort paths, and a troubleshooting table of failures we
have actually hit. [`docs/GAZEBO_RUNBOOK.md`](docs/GAZEBO_RUNBOOK.md) rehearses the same stack
against four simulated drones first, where mistakes are free.
[`docs/COMMANDS.md`](docs/COMMANDS.md) is the reference for every command, with what each one is
for.

The command path, which everything that moves a drone goes through:

```
teleop_velocity ─┐
                 ├─> /droneN/cmd_vel ─> command_bridge ─> /droneN/mavros/setpoint_velocity/
RSCL controller ─┘                      clamp + watchdog      cmd_vel_unstamped

first_flight:  GUIDED → arm → takeoff → hand over → land   (Ctrl-C lands)
```

## Hardware

Four X500 V2 airframes with Pixhawk 6C running ArduCopter, each carrying an F9P RTK receiver and an
ESP8266 WiFi bridge. Ground station is Ubuntu 22.04 running ROS 2 Humble.

| Namespace | Name    | sysid | ESP8266 IP     | Ground-station port |
| --------- | ------- | ----- | -------------- | ------------------- |
| `drone1`  | PAPOI   | 19    | 192.168.50.11  | 14551               |
| `drone2`  | TRINEO  | 17    | 192.168.50.12  | 14552               |
| `drone3`  | BISUU   | 32    | 192.168.50.13  | 14553               |
| `drone4`  | LUCAS   | 31    | 192.168.50.14  | 14554               |

All four join the router `vanttec_drones_swarm` on static IPs assigned by MAC address, kept below
`192.168.50.100` so they stay out of the DHCP range. The ESP8266 is wired to TELEM2 at 500000 baud
and listens on port 14555.

## Packages and files

### `src/swarm_bringup` — launch and configuration

- **`launch/mavros_x4.launch.py`** — brings up the whole link layer. Includes MAVROS's `node.launch`
  four times, one instance per drone, each with its own namespace, an `fcu_url` of
  `udp://:<port>@<esp_ip>:14555`, and `tgt_system` set to that drone's sysid so the four links don't
  mix. Each instance gets the shared plugin allowlist plus MAVROS's stock `apm_config.yaml`. It then
  starts `stream_setup` with the four namespaces at 25 Hz.
  The `DRONES` list at the top of the file is the single place to edit when a drone's IP or sysid
  changes.

- **`launch/mavros_sitl_x4.launch.py`** — the same four MAVROS instances pointed at ArduPilot SITL
  on localhost instead of the ESP8266 bridges, using the sysids SITL assigns with `--auto-sysid`
  (1–4) rather than the aircraft's own. Connects to SITL's TCP ports 5760–5790 by default; pass
  `use_mavproxy:=true` to use UDP 14551–14554 instead, when SITL is running with MAVProxy
  forwarding. See [Simulation](#simulation-ardupilot-sitl--gazebo).

- **`config/swarm_pluginlists.yaml`** — MAVROS plugin allowlist. It denies `*` and then allows only
  the eleven plugins the platform actually uses: `sys_status`, `sys_time`, `command`, `imu`,
  `local_position`, `global_position`, `setpoint_velocity`, `setpoint_raw`, `param`, `gps_rtk` and
  `gps_status`. This cuts each drone from several hundred topics down to roughly fifty, which
  matters because every one of them crosses a 2.4 GHz ESP8266 telemetry link. The `/**:` wildcard
  applies it to every MAVROS node.

- **`gazebo/models/x500_v2/`** — the Holybro X500 V2 airframe for Gazebo, and
  **`gazebo/worlds/`** the generated worlds that bind one copter per SITL instance.
  **`gazebo/tools/gen_x500_world.py`** writes those worlds. See
  [Simulation](#simulation-ardupilot-sitl--gazebo).

- **`config/x500_v2.parm`** — the ArduCopter side of the same airframe: frame class, hover thrust,
  the speed limits `command_bridge` clamps to, and the iris leftovers switched off. Pass it to
  `sim_vehicle.py` with `--add-param-file`.

- **`setup.py`** — installs `launch/`, `config/` and `gazebo/` into `share/` so `FindPackageShare` can resolve
  them at launch time.

- **`package.xml`** — depends on `launch`, `launch_ros` and `mavros`; exec-depends on `swarm_nodes`.

### `src/swarm_nodes` — runtime nodes

- **`swarm_nodes/stream_setup.py`** → `ros2 run swarm_nodes stream_setup`
  Requests MAVLink message rates from every drone through its
  `/droneN/mavros/set_message_interval` service: message ids 30 `ATTITUDE`,
  31 `ATTITUDE_QUATERNION`, 32 `LOCAL_POSITION_NED` and 33 `GLOBAL_POSITION_INT`, all at 25 Hz.
  It watches each `/droneN/mavros/state`, requests the rates when a drone connects, re-requests them
  on reconnect, and reapplies them every 30 s — so a flight-controller reboot or a ground control
  station overriding stream rates cannot silently drop the pose rate.
  Parameters: `namespaces`, `message_ids`, `rate_hz`, `reapply_period`.
  Started automatically by the launch file.

- **`swarm_nodes/rtcm_relay.py`** → `ros2 run swarm_nodes rtcm_relay`
  Reads RTCM3 corrections from the local RTK base and fans them out to every drone's
  `/droneN/mavros/gps_rtk/send_rtcm`. **This is what turns 1–3 m GPS into roughly 1 cm** — without
  it the drones sit at fix type 3 however good the receivers are, and "RTK-grade pose" is not true.
  MAVROS fragments each message over the existing telemetry link, so no extra radio is needed.
  Parameters: `namespaces`, `port` (`/dev/ttyACM0`), `baudrate`, `udp_port` (`0`),
  `report_period`, `reconnect_period`.

  Corrections come from either the base's **serial port**, or over **UDP** from QGroundControl's
  *NTRIP/RTK → UDP Forwarding*. Use UDP when QGC is doing the survey-in: a serial port has only
  one owner, so QGC and this node cannot both read the base.

  It parses whole RTCM3 frames and checks each CRC rather than forwarding raw bytes, so corrupt
  data is dropped instead of being pushed to four aircraft. The periodic report lists which RTCM
  message numbers are arriving, which is how you confirm the base is configured: **1005 or 1006 is
  the base position — if it never appears, the base has not finished surveying in.**

- **`swarm_nodes/link_monitor.py`** → `ros2 run swarm_nodes link_monitor`
  Subscribes to `/droneN/mavros/imu/data` on all four drones, counts messages and tracks the largest
  gap between consecutive ones, then logs a single line per drone every 5 s and resets the counters:

  ```
  PAPOI:  24.8 Hz, max gap  112 ms | TRINEO:  25.0 Hz, max gap   48 ms | ...
  ```

  Use it to judge WiFi link health before and during a flight.

- **`swarm_nodes/orientation_monitor.py`** → `ros2 run swarm_nodes orientation_monitor`
  Subscribes to `/droneN/mavros/imu/data` and `/droneN/mavros/global_position/compass_hdg`. It
  converts the IMU quaternion to roll/pitch/yaw, turns the ENU yaw into a compass heading with
  `(90 - yaw) % 360`, and prints roll, pitch, heading from the IMU and heading from the compass for
  all four drones once a second.
  Use it to confirm the four agree and to catch a badly calibrated compass: RTK fixes position but
  not heading, so compass error turns directly into formation yaw error.

- **`swarm_nodes/odom_relay.py`** → `ros2 run swarm_nodes odom_relay`
  Subscribes `/droneN/mavros/local_position/odom` on all four drones and republishes each one on
  **`/droneN/odom`** — the topic the formation controller subscribes to — rewriting
  `child_frame_id` to `droneN/base_link`.
  Parameters: `namespaces`, `max_rate_hz` (default `0.0`, meaning no limit).

  Every message is forwarded as it arrives, so **the output rate equals the input rate**. The
  autopilot sets that rate and `stream_setup` requests 25 Hz; this node must not resample it. An
  earlier version latched the newest pose and published on a 25 Hz timer, which aliased against the
  25 Hz input and silently lost ~24% of messages (measured 19 Hz out for 25 Hz in). `max_rate_hz`
  is an optional ceiling for when MAVLink is deliberately streaming faster than `/droneN/odom`
  needs; it bounds rather than targets, landing at or just below the cap.

  **It also computes the shared frame.** Each drone's EKF measures from its own private origin —
  wherever that vehicle first got a good fix — so four drones report four incompatible frames and a
  formation controller cannot tell where they are relative to each other. ArduPilot will not let us
  force a common EKF origin while using GPS, so the shift is computed here, once per drone:

  ```
  offset = geodetic_to_enu(global_fix, swarm_origin) − local_position
  ```

  The drone's absolute position expressed in the shared frame, minus where its own EKF thinks it
  is, is the constant gap between the two origins. Adding it to every later pose puts all four in
  one frame, stamped `frame_id: swarm_origin`.

  It needs only `global_position/global` and `local_position/odom`, both already streaming at
  25 Hz, so there is no extra MAVLink traffic. Offsets are averaged over `origin_samples` messages,
  which means **the drones must be stationary while it initialises** — it warns if the samples
  spread more than a metre.

  Extra parameters: `shared_origin` (`true`), `reference_drone` (`drone1`), `origin_samples` (25),
  `origin_lat` / `origin_lon` / `origin_alt` (unset), `publish_global_check` (`false`),
  `frame_id` (`swarm_origin`).

  By default `swarm_origin` is the reference drone's first fix. Set `origin_lat`/`lon`/`alt` to pin
  it somewhere fixed instead — the RTK base's surveyed position, so the frame is identical across
  sessions. With `shared_origin:=false` it reverts to raw passthrough. **While the offsets are
  unresolved it publishes nothing**: silence is safer than poses labelled `swarm_origin` that are
  not in it.

  `publish_global_check:=true` adds `/droneN/odom_global`, computed straight from the GPS fix
  instead of local position + offset. That is the spec's method B; the two should agree within
  centimetres, and a growing gap means EKF drift.

  The geodetic conversion is verified against GeographicLib's `CartConvert` — the same library
  MAVROS links against — agreeing to 3 nanometres.

- **`swarm_nodes/command_bridge.py`** → `ros2 run swarm_nodes command_bridge`
  The safety layer for everything that commands a drone. Subscribes `/droneN/cmd_vel`
  (`geometry_msgs/Twist`, world ENU) and forwards it to
  `/droneN/mavros/setpoint_velocity/cmd_vel_unstamped` at a steady 20 Hz.
  Parameters: `namespaces`, `stream_hz` (20), `stale_timeout` (0.5), `max_xy_speed` (1.0),
  `max_z_speed` (0.5), `max_yaw_rate` (0.5), `hold_mode` (**empty**), `hold_timeout` (2.0).

  It rejects NaN/inf, clamps speed, and watchdogs the command stream:

  | Since last command | Action |
  | ------------------ | ------ |
  | never commanded    | sends nothing, so it cannot fight the mission node during takeoff |
  | < 0.5 s            | forwards the clamped command |
  | 0.5 – 2.0 s        | sends **zero velocity** — holds position |
  | > 2.0 s            | *only if* `hold_mode` is set: switch to it once, then stop streaming |

  **Mode switching is off by default**, and that is deliberate. ArduPilot already has a backstop:
  after `GUID_TIMEOUT` seconds (default 3.0) with no setpoint it zeroes its own target and holds
  position *autonomously* — no ground station, no pilot. Our 0.5 s zero-velocity does the same
  thing 2.5 s sooner, which at 1 m/s saves 2.5 m of uncommanded travel. That is the real value.

  Taking the drone **out** of GUIDED is not an improvement on that, and an earlier version of this
  node did it anyway, which was a mistake. Set `hold_mode` only if you specifically want a
  deliberate re-entry to GUIDED before anything can command the drone again:

  | `hold_mode` | Behaviour |
  | ----------- | --------- |
  | *(empty, default)* | never change mode; hold in GUIDED, ArduPilot backstops |
  | `BRAKE` | holds position and altitude by itself, ignores the sticks — **use this one** |
  | `LOITER` | takes climb rate from the pilot's throttle; with nobody on the sticks it descends and lands |

  After a switch the bridge stays silent until the drone is back in `GUIDED`. The horizontal clamp scales the (x, y) vector by its magnitude rather than clamping
  each axis — clamping separately would rotate the command, turning (3, 4) into (1, 1) and pointing
  45° off course.

  Note a timer is right here but wrong in `odom_relay`: commands must be synthesized when input
  stops (ArduPilot halts a vehicle after 3 s without a setpoint), telemetry never should be.

- **`swarm_nodes/first_flight.py`** → `ros2 run swarm_nodes first_flight`
  Owns the flight sequence: **GUIDED → arm → takeoff → hand over → land**, so the bridge never has
  to know about modes or arming.
  Parameters: `namespaces`, `takeoff_altitude` (2.0), `confirm` (**false**), `arm_delay` (5.0),
  `connect_timeout` (30.0).

  ```bash
  ros2 run swarm_nodes first_flight --ros-args \
      -p confirm:=true -p namespaces:="['drone1']" -p takeoff_altitude:=2.0
  ```

  It refuses to arm without `confirm:=true`, counts down `arm_delay` seconds first, and will not arm
  a partial swarm. **Ctrl-C lands every drone** — it installs its own signal handler so the ROS
  context survives long enough to issue the land command, which rclpy's default handler does not
  allow.

- **`swarm_nodes/sequence_player.py`** → `ros2 run swarm_nodes sequence_player`
  Plays a scripted Twist sequence to `/droneN/cmd_vel` — the acceptance test for the velocity
  interface, since deliverable 5 is done when each drone "follows a scripted Twist sequence in SITL
  and outdoors". It goes through `command_bridge`, so it is clamped and watchdogged like anything
  else.
  Parameters: `namespaces`, `rate_hz` (20), `sequence_file` (built-in default otherwise).

  The default is a 10 m square at 0.5 m/s returning to its start, which makes drift obvious — if it
  does not come back, something is wrong with the frame or the velocity scaling. A sequence file is
  a YAML list of steps:

  ```yaml
  - {duration: 4.0, x: 0.5}    # 2 m east
  - {duration: 4.0, y: 0.5}    # 2 m north
  - {duration: 2.0}            # hold
  ```

  `x`/`y`/`z` are m/s in world ENU, `yaw` is rad/s, omitted axes are zero. Ctrl-C stops and holds
  zero velocity; it does not land.

- **`swarm_nodes/dashboard.py`** → `ros2 run swarm_nodes dashboard`
  Qt ground-station window showing all four drones side by side: link rate, mode, armed, battery,
  position in the shared frame, and age of the last command — colour-coded so a problem is visible
  without reading numbers. A banner states whether the four are actually in one comparable frame.

  Per-drone buttons for GUIDED / ARM / TAKEOFF / LAND, and a large **LAND ALL**. ARM and TAKEOFF
  ask for confirmation with Cancel as the default button; LAND and GUIDED do not, because both are
  recoveries. LAND ALL has no dialog on purpose — in an emergency you should not have to dismiss
  one. For a planned launch use `first_flight`, which has the countdown; these buttons are for
  recovering a single drone mid-session.

  ROS spins in a background thread, so a stalled link never freezes the window.

- **`swarm_nodes/teleop_velocity.py`** → `ros2 run swarm_nodes teleop_velocity`
  Keyboard velocity teleop for a single drone, for bench and SITL testing. Publishes a `Twist` to
  **`/<drone_id>/cmd_vel`** — the same topic the formation controller uses — while a key is held:
  `w`/`s` for ±x, `a`/`d` for ∓y, `q`/`e` for ∓z, left/right arrows for yaw rate.
  Parameters: `drone_id` (**required**, e.g. `drone1`), `speed` (0.5 m/s), `yaw_rate` (0.5 rad/s),
  `rate_hz` (20), `hold_timeout` (0.5 s).

  ```bash
  ros2 run swarm_nodes teleop_velocity --ros-args -p drone_id:=drone1
  ```

  **Requires `command_bridge` running** — the bridge is what clamps these speeds and holds position
  when you stop pressing keys, so without it nothing reaches the drone. **The axes are world ENU,
  not body**: `w` means *east*, not *forward*, no matter where the drone is pointing.
  Run it in SITL first, and on a real aircraft only with an RC safety pilot and the geofence on.

- **`setup.py` / `package.xml`** — console-script entry points and dependencies (`rclpy`,
  `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `mavros_msgs`).

All three nodes subscribe with `qos_profile_sensor_data` (best-effort), matching the QoS that MAVROS
publishes its sensor topics with.

## Simulation (ArduPilot SITL + Gazebo)

The four aircraft can be replaced by ArduPilot SITL flying a Gazebo model of the **Holybro X500
V2**, so the simulated airframe is the one on the bench rather than the stock Iris. SITL is
useful here because each copter can be given **its own EKF origin** — the same frame mismatch you
get outdoors, reproducible on a laptop without flying.

Gazebo runs in the `humble` container (that's where `gz` and the headers are); ArduPilot SITL runs
on the host (its binary is built against the host's glibc). They reach each other over localhost
because the container shares the host network namespace.

### The X500 V2 model

`gazebo/models/x500_v2/` is the airframe, and `config/x500_v2.parm` is the matching ArduCopter
side. Against the stock `gazebo-iris`:

|                | iris (stock) | `x500_v2`      |
| -------------- | ------------ | -------------- |
| wheelbase      | 450 mm       | **500 mm**     |
| all-up weight  | 1.75 kg      | **2.02 kg**    |
| inertia (x/z)  | 0.008 / 0.017 | **0.022 / 0.040** kg m² |
| props          | 2-blade, 0.2 m | **1045, 0.254 m** |
| thrust/weight  | ~3.5         | **2.2**        |

Only mass, inertia and thrust reach the flight dynamics, so those are what the model gets right;
collisions and inertias are primitives sized to them. The **visuals** are meshes from PX4's
`x500_base` (frame, motors, props), fetched from Gazebo Fuel (version 1, CC0) and cached in
`~/.gz/fuel` the first time a world loads, so **the first start needs internet** (27 MB). That
frame is NXP's HoverGames X500-class kit rather than Holybro's own CAD, but the same 500 mm carbon
X layout. Visuals never change how the copter flies: the hover numbers below are identical with
the primitive model and the meshes.

The meshes cost GPU, not physics. Measured with four copters and the GUI open: 0.339x real time
with primitives, 0.332x with meshes rendered on the discrete GPU. On a laptop in NVIDIA "on-demand"
PRIME mode Gazebo renders on the integrated GPU by default, where the GUI turns choppy; render on
the discrete one with `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` in front of
`./tools/sim_gazebo.sh`, or use `--headless`.

Flown in SITL, the model arms, climbs to 5 m, and holds it at **44% throttle with under 0.2° of
tilt** — matching the `MOT_THST_HOVER 0.45` in the param file, which is the check that the thrust
and the weight agree.

The world is generated, not hand-written, because each copter needs its own ArduPilotPlugin on its
own FDM port and SDF cannot patch a value inside an `<include>`:

```bash
cd src/swarm_bringup/gazebo/tools
./gen_x500_world.py                      # worlds/x500_swarm.sdf, four copters 3 m apart
./gen_x500_world.py --count 1 --out ../worlds/x500_single.sdf
./gen_x500_world.py --first-instance 10  # ports 9102.., 5860.., to run beside a live sim
```

Edit `gen_x500_world.py` and regenerate; do not edit the `.sdf`.

### Running it

| Instance | sysid | FDM (SITL↔gz) | MAVLink TCP |
| -------- | ----- | ------------- | ----------- |
| `-I0`    | 1     | 9002 / 9003   | 5760        |
| `-I1`    | 2     | 9012 / 9013   | 5770        |
| `-I2`    | 3     | 9022 / 9023   | 5780        |
| `-I3`    | 4     | 9032 / 9033   | 5790        |

One-time setup of the ArduPilot↔Gazebo bridge plugin, **inside the container**:

```bash
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && export GZ_VERSION=harmonic
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j$(nproc)
```

Then, in the container, point Gazebo at both the plugin and this repo's model and start the world.
Verify one vehicle works before trying four:

```bash
cd <workspace root>/src/swarm_bringup/gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$PWD/models
gz sim -r worlds/x500_single.sdf      # then worlds/x500_swarm.sdf for all four
```

On the host, with the workspace sourced:

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --count 4 --auto-sysid -L CMAC --no-mavproxy \
    --add-param-file=$(ros2 pkg prefix swarm_bringup)/share/swarm_bringup/config/x500_v2.parm
```

`-f gazebo-iris` is still the right frame: it is what marks the sim as externally driven, and the
only iris-specific parameters it loads (IRLock, a simulated analog rangefinder) are switched back
off in `x500_v2.parm`.

And in the container, in place of `mavros_x4.launch.py`:

```bash
ros2 launch swarm_bringup mavros_sitl_x4.launch.py
```

Everything downstream — `odom_relay`, `link_monitor`, `orientation_monitor`, `teleop_velocity` —
works unchanged against the simulator, because the namespaces and topic names are identical.

### Two things that will confuse you once

**Nothing moves until every link is up.** The plugin runs `lock_step`, and all four copters live in
one Gazebo world, so the world cannot step until all four SITL instances are being driven — and a
SITL instance does not start its loop until a GCS attaches to its MAVLink port. Start Gazebo, start
SITL, then launch MAVROS; the scene is frozen in between, which is not a fault. Expect roughly
**0.37× real time** with four copters on one machine (measured), so a 25 Hz stream arrives at
closer to 9 Hz on the wall clock. Judge rates against `link_monitor` in SITL only relatively.

**Spacing lives in the world, not in `--auto-offset-line`.** SITL anchors each instance's EKF
origin to its `--home`, and `--auto-offset-line` shifts that home *without moving the model in
Gazebo*. Use both and the two disagree: measured with a 1.5 m world plus `90,1.5`, Gazebo showed
the copters 1.5 m apart while `/droneN/odom` reported 3 m — both "correct", since the published
value is the true geodetic position, but the picture no longer matches the field.

So the world carries the full 3 m and no offset is passed. Gazebo and `/droneN/odom` then agree,
and the layout matches what you tape out on the ground.

The offset-cancellation maths was proven separately on 30 Oct with a 1.5 m world plus `90,1.5`:
offsets resolved to `+0.01 / +1.51 / +3.01 / +4.51` and published positions came out
`0 / 2.999 / 5.999 / 8.998 m` — within 2 mm across four drones.

Each drone sits at a different, non-zero offset from its own origin, which is the condition the
shared-frame maths needs to be tested against. Do not read absolute formation distances off the
simulated map.

## Build and run

Prerequisites: Ubuntu 22.04 with ROS 2 Humble, plus
`sudo apt install ros-humble-mavros ros-humble-mavros-extras` and the MAVROS GeographicLib datasets
script.

```bash
cd <workspace root>
colcon build --symlink-install
source install/setup.bash

# 4x MAVROS + stream_setup
ros2 launch swarm_bringup mavros_x4.launch.py

# in other terminals
ros2 run swarm_nodes link_monitor
ros2 run swarm_nodes orientation_monitor
```
