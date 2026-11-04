# Command reference

Every command used to build, simulate, fly and debug this stack, with what each one is for. For the
*order* to run things in on a test day, see [`FIELD_RUNBOOK.md`](FIELD_RUNBOOK.md).

**Everything runs inside the container.** The Ubuntu 26 host has no ROS 2.

---

## Environment

| Command | Purpose |
| --- | --- |
| `distrobox enter humble` | Enter the ROS 2 Humble container. Needed before anything else |
| `distrobox list` | Check the container exists and is up |
| `source /opt/ros/humble/setup.bash` | Make `ros2` available |
| `source ~/Documents/VANTTEC/ROS2_SWARM_WS/install/setup.bash` | Make *our* nodes available (also sources ROS) |
| `distrobox enter humble -- bash -lc '<cmd>'` | Run one command in the container without staying in it |

---

## Build

| Command | Purpose |
| --- | --- |
| `colcon build --symlink-install` | Build both packages. `--symlink-install` means Python edits take effect without rebuilding |
| `colcon build --symlink-install --packages-select swarm_nodes` | Rebuild one package — faster while iterating |
| `ros2 pkg executables swarm_nodes` | List runnable nodes. **The check that catches a broken `setup.py`** — expect 10 |
| `rm -rf build install log && colcon build --symlink-install` | Nuclear rebuild when the build state looks wrong |

### ArduPilot SITL (one-time, in the container)

| Command | Purpose |
| --- | --- |
| `pip3 install --user "empy==3.3.4"` | Build dependency. 4.x breaks ArduPilot |
| `cd ~/ardupilot && ./waf configure --board sitl` | Configure for simulation |
| `./waf copter` | Build `arducopter` |
| `./waf distclean` | **Wipe the build tree.** Required when switching between host and container — mixing their object files gives `undefined reference to __isoc23_strtoul` |
| `./build/sitl/bin/arducopter --help` | Confirm the binary runs here. `GLIBC not found` means it was built for the other environment |
| `Tools/environment_install/install-prereqs-ubuntu.sh -y` | Install everything ArduPilot needs, if `configure` complains |

---

## Simulation

Start **Gazebo first** — SITL sends to UDP 9002 and needs something listening.

| Command | Purpose |
| --- | --- |
| `gz sim -v4 -r <world>.sdf` | Start Gazebo with a world. `-r` runs immediately (otherwise press ▶) |
| `gz sim -s -v4 -r <world>.sdf` | Headless — physics only, no window. Use if the GUI misbehaves under Wayland |
| `$(ros2 pkg prefix swarm_bringup)/share/swarm_bringup/gazebo/worlds/x500_swarm.sdf` | Four X500s, 3 m apart, FDM ports 9002/9012/9022/9032 |
| `.../worlds/x500_single.sdf` | One X500 — use this first |
| `src/swarm_bringup/gazebo/tools/gen_x500_world.py --count 4 --spacing 3` | Regenerate a world. Edit this, not the generated `.sdf` |

### SITL

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --count 4 --auto-sysid -L CMAC --no-mavproxy \
    --add-param-file=$(ros2 pkg prefix swarm_bringup)/share/swarm_bringup/config/x500_v2.parm
```

| Flag | Purpose |
| --- | --- |
| `-f gazebo-iris --model JSON` | Take physics from Gazebo instead of SITL's internal model |
| `--count 4 --auto-sysid` | Four instances, sysids 1–4 |
| *(no `--auto-offset-line`)* | the world carries the spacing. An offset moves the GPS home but not the model, so the two disagree |
| `-L CMAC` | Where on the globe the simulated field sits. Required if you add `--auto-offset-line`, which otherwise fails with `ValueError: location needed for auto-offset-line` |
| `--no-mavproxy` | Don't start MAVProxy, leaving TCP 5760 free for MAVROS |
| `--console --map` | MAVProxy console and moving map (omit `--no-mavproxy`) |
| `--add-param-file=...x500_v2.parm` | Load the X500 V2 tuning |
| `-w` | Wipe parameters to frame defaults. Fixes `FRAME_CLASS = 0` |
| `-L CMAC` | Spawn at a named location from `Tools/autotest/locations.txt` |
| `-I0` | Run a single specific instance |

`sim_vehicle.py` rebuilds on launch, so the first run after a `distclean` takes several minutes.

### MAVProxy console (inside its own prompt)

| Command | Purpose |
| --- | --- |
| `mode guided` | Switch flight mode |
| `arm throttle` | Arm |
| `takeoff 5` | Climb to 5 m (must be armed and in GUIDED) |
| `param show FRAME_CLASS` / `param set FRAME_CLASS 1` | Read / write a parameter |
| `reboot` | Restart the autopilot — needed after `FRAME_CLASS` |
| `output add 127.0.0.1:14551` | **Fan MAVLink out to MAVROS.** How MAVROS and MAVProxy coexist, since SITL's TCP 5760 serves only one client |

---

## Bring-up

| Command | Purpose |
| --- | --- |
| `ros2 launch swarm_bringup mavros_x4.launch.py` | **Real aircraft.** 4× MAVROS over the ESP8266 bridges, sysids 19/17/32/31, plus `stream_setup` |
| `ros2 launch swarm_bringup mavros_sitl_x4.launch.py` | **Simulation.** Same four, pointed at SITL on localhost, sysids 1–4 |
| `... mavros_sitl_x4.launch.py num_drones:=1` | Connect to one SITL instance — the single-vehicle smoke test |
| `... mavros_sitl_x4.launch.py use_mavproxy:=true` | Use UDP 14551–14554 instead of TCP 5760–5790, when MAVProxy is running |
| `ros2 launch swarm_bringup <file> --show-args` | List a launch file's arguments without starting it |

---

## Our nodes

| Command | Purpose |
| --- | --- |
| `ros2 run swarm_nodes stream_setup` | Ask each drone for MAVLink messages 30/31/32/33 at 25 Hz, and re-ask on reconnect. Started by the launch files |
| `ros2 run swarm_nodes odom_relay` | **Publish `/droneN/odom` in the shared `swarm_origin` frame.** Drones must be stationary while offsets resolve |
| `ros2 run swarm_nodes command_bridge` | **The safety layer.** Clamps every `/droneN/cmd_vel` and watchdogs the stream. Nothing reaches a drone without it |
| `ros2 run swarm_nodes first_flight --ros-args -p confirm:=true -p namespaces:="['drone1']"` | GUIDED → arm → takeoff → hand over → land. **Ctrl-C lands.** Won't arm without `confirm:=true` |
| `ros2 run swarm_nodes teleop_velocity --ros-args -p drone_id:=drone1` | Keyboard flying. `w/s` east/west, `a/d` south/north, `q/e` down/up, arrows yaw. Needs `command_bridge` |
| `ros2 run swarm_nodes sequence_player --ros-args -p namespaces:="['drone1']"` | Fly a scripted Twist sequence. Default is a 10 m square that returns to its start |
| `ros2 run swarm_nodes dashboard` | **All four drones on one screen** plus controls and LAND ALL |
| `./tools/find_rtk_base.py` | **Find the base's serial port and say what it is emitting.** Run this first — it tells you whether the base is already surveyed in (RTCM3 with message 1005) or still needs configuring (UBX/NMEA). Needs no ROS and no built workspace |
| `ros2 run swarm_nodes rtcm_relay --ros-args -p port:=/dev/ttyACM0` | **Feed RTK corrections** from the base's serial port to all four drones. Without corrections they stay at fix type 3 |
| `ros2 run swarm_nodes rtcm_relay --ros-args -p udp_port:=13320` | Same, but take corrections over UDP from QGroundControl's *NTRIP/RTK → UDP Forwarding*. Use this when QGC is doing the survey-in, since a serial port has only one owner |
| `ros2 run swarm_nodes link_monitor` | Per-drone message rate and **max gap** — how you judge the WiFi link |
| `ros2 run swarm_nodes orientation_monitor` | Roll/pitch/heading for all four, IMU vs compass |

Useful parameters:

```bash
# tighter speed limits
ros2 run swarm_nodes command_bridge --ros-args -p max_xy_speed:=0.5

# opt in to taking the drone out of GUIDED when commands stop (off by default -- ArduPilot's own
# GUID_TIMEOUT already holds position autonomously, so this only adds a re-handover barrier)
ros2 run swarm_nodes command_bridge --ros-args -p hold_mode:=BRAKE

# pin the shared frame to the RTK base's surveyed position
ros2 run swarm_nodes odom_relay --ros-args \
    -p origin_lat:=25.6515 -p origin_lon:=-100.2895 -p origin_alt:=540.0

# publish the method-B cross-check alongside, for comparing against /droneN/odom
ros2 run swarm_nodes odom_relay --ros-args -p publish_global_check:=true
```

---

## Inspecting a running system

| Command | Purpose |
| --- | --- |
| `ros2 topic list` | Everything being published |
| `ros2 topic echo /drone1/odom --once` | One message. **`--once` matters** — otherwise it streams forever |
| `ros2 topic echo /drone1/mavros/state --field mode` | Watch just one field live, e.g. the flight mode changing |
| `ros2 topic hz /drone1/odom` | Measure rate. Reports `average rate`, `min`, `max`, `std dev` — **`max` is the gap that matters** |
| `ros2 topic info /drone1/cmd_vel` | Type, publisher and subscriber counts. `Subscription count: 1` proves the bridge is listening |
| `ros2 topic pub -r 20 /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"` | Send a velocity by hand |
| `ros2 topic echo /drone1/cmd_vel geometry_msgs/msg/Twist` | Echo a topic **before** anything publishes, by naming the type |
| `ros2 node list` / `ros2 service list` | What's running / what can be called |
| `ros2 interface show mavros_msgs/srv/CommandTOL` | A message or service's fields |
| `ros2 param list /drone1/mavros` | Parameters on a node |

### Checking RTK — the Friday question

```bash
ros2 topic echo /drone1/mavros/gpsstatus/gps1/raw --once | grep -E "fix_type|satellites|h_acc"
```

| `fix_type` | Meaning |
| --- | --- |
| 3 | plain 3D GPS, 1–3 m — **not good enough** |
| 5 | RTK Float — corrections arriving, not yet converged |
| **6** | **RTK Fixed, ~1 cm — what you need** |

`h_acc` is horizontal accuracy in **millimetres**. The dashboard shows all of this per drone, with
a banner that says whether *all four* are RTK Fixed.

---

## Commanding a drone directly (no ROS node)

For recovery or bench testing. `first_flight` and the dashboard are the normal routes.

```bash
ros2 service call /drone1/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
ros2 service call /drone1/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /drone1/mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 2.0}"
ros2 service call /drone1/mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"
```

Spin one motor for 3 s at 10% — **props off**, disarmed, safety switch off. `param1` is the motor
number (1–4). Verifies motor order and direction:

```bash
ros2 service call /drone1/mavros/cmd/command mavros_msgs/srv/CommandLong \
  "{broadcast: false, command: 209, confirmation: 0,
    param1: 1.0, param2: 0.0, param3: 10.0, param4: 3.0,
    param5: 0.0, param6: 0.0, param7: 0.0}"
```

A `CommandBool` reply of `success=False, result=4` means **pre-arm checks refused** — the reason is
printed in the MAVROS terminal as `FCU: PreArm: <reason>`, not returned by the service.

---

## Diagnosing when something won't connect

| Command | Purpose |
| --- | --- |
| `ss -ltn \| grep 5760` | Is SITL listening? Ports are 5760 + 10×instance |
| `ss -lun \| grep 9002` | Is Gazebo bound to the FDM port? Nothing here means SITL has no physics |
| `ss -tnp \| grep 5760` | **Who is connected.** Two clients means MAVROS and MAVProxy are fighting — only one is served |
| `pgrep -af "gz sim\|arducopter\|mavros_node"` | What's running |
| `ps -eo args --no-headers \| grep <name>` | Process list without `pgrep` matching its own command line |
| `ip route \| grep default` | Router address, for the drone-network admin page |
| `sudo arp-scan --localnet` | Find a drone's MAC/IP on the network |
| `stty sane` | Repair a terminal left in raw mode |
| `ls /dev/ttyACM* /dev/ttyUSB*` | Which serial devices exist. Run before and after plugging the base in |
| `udevadm info -q property -n /dev/ttyACM0 \| grep ID_VENDOR` | Confirm a port really is the u-blox |
| `id -nG \| grep dialout` | **Check you can open serial ports.** Nothing printed means *Permission denied* on the RTK base. `newgrp` does not work in distrobox — `exit`, `distrobox stop humble`, then re-enter. In a hurry: `sudo chmod a+rw /dev/ttyACM0` |

---

## Git

| Command | Purpose |
| --- | --- |
| `git status --short` | What changed |
| `git add -A src README.md docs` | Stage source and docs |
| `git commit -m "..."` | Commit |
| `git push -u origin <branch>` | Push a new branch |
| `gh pr create --base main --fill` | Open a pull request |
| `git rm -r --cached build install log` | **Untrack build artifacts** without deleting them from disk |
| `git check-ignore -v --no-index <path>` | Why a path is (or isn't) ignored. `--no-index` tests the pattern even for tracked files |
| `git log --oneline -10` | Recent history |
| `git diff --stat main...origin/<branch>` | What a teammate's branch changes |

Remember `.gitignore` never applies to files git already tracks — that's why untracking needs
`git rm --cached`.
