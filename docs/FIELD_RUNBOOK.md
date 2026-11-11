# Field Runbook — SWARM outdoor test

Procedure for a flight test with the X500 V2 swarm. Every stage ends in a **go / no-go**: if it
fails, stop there rather than pressing on. Node descriptions are in the [README](../README.md),
every command is in [COMMANDS.md](COMMANDS.md) — this document is the order to do them in and what
you should see.

**Read [Emergencies](#emergencies) before the first flight of the day, not during one.**

Every command runs inside the container:

```bash
distrobox enter humble
source ~/Documents/VANTTEC/ROS2_SWARM_WS/install/setup.bash
```

---

## Terminal layout

Nine terminals, plus **T0** on a test that includes the leader. Label them — in the field you
will be reaching for the right one under pressure.

| # | Runs | Stays open | Purpose |
| --- | --- | --- | --- |
| **T0** | `mochila.launch.py` | all day, leader tests only | the backpack's MAVROS + `stream_setup_mochila` + `nodo_mochila` |
| **T1** | `mavros_x4.launch.py` | all day | the four links + `stream_setup` |
| **T2** | `rtcm_relay` | all day | RTK corrections to every drone — and the backpack, it's in the node's default namespace list |
| **T3** | `odom_relay` | all day | `/droneN/odom` in the shared frame — `/mochila_rtk/odom` too, same default |
| **T4** | `command_bridge` | all day | clamp + watchdog. **Nothing moves without it** |
| **T5** | `dashboard` | all day | the screen you actually watch — the backpack is a monitor-only column, no flight buttons |
| **T6** | `first_flight` | per flight | arm / takeoff / land. **Ctrl-C here lands** |
| **T7** | `teleop_velocity` or `sequence_player` | per test | flying it |
| **T8** | `link_monitor` / `orientation_monitor` | as needed | diagnostics — both already cover the backpack |
| **T9** | free | — | `ros2 topic echo`, one-off service calls |

T0–T5 start once and are left alone. T6 is the one you reach for in a hurry. Skip T0 entirely on
a drones-only session — nothing else here depends on the leader being present.

---

## Before you leave the lab

| | Check |
| --- | --- |
| ☐ | `colcon build --symlink-install` clean; `ros2 pkg executables swarm_nodes` lists **10** |
| ☐ | `id -nG \| grep dialout` returns something, or the RTK base will not open. See [Serial permissions](#serial-permissions) — `newgrp` does **not** work inside the container |
| ☐ | Work committed and pushed — a known-good state to return to |
| ☐ | Batteries charged, props inspected, airframe bolts checked |
| ☐ | **Failsafes verified on every drone** — see the table below. This is not optional |
| ☐ | Tape measure, cones or chalk for four marks, first-aid kit |
| ☐ | One RC transmitter per drone, charged, bound, mode switch reachable |

### Failsafes — check these on all four, in Mission Planner or QGC

These are what save an aircraft when the software above fails. Verify, do not assume.

| Parameter | Set to | Why |
| --- | --- | --- |
| `FENCE_ENABLE` | `1` | the hard boundary. Without it nothing stops a flyaway |
| `FENCE_RADIUS` | your field, e.g. `50` m | breach triggers the action below |
| `FENCE_ALT_MAX` | e.g. `15` m | stops a runaway climb |
| `FENCE_ACTION` | `1` (RTL) | come home on breach |
| `RTL_ALT` | above the tallest obstacle | the height it returns at |
| `BATT_FS_LOW_ACT` | `2` (RTL) | come home on low battery |
| `BATT_FS_CRIT_ACT` | `1` (Land) | land now on critical |
| `FS_THR_ENABLE` | `1` | RC link lost → RTL |
| `FS_GCS_ENABLE` | `1` | **MAVLink lost → RTL.** Your 500 ms link gaps are far inside the 5 s default, so this will not false-trigger |
| `GUID_TIMEOUT` | `3` (default) | the backstop the whole watchdog design leans on |

`FS_GCS_ENABLE` is the one people skip. It is what brings the drone home if your **laptop dies**,
rather than leaving it hovering until the battery runs out.

## On site

- Ground station laptop, charged, plus the router
- RTK base on a tripod with clear sky. Run `./tools/find_rtk_base.py` to find its port and confirm
  it is emitting RTCM3 — **message 1005 is the proof survey-in finished**
- **One RC safety pilot per aircraft in the air.** Not one pilot for four drones
- Brief every pilot before anything spins — see [Pilot briefing](#pilot-briefing)
- Clear area; nobody downwind, nobody under the flight line

### Serial permissions

`rtcm_relay` cannot open `/dev/ttyACM0` unless your shell is in the `dialout` group. Check:

```bash
id -nG | grep dialout
```

Nothing printed means it will fail with **`Permission denied`** the moment you plug the base in.

The confusing part: you are probably a member already. These disagree —

```
getent group dialout  ->  dialout:x:20:ricardo07     you ARE in the group
id -nG                ->  ricardo07 nogroup          this SHELL is not
```

The membership was added after this container session started, so the session never picked it up.
**`newgrp dialout` does not fix it inside distrobox** — it prompts for a password and fails.

**Fix — restart the container:**

```bash
exit                      # leave the container
distrobox stop humble     # confirm when asked
distrobox enter humble
id -nG | grep dialout     # should print now
```

**If you are already in the field and cannot restart**, open the device directly:

```bash
sudo chmod a+rw /dev/ttyACM0
```

Blunt, and it resets when the base is unplugged — but it gets you flying. Do the proper fix later.

---

# Stage 0 — Ground station and links

**T1:**
```bash
ros2 launch swarm_bringup mavros_x4.launch.py
```

Per drone, expect:
```
[mavros_node-1] [INFO] [drone1.mavros]: CON: Got HEARTBEAT, connected. FCU: ArduPilot
[stream_setup-2] [INFO] [stream_setup]: drone1: connected, message rates requested
[stream_setup-2] [INFO] [stream_setup]: drone1: message 30 at 25 Hz
[stream_setup-2] [INFO] [stream_setup]: drone1: message 31 at 25 Hz
[stream_setup-2] [INFO] [stream_setup]: drone1: message 32 at 25 Hz
[stream_setup-2] [INFO] [stream_setup]: drone1: message 33 at 25 Hz
```

**Go / no-go:** all four connected, all four reporting four rates each. A missing drone is a
network or ESP8266 problem — fix it before anything spins.

> Use `mavros_x4.launch.py`, **not** `mavros_sitl_x4.launch.py`. The real one carries the aircraft
> sysids (19 / 17 / 32 / 31) and the ESP8266 addresses.

**Bad signs:**

| Message | Meaning |
| --- | --- |
| `drone3: rate request for message 32 failed` | that drone did not accept the rate — its pose will be slow |
| `drone2: connection lost, will re-request on reconnect` | link dropping |
| nothing at all from one namespace | check power, WiFi, and the router's DHCP table |

---

# Stage 1 — Link health, props OFF

**T8:**
```bash
ros2 run swarm_nodes link_monitor
```

Let it run **five minutes with all four powered** — contention between the four radios is the
thing you are measuring. Output every 5 s:

```
PAPOI:  24.8 Hz, max gap  112 ms | TRINEO:  25.0 Hz, max gap   48 ms | BISUU: ... | LUCAS: ...
```

**Go / no-go** on the `max gap` column:

| max gap | Verdict |
| --- | --- |
| < 0.5 s | Good. Formation flight is viable |
| 0.5 – 2 s | Usable, expect jerky formation |
| **> 3 s** | **Stop.** ArduPilot halts a vehicle after 3 s without a setpoint. Fix the radio: a quieter 2.4 GHz channel, or trim stream rates |

*Measured in the lab on 29 Oct: 500 ms max, all four. That is comfortably inside the band.*

### Clamp check, still props off

**T4:**
```bash
ros2 run swarm_nodes command_bridge
```
```
[INFO] [command_bridge]: bridging 4 drones at 20 Hz | limits 1.0 m/s xy, 0.5 m/s z,
0.5 rad/s yaw | zero velocity after 0.5s, then hold in GUIDED (ArduPilot's GUID_TIMEOUT
is the backstop)
```

**T9:**
```bash
ros2 topic pub -r 20 /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}}"
```
**T8:**
```bash
ros2 topic echo /drone1/mavros/setpoint_velocity/cmd_vel_unstamped --once
```
Must read `x: 1.0`, never 5.0. Then try `{linear: {x: 3.0, y: 4.0}}` → must come out
`(0.6, 0.8)`, not `(1.0, 1.0)` — that proves the clamp preserves direction.

Ctrl-C the publisher. Output drops to zeros within half a second and **keeps streaming them**.
With the default `hold_mode` the drone stays in GUIDED — correct, not a fault.

---

# Stage 2 — RTK, props OFF

## 2a. Survey the base in

**Do this first and do it early** — everything downstream inherits it, and it takes minutes
during which nobody can fly.

**Survey-in** is the base teaching itself where it is. A GPS receiver on its own is accurate to
1–3 m, and that error wanders. So the base sits perfectly still and averages its own fixes for
several minutes. The wander averages out, the estimate converges, and once it is good enough the
base **locks that one position and never recomputes it**. From then on it compares every live
satellite measurement against what it *should* see from that locked spot, and the difference is
the correction it broadcasts.

The base must be **stationary from the first second to the last**. Bump the tripod halfway through
and the average is poisoned — restart it.

### Physical setup

1. Tripod on firm ground, antenna with the **widest possible view of the sky**. Not against a
    building, not under a tree. Multipath (signal bouncing off a wall) is the main thing that
    ruins a survey-in.
2. Antenna on its **ground plane** (the metal disc), level.
3. Somewhere it will not be kicked, and **outside the flight area**.
4. USB from the base to the ground-station laptop.
5. Note roughly where it is — if you ever want `origin_lat/lon/alt` to pin the frame across
    sessions, this is the spot.

### Starting it, in QGroundControl

QGC drives the survey-in; our nodes only consume the result.

1. **Application Settings → General → RTK GPS**
2. Set the two fields:

     | Field | Use | Why |
     | --- | --- | --- |
     | Survey-in accuracy | **2.0 m** | how tightly the base must know itself before it locks |
     | Minimum observation time | **180 s** | the floor, regardless of accuracy |

3. Connect the base. QGC starts the survey automatically and shows a progress bar with a live
    accuracy figure, which falls as it converges.
4. Wait. It finishes when **both** conditions are met.

> **2 m sounds terrible. It is not, for us.** That number is how well the base knows its own
> absolute position on Earth. Every drone is corrected against the *same* base, so that error is
> identical for all four and cancels out between them. Relative positions — the only thing a
> formation cares about — stay centimetre-accurate. Asking for 0.5 m accuracy instead can take
> 20+ minutes and buys us nothing.

### Confirm it actually finished

QGC's progress bar is one signal. The proof is **RTCM message 1005**, which *is* the base
position — a base that has not finished surveying has no position to send:

```bash
./tools/find_rtk_base.py
```

Look for `1005` in the message types it reports. If it is there, you are done; move on.

If QGC holds the serial port, close QGC's link first, or skip straight to `rtcm_relay` on UDP
(below) and read the message types from its report instead.

### If it will not converge

| Symptom | Cause |
| --- | --- |
| Accuracy stalls around 3–4 m | poor sky view, or multipath off a nearby wall. Move the tripod |
| Restarts from zero | the base was bumped, or power browned out |
| Only UBX/NMEA, never RTCM3 | base is still in rover mode — it was never told to be a base |

---

## 2b. Fan the corrections out to the drones

**T2:**
```bash
./tools/find_rtk_base.py                                    # which port, and is it configured
ros2 run swarm_nodes rtcm_relay --ros-args -p port:=/dev/ttyACM0
# or, if QGroundControl holds the serial port and is forwarding:
ros2 run swarm_nodes rtcm_relay --ros-args -p udp_port:=13320
```

Every 10 s:
```
[INFO] [rtcm_relay]: 94 frames from /dev/ttyACM0, 12.4 kB to 4 drones | types: 1005, 1077, 1087, 1097, 1230
```

| What you see | Meaning |
| --- | --- |
| types include **1005** or **1006** | **survey-in finished.** The base knows where it is |
| types but no 1005 | still surveying. Wait, then look again |
| `no base-position message (1005/1006) yet` | same — the node is telling you explicitly |
| `no RTCM frames from ... -- check it is powered` | wrong port, not powered, or not emitting RTCM3 |
| `cannot open /dev/ttyACM0 ... is the RTK base plugged in?` | port or permissions. `newgrp dialout` |
| `... unparsable data ... is it set to emit RTCM3?` | base is in rover mode, sending UBX or NMEA |
| rising `| N rejected` | corrupt frames — bad cable or a flaky link. Investigate |

### Confirm every drone reached RTK Fixed

Watch the **T5 dashboard** banner. You want:
```
shared frame resolved    |    RTK Fixed on all 4    |    0 armed
```
Not `RTK on 2/4 — NOT all centimetre`.

Per drone the `gps` row should read `RTK Fixed 12` in green and `gps acc` about `0.02 m`.
Or check directly in **T9**:
```bash
for n in 1 2 3 4; do ros2 topic echo /drone$n/mavros/gpsstatus/gps1/raw --once | grep -E "fix_type|h_acc"; done
```
`fix_type: 6` = RTK Fixed. `5` = Float, still converging — give it a minute. `3` = no corrections.

**Go / no-go: all four at fix type 6.** Everything downstream inherits this accuracy.

---

## 2c. The leader (mochila), if this test uses one

The backpack is a Pixhawk + RTK rover carried on foot, not a flying vehicle — there is no arm,
takeoff or land for it anywhere in this stack. What you are confirming here is the same thing as
for a drone: the link is up, and corrections have converged to RTK Fixed.

**T0**, if not already running:
```bash
ros2 launch swarm_bringup mochila.launch.py
```

Expect, in order:
```
[mavros_node-1] [INFO] [mochila_rtk.mavros]: CON: Got HEARTBEAT, connected. FCU: ArduPilot
[nodo_mochila-3] [INFO] [nodo_mochila]: Mochila: FCU conectado.
[nodo_mochila-3] [INFO] [nodo_mochila]: Mochila: primera pose recibida.
[nodo_mochila-3] [INFO] [nodo_mochila]: Mochila: primer fix GPS recibido.
```

Then every `report_period` (10 s by default):
```
[nodo_mochila-3] [INFO] [nodo_mochila]: Mochila: RTK Fixed, 22 satélites -- RTK OK.
```

| What you see | Meaning |
| --- | --- |
| `FCU no conectado` | check the backpack's USB/telemetry link and that T0 is actually running |
| `conectada pero sin pose ni GPS todavía` | connected, but MAVROS has not published either yet — give it a moment |
| `sin GPS_RAW_INT (24) todavía` | `stream_setup_mochila` has not reached it yet, or hasn't requested the rate — check T0's log |
| `RTK Float` / `3D` (not Fixed) | same as a drone: check **T2** is running and that `mochila_rtk` is in its namespace list (it is, by default) |

**Go / no-go: `RTK OK` from `nodo_mochila`.** Same bar as the drones — no formation test should
start on a leader that is not RTK Fixed.

Cross-check against the rest of the stack, same data from two other angles:

- **T5 dashboard** — the `MOCHILA` column shows `gps` / `gps acc` directly, and `east` / `north` /
  `up` once `odom_relay` resolves it into the shared frame (same `/mochila_rtk/odom` as the drones'
  `/droneN/odom` — directly comparable once both show `swarm_origin`). It has no buttons: the
  column under it just reads `monitor only`.
- **T8 `orientation_monitor`** — add `MOCHILA`'s line to the drones' when checking that all
  headings agree (Compass check, Stage 3).

---

# Stage 3 — The shared frame, props OFF

**This proves deliverable 4, and nothing leaves the ground.**

Place the four on marks — a straight line 3 m apart is easiest to measure. **Write down the real
tape numbers.** Then, with the drones **completely still**:

**T3:**
```bash
ros2 run swarm_nodes odom_relay
```

```
[INFO] [odom_relay]: swarm_origin will be drone1's first fix; keep the drones STILL while offsets settle
[INFO] [odom_relay]: swarm_origin adopted from drone1: 25.6515000, -100.2895000, 540.00 m
[INFO] [odom_relay]: drone1: first odometry message received
[INFO] [odom_relay]: drone1: offset +0.00, +0.00, -0.19 m (spread over 25 samples: 0.01, 0.01, 0.02 m)
[INFO] [odom_relay]: drone2: offset +2.99, +0.02, -0.18 m (spread over 25 samples: 0.01, 0.02, 0.02 m)
...
[INFO] [odom_relay]: all drones resolved; /droneN/odom now shares one frame
```

The **spread** is your quality gauge. Under ~5 cm means a clean fix and still drones.

**T9 — the measurement:**
```bash
for n in 1 2 3 4; do echo "drone$n:"; ros2 topic echo /drone$n/odom --once | grep -A3 position; done
```

**Go / no-go:**

| Check | Pass |
| --- | --- |
| Positions match your tape | **within 5 cm** |
| Every message | `frame_id: swarm_origin` |
| Offset spread | under ~5 cm |

If two drones both read near `(0,0,0)` while metres apart, the shift was not applied — stop.

**Messages that mean wait, not broken:**

| Message | Do |
| --- | --- |
| `still no swarm_origin: waiting for a GPS fix from drone1` | drone1 has no fix yet |
| `offsets unresolved for drone3 (no GPS fix yet: drone3)` | that drone is not ready |
| `drone3: GPS reports no fix, ignoring position` | correct behaviour — it is refusing bad data |
| `offset samples spread over 1.4 m -- was the drone moving?` | **restart `odom_relay`** with everything still |

> `odom_relay` **publishes nothing until offsets resolve.** Silence is deliberate — it beats poses
> labelled `swarm_origin` that are not in it. Read its log rather than assuming it crashed.

To pin the frame across sessions, use the base's surveyed position:
```bash
ros2 run swarm_nodes odom_relay --ros-args -p origin_lat:=X -p origin_lon:=Y -p origin_alt:=Z
```

### Compass check while you are out there

**T8:**
```bash
ros2 run swarm_nodes orientation_monitor
```
Point all four the same way — headings should agree within a few degrees. **RTK fixes position,
not heading**, so a bad compass becomes formation yaw error that nothing else catches.

---

# Stage 4 — First flight, ONE drone, props ON

**Everything from here moves aircraft. Brief the pilots first.**

Confirm T1–T5 are all up and the dashboard is green. Then:

**T6:**
```bash
ros2 run swarm_nodes first_flight --ros-args \
    -p confirm:=true -p namespaces:="['drone1']" -p takeoff_altitude:=1.0
```

```
[INFO] [first_flight]: waiting for 1 drone(s) to connect...
[INFO] [first_flight]: all drones connected
[INFO] [first_flight]: drone1: GUIDED
[WARN] [first_flight]: ARMING 1 DRONE(S) in 10s -- Ctrl-C to abort
[WARN] [first_flight]:   10...
...
[INFO] [first_flight]: drone1: armed
[INFO] [first_flight]: drone1: taking off to 1.0 m
[INFO] [first_flight]: HANDOVER: the command bridge now has control. Publish to /droneN/cmd_vel. Ctrl-C here to land.
```

**The countdown is your abort window. Ctrl-C during it and nothing arms.**

| Message | Meaning |
| --- | --- |
| `refusing to arm without confirm:=true` | you forgot the flag. Good |
| `not connected: drone2 -- refusing to arm` | it will not arm a partial swarm |
| `drone1: ARMING REFUSED (check pre-arm messages)` | read `FCU: PreArm:` in **T1** |
| `drone1: takeoff refused` | usually not in GUIDED, or no position estimate |

Then, in order, stopping if anything misbehaves:

**1 — one tap of a key.** T7:
```bash
ros2 run swarm_nodes teleop_velocity --ros-args -p drone_id:=drone1
```
Tap `w` once. It moves **east** — not forward — and stops when you release.

**2 — short continuous moves.** Hold a key briefly. It must never exceed walking pace.

**3 — the watchdog.** Ctrl-C the teleop. The drone **stops and holds**, staying in GUIDED.

**4 — the scripted sequence.** T7:
```bash
ros2 run swarm_nodes sequence_player --ros-args -p namespaces:="['drone1']"
```
```
[INFO] [sequence_player]: 6 steps, 85s total, on drone1 at 20 Hz -- requires command_bridge and an airborne drone
[INFO] [sequence_player]: step 2/6: 20.0s  x=0.5
...
[INFO] [sequence_player]: sequence complete: holding zero velocity
```
The square should return it to its start. **Drift means a frame or scaling problem** — note how far.

**5 — the landing.** Ctrl-C **T6**:
```
[WARN] [first_flight]: landing 1 drone(s)
[INFO] [first_flight]: drone1: landing
```

**Go / no-go: all five clean.** Step 5 especially — that is the abort path for every later stage.

---

# Stage 5 — The swarm

Add **one drone at a time**, a safety pilot each, real spacing.

**T6:**
```bash
ros2 run swarm_nodes first_flight --ros-args \
    -p confirm:=true -p namespaces:="['drone1','drone2']" -p takeoff_altitude:=1.0
```

Repeat Stage 4's five checks at each size before adding another. Only when all four hold station
and the dashboard reads `shared frame resolved | RTK Fixed on all 4` should the formation
controller be connected to `/droneN/odom` and `/droneN/cmd_vel`.

---

# Emergencies

## The hierarchy — memorise this order

| # | Situation | Action | Who |
| --- | --- | --- | --- |
| 1 | One drone misbehaving | `LAND` button on the dashboard | operator |
| 2 | Lost confidence in the software | **LAND ALL** on the dashboard, or **Ctrl-C T6** | operator |
| 3 | Lost confidence in anything | **PILOTS TAKE MANUAL CONTROL** | pilots |
| 4 | Drone heading for people or leaving the field | **RTL on the RC**, or manual flight back | pilot |
| 5 | About to hit a person and nothing else has worked | **emergency disarm** — see below | pilot |

**The RC pilot always outranks the ground station.** Nothing in this stack can stop a pilot taking
control, and that is deliberate. A pilot who thinks something is wrong should act and explain
afterwards — never wait for permission from whoever is on the laptop.

## "The drone will not come down"

Work down this list. Do not spend more than a few seconds on each:

1. **Dashboard `LAND`** for that drone
2. **Ctrl-C in T6** — lands every armed drone
3. **T9, direct:**
   ```bash
   ros2 service call /drone1/mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"
   ```
4. **Pilot: mode switch to `LAND`** on the RC
5. **Pilot: switch to `STABILIZE` or `ALT_HOLD` and fly it down manually**
6. **Emergency disarm** — last resort, below

If the log says `drone1: LAND REFUSED -- take manual control with the RC now`, skip straight to 4.

## "The drone is flying away"

A flyaway is the scenario with the least time to think. In order:

1. **Pilot: RTL on the mode switch.** Fastest correct action
2. **If it does not respond, switch to STABILIZE and fly it back manually**
3. **If it is out of RC range** — the failsafes are what save you now. `FS_THR_ENABLE` triggers
   RTL on RC loss; `FENCE_ACTION` triggers RTL on breach. **This is why Stage 0's failsafe table
   is not optional**
4. **Shout the direction and keep eyes on it.** Someone tracks it visually while someone else
   works the radio
5. **Do not chase it with another drone**

If it leaves the field entirely: note the last heading and altitude, check the dashboard's last
`east / north / up` reading, and pull the flight log afterwards (`logs/` on the ground station,
and the dataflash log on the aircraft).

## Emergency disarm — read before you need it

Disarming in flight **stops the motors and the aircraft falls.** It destroys the drone.

It is the right call in exactly one situation: **the drone is about to hit a person and nothing
else has worked.** A broken airframe is always cheaper than an injury.

The pilot does this on the RC — hold throttle down and full left yaw, or the emergency-stop
switch if one is assigned. Decide **before the flight** which it is on your transmitters, and make
sure every pilot knows. Finding out mid-incident is too late.

## If the ground station dies

Nothing to do on the laptop — it is gone. What happens by itself:

1. The bridge stops sending setpoints
2. **ArduPilot's `GUID_TIMEOUT` (3 s) zeroes its own target and holds position** — autonomously,
   needing neither the ground station nor a pilot
3. If `FS_GCS_ENABLE` is on, it then triggers **RTL** and the drone comes home

The drones will not fall and will not run away. Pilots should still take manual control to bring
them down in a controlled way rather than waiting for a battery failsafe.

## Taking manual control — throttle stick to MID first

**Found in the field, 30 Oct.** The safety pilot could change modes from the RC but could not fly
the aircraft. This is the cause, and it will happen again to anyone who has not been told.

GUIDED **ignores the roll, pitch and throttle sticks by design** — the ground station owns the
aircraft. There is no parameter that changes this; `GUID_OPTIONS` has no stick-override bit. The
only stick that works in GUIDED is **yaw** (with the default `GUID_OPTIONS = 0`).

So the handover *is* the mode switch. The trap is what the next mode does with the throttle stick:

| Mode you switch to | Throttle stick at BOTTOM | Throttle stick at MID |
| --- | --- | --- |
| LOITER / ALT_HOLD / POSHOLD | **descends** at `PILOT_SPD_DN` | holds altitude |
| STABILIZE | **motors to idle — it falls** | pilot flies it |

The ground station took off, so **the pilot's throttle stick is sitting at the bottom.** Flip the
mode switch from there and the aircraft immediately starts coming down. It feels like no control.

> ### The procedure
>
> 1. Safety pilot calls **"throttle centre"** and moves the stick to mid
> 2. Calls **"taking control"**
> 3. *Then* flips the mode switch to LOITER
>
> Stick first, switch second. Always that order.

**For a runaway, do not use the sticks at all.** Flip straight to **RTL** or **LAND**. ArduPilot
flies the whole recovery itself and it works regardless of where the throttle is sitting — no
stick skill, nothing to get wrong under pressure. This is the better tool for emergency 4 above.

### Set up before you fly

- **`FLTMODE1`–`FLTMODE6`**: put **LOITER** where the pilot can reach it blind, ideally one click
  from where the switch rests. Blind reachability beats tidy ordering.
- **`PILOT_THR_BHV = 1`** — *only* if the transmitter has a **sprung, self-centering throttle**.
  Mid-stick then becomes the stick's resting place and this failure mode disappears entirely.
  With a ratcheted throttle that stays where you put it, leave it at `0` and rely on the callout.
- **`THR_DZ`**: default 100 µs. Widening to ~150 makes "mid" more forgiving during a tense
  handover. Minor, but free.

### Once the pilot has it, they keep it

`command_bridge` runs with `hold_mode` empty, so it **never changes mode**, and ArduPilot ignores
`SET_POSITION_TARGET` outside GUIDED. The bridge's continued publishing is inert. Nothing on the
ground station takes the aircraft back by itself — only a human pressing **GUIDED** on the
dashboard, or starting `first_flight` again.

## Pilot briefing

Say all of this out loud before the first flight:

- **You outrank the laptop.** If something looks wrong, take control. No permission needed
- **Throttle stick to MID before you flip out of GUIDED** — otherwise the drone descends the
  instant you take it. See [Taking manual control](#taking-manual-control--throttle-stick-to-mid-first)
- **Which switch is RTL**, which is LAND, which is STABILIZE — point at them
- **Which switch or stick combination is emergency disarm**, and that it drops the aircraft
- **One pilot, one drone.** Do not watch two
- The ground station may command the drone at any moment during a test — **expect movement**
- The geofence radius and maximum altitude, in metres, for this field
- Where everyone stands, and that nobody walks under a drone

---

# Troubleshooting

Everything here has actually happened during development or testing.

| Symptom | Cause | Fix |
| --- | --- | --- |
| `CommandBool(success=False, result=4)` | pre-arm check refused; `result=4` is `MAV_RESULT_FAILED` | read `FCU: PreArm: <reason>` in **T1**. `statustext` is not in the allowlist, so the terminal is the only place it appears |
| `PreArm: Need Position Estimate` | GUIDED needs a position; no sky view | go outside, or use STABILIZE for a bench arm test |
| `PreArm: Motors: Check frame class and type` | `FRAME_CLASS=0` after a parameter wipe | `param set FRAME_CLASS 1` then `reboot` |
| MAVROS `VER: request timeout` then `FCU don't support AUTOPILOT_VERSION` | two clients on SITL's TCP 5760 | only one client per port. Use `--no-mavproxy`, or MAVProxy `output add` plus `use_mavproxy:=true` |
| `/droneN/odom` silent | `odom_relay` has not resolved offsets | read its log — it says why every 10 s |
| Two drones both read `(0,0,0)` metres apart | shift not applied | check `frame_id`; if not `swarm_origin`, offsets never resolved |
| Positions off by metres, not centimetres | drones were not RTK Fixed when offsets resolved | Ctrl-C `odom_relay`, wait for green, run again |
| `offset samples spread over N m` | something moved during the 25 samples | restart `odom_relay`, everything still |
| GPS drops Fixed → Float mid-flight | corrections going stale — RTCM not getting through | check **T2**; corrections share the radio with everything else |
| Teleop keys do nothing | no `command_bridge`, or drone not armed / not GUIDED | `ros2 topic info /drone1/cmd_vel` should show `Subscription count: 1` |
| Was responding, now ignores commands | watchdog tripped and `hold_mode` was set | `ros2 service call /drone1/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"` |
| Dashboard shows `DEAD 8s` / `NO DATA` | that drone stopped sending | check power and WiFi. **Trust this over a stale-looking number** |
| Battery row red on all four | thresholds are 4S defaults | 6S packs: `-p battery_warn_v:=21.0 -p battery_low_v:=19.8` |
| Terminal garbled after teleop | raw mode not restored | `stty sane` |
| `Permission denied` on `/dev/ttyACM0` | group not active in this shell | `newgrp dialout`, or re-enter the container |

---

# Known limits

- **The hold mode is off by default, deliberately.** When commands stop, the bridge streams zero
  velocity and the drone holds in GUIDED. If the bridge dies, `GUID_TIMEOUT` holds it
  autonomously. Setting `hold_mode:=LOITER` would make this **worse** — LOITER takes climb rate
  from the pilot's throttle and descends with nobody holding it. Use `BRAKE` if you want a
  re-handover barrier; never `LOITER` for an unattended hold.
- **Link gaps**: 500 ms measured in the lab, against ArduPilot's 3 s timeout. Re-measure at
  Stage 1 every session — it is the number that decides whether formation flight is viable.
- **Altitude in the shared frame is only as good as the barometers.** The offset is captured once;
  if baro drift differs between drones afterwards their `z` diverges. Use GPS as the EKF height
  source (`EK3_SRC1_POSZ`) when RTK is fixed.
- **Tested on hardware**: links, clamp, refusals, motor order, dashboard.
  **Tested only in simulation or synthetically**: the shared origin, the RTCM relay, the
  land-on-Ctrl-C path, and anything with four drones at once. Treat each first-on-hardware run as
  a test, not a demonstration.
- **`/droneN/status`**, the third topic in the RSCL contract, is not implemented.
- **`/mochila_rtk/pose` and `/mochila_rtk/gps`** (published by `nodo_mochila`) have no subscriber
  anywhere in this repo yet — the shared-frame position everything downstream actually uses is
  `/mochila_rtk/odom` from `odom_relay`, exactly like a drone. The two re-published topics exist
  for whatever the RSCL formation controller ends up wanting from the leader directly; until that
  controller lands, treat them as unverified.

# The interface handed to RSCL

| Topic | Type | Direction |
| --- | --- | --- |
| `/droneN/odom` | `nav_msgs/Odometry` | us → controller, 25 Hz, `frame_id: swarm_origin` |
| `/droneN/cmd_vel` | `geometry_msgs/Twist` | controller → us, world ENU, clamped |

Their controller should need nothing else. If it does, that is a gap in this stack, not theirs.
