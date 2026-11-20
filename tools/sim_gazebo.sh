#!/usr/bin/env bash
# Start the Gazebo side of the swarm rehearsal.
#
#     ./tools/sim_gazebo.sh            four X500s, with the GUI
#     ./tools/sim_gazebo.sh --headless physics only, no window
#     ./tools/sim_gazebo.sh --single   one X500
#
# Gazebo MUST be running before SITL, because SITL sends to UDP 9002+10*i and needs something
# listening there. This script exits once the FDM ports are bound, so you know when to start SITL.
#
# The one thing people get wrong: GZ_SIM_RESOURCE_PATH has to include OUR models directory, not
# just ardupilot_gazebo's. Without it every drone fails with "Unable to find uri[model://x500_v2]"
# and the world loads with no vehicles in it -- the ports never bind and SITL hangs forever on
# "Waiting for connection".
set -u

WORLD_NAME="x500_swarm.sdf"
GUI_ARGS=""
for arg in "$@"; do
  case "$arg" in
    --headless) GUI_ARGS="-s" ;;
    --single)   WORLD_NAME="x500_single.sdf" ;;
    -h|--help)  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

if ! command -v gz >/dev/null; then
  echo "gz not found -- are you inside the 'humble' container?" >&2
  exit 1
fi
if ! command -v ros2 >/dev/null; then
  echo "ros2 not found -- source the workspace first:" >&2
  echo "  source <your workspace>/install/setup.bash" >&2
  exit 1
fi

SHARE="$(ros2 pkg prefix swarm_bringup)/share/swarm_bringup"
WORLD="$SHARE/gazebo/worlds/$WORLD_NAME"
MODELS="$SHARE/gazebo/models"

[ -f "$WORLD" ]  || { echo "world not found: $WORLD -- run colcon build" >&2; exit 1; }
[ -d "$MODELS" ] || { echo "models not found: $MODELS -- run colcon build" >&2; exit 1; }

# The ArduPilot bridge plugin, and our models. Both must be on the path.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$MODELS:$SHARE/gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"

if [ ! -f "$HOME/gz_ws/src/ardupilot_gazebo/build/libArduPilotPlugin.so" ]; then
  echo "libArduPilotPlugin.so missing -- build ardupilot_gazebo first (see docs/GAZEBO_RUNBOOK.md)" >&2
  exit 1
fi

# Already running? Leave it alone rather than starting a second server on the same ports.
if pgrep -f "[g]z sim" >/dev/null; then
  echo "Gazebo is already running. Stop it first:"
  echo "  ps -eo pid,args | grep '[g]z sim'   then kill those PIDs"
  exit 1
fi

LOG=/tmp/gz_swarm.log
echo "world   : $WORLD"
echo "models  : $MODELS"
echo "log     : $LOG"
echo

# setsid detaches it from this shell's process group, so it survives when the shell exits.
setsid nohup gz sim -v4 -r $GUI_ARGS "$WORLD" </dev/null >"$LOG" 2>&1 &
disown

# Wait for the ArduPilotPlugin to bind its FDM ports -- that is the real "ready" signal.
echo -n "waiting for FDM ports"
for i in $(seq 1 40); do
  BOUND=$(ss -lun 2>/dev/null | grep -cE ':(9002|9012|9022|9032)\b')
  if [ "$BOUND" -gt 0 ]; then
    echo
    echo "FDM ports bound: $(ss -lun 2>/dev/null | grep -oE ':(9002|9012|9022|9032)\b' | tr -d ':' | sort -u | tr '\n' ' ')"
    echo
    echo "Gazebo is ready. Now start SITL."
    exit 0
  fi
  if ! pgrep -f "[g]z sim" >/dev/null; then
    echo; echo "Gazebo exited. Last lines of $LOG:" >&2
    tail -12 "$LOG" >&2
    exit 1
  fi
  echo -n "."
  sleep 1
done

echo
echo "FDM ports never bound after 40s." >&2
if grep -q "Unable to find uri" "$LOG" 2>/dev/null; then
  echo >&2
  echo "The world could not find its vehicle model:" >&2
  grep -m1 "Unable to find uri" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | sed 's/^/  /' >&2
  echo "  -> GZ_SIM_RESOURCE_PATH is missing $MODELS" >&2
else
  tail -12 "$LOG" | sed 's/\x1b\[[0-9;]*m//g' >&2
fi
exit 1
