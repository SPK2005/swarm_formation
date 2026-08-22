#!/usr/bin/env bash
#
# start_swarm.sh -- launch PX4 SITL multi-vehicle swarm + ROS 2 guidance stack.
#
# Stack: ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17.0 / Micro-XRCE-DDS-Agent v2.4.3
#
# Usage:
#   ./start_swarm.sh                                  # figure8 leader, rviz on
#   ./start_swarm.sh --leader circle --no-rviz
#   ./start_swarm.sh --n 1 --leader straight_line     # single-drone harness
#   ./start_swarm.sh --log-dir ~/swarm_logs
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
N_DRONES=4
LEADER="figure8"
USE_RVIZ="true"
LOG_DIR="${HOME}/swarm_logs/$(date +%Y%m%d_%H%M%S)"
GZ_MODEL="x500"
PX4_AIRFRAME=4001

# ---------------------------------------------------------------------------
# SINGLE SOURCE OF TRUTH: spawn poses in NED (north, east), metres.
#
# This table MUST match `self.spawn_ned` in follower_controller.py. PX4
# multi-vehicle SITL reports odometry relative to each drone's OWN spawn
# point, so a mismatch here injects a constant frame bias straight into
# e_xi / e_yi with no other symptom -- the formation still converges, just
# to the wrong place.
#
# Gazebo's PX4_GZ_MODEL_POSE is ENU. Conversion (NED -> ENU):
#     ENU_x = NED_east
#     ENU_y = NED_north
#     ENU_z = -NED_down
#     ENU_yaw = pi/2 - NED_heading
#
# The ENU string is DERIVED below rather than hard-coded, so the invariant
# holds structurally and cannot drift out of sync again.
# ---------------------------------------------------------------------------
declare -A SPAWN_NED=(
  [1]="3.0 3.0"
  [2]="-3.0 3.0"
  [3]="3.0 -3.0"
  [4]="-3.0 -3.0"
)
SPAWN_HEADING_NED=0.0   # all drones face North at spawn

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --n)        N_DRONES="$2"; shift 2 ;;
    --leader)   LEADER="$2";   shift 2 ;;
    --log-dir)  LOG_DIR="$2";  shift 2 ;;
    --rviz)     USE_RVIZ="true";  shift ;;
    --no-rviz)  USE_RVIZ="false"; shift ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \?//' | head -20
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

if (( N_DRONES < 1 || N_DRONES > 4 )); then
  echo "ERROR: --n must be 1..4 (spawn table has 4 entries)" >&2
  exit 1
fi

mkdir -p "$LOG_DIR"
echo "Logs -> $LOG_DIR"

# ---------------------------------------------------------------------------
# Cleanup: kill everything we started, on exit or Ctrl-C
# ---------------------------------------------------------------------------
PIDS=()
cleanup() {
  echo
  echo "Shutting down..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "Logs remain in $LOG_DIR"
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# ned_to_enu_pose <north> <east> -> "x,y,z,roll,pitch,yaw" ENU string
ned_to_enu_pose() {
  local north="$1" east="$2"
  python3 - "$north" "$east" "$SPAWN_HEADING_NED" <<'PY'
import math, sys
north, east, chi = (float(a) for a in sys.argv[1:4])
enu_x, enu_y, enu_z = east, north, 0.0
enu_yaw = math.pi / 2.0 - chi
print(f"{enu_x},{enu_y},{enu_z},0,0,{enu_yaw:.6f}")
PY
}

# wait_for <description> <timeout_s> <command...>
wait_for() {
  local desc="$1" timeout="$2"; shift 2
  local deadline=$(( SECONDS + timeout ))
  printf "  waiting for %s " "$desc"
  while (( SECONDS < deadline )); do
    if "$@" >/dev/null 2>&1; then
      printf " ok (%ds)\n" "$(( timeout - (deadline - SECONDS) ))"
      return 0
    fi
    printf "."
    sleep 0.5
  done
  printf " TIMEOUT after %ds\n" "$timeout"
  echo "ERROR: gave up waiting for $desc. Check $LOG_DIR" >&2
  return 1
}

agent_is_up()   { ss -lun 2>/dev/null | grep -q ':8888'; }
gazebo_is_up()  { gz topic -l 2>/dev/null | grep -q '^/world/'; }
px4_topic_is_up() {
  ros2 topic list 2>/dev/null | grep -q "^/px4_$1/fmu/out/vehicle_local_position$"
}

# ---------------------------------------------------------------------------
# 1. Micro-XRCE-DDS Agent
# ---------------------------------------------------------------------------
echo "[1/3] Starting Micro-XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
PIDS+=($!)
wait_for "agent on UDP 8888" 20 agent_is_up

# ---------------------------------------------------------------------------
# 2. PX4 SITL instances
#
# Instance 1 launches Gazebo. Instances 2..N attach to the already-running
# server and MUST set PX4_GZ_STANDALONE=1 -- previously this flag was on
# instance 1 instead, so every instance raced to create the world.
# ---------------------------------------------------------------------------
echo "[2/3] Starting $N_DRONES PX4 SITL instance(s)..."
for (( i = 1; i <= N_DRONES; i++ )); do
  read -r north east <<< "${SPAWN_NED[$i]}"
  pose="$(ned_to_enu_pose "$north" "$east")"

  standalone=1
  if (( i == 1 )); then
    standalone=0    # instance 1 owns the Gazebo server
  fi

  printf "  drone %d: NED(%s, %s) -> ENU pose %s\n" "$i" "$north" "$east" "$pose"

  (
    cd "$PX4_DIR"
    PX4_GZ_STANDALONE=$standalone \
    PX4_SYS_AUTOSTART=$PX4_AIRFRAME \
    PX4_GZ_MODEL="$GZ_MODEL" \
    PX4_GZ_MODEL_POSE="$pose" \
    ./build/px4_sitl_default/bin/px4 -i "$i"
  ) > "$LOG_DIR/px4_$i.log" 2>&1 &
  PIDS+=($!)

  if (( i == 1 )); then
    wait_for "Gazebo world" 90 gazebo_is_up
  fi
  wait_for "/px4_$i/fmu/out/vehicle_local_position" 90 px4_topic_is_up "$i"
done

# ---------------------------------------------------------------------------
# 3. ROS 2 guidance stack
# ---------------------------------------------------------------------------
echo "[3/3] Starting ROS 2 swarm launch..."
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"

ros2 launch swarm_formation swarm.launch.py \
  leader:="$LEADER" \
  rviz:="$USE_RVIZ" \
  n_drones:="$N_DRONES" \
  log_dir:="$LOG_DIR" \
  > "$LOG_DIR/swarm_launch.log" 2>&1 &
PIDS+=($!)

echo
echo "Swarm running. Ctrl-C to stop."
echo "  tail -f $LOG_DIR/swarm_launch.log"
wait
