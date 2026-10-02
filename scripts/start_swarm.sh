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
# Carousel A/B (two runs differing ONLY in the Delta correction):
#   ./start_swarm.sh --leader sinusoid --wavelength 100 --delta off --tag nodelta
#   ./start_swarm.sh --leader sinusoid --wavelength 100 --delta on  --tag delta
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PX4_DIR="${PX4_DIR:-$HOME/tools/PX4-Autopilot}"
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
N_DRONES=4
LEADER="figure8"
USE_RVIZ="true"
LOG_DIR="${HOME}/swarm_logs/$(date +%Y%m%d_%H%M%S)"
GZ_MODEL="x500"
PX4_AIRFRAME=4001

# Valid leader node names. Must match the choices handled in swarm.launch.py.
VALID_LEADERS=(figure8 circle straight_line sinusoid)

# --- carousel / path options (passed through to swarm.launch.py) ------------
ENABLE_DELTA="true"     # follower's carousel heading correction (Delta)
LOG_TAG=""              # appended to the logger's CSV filename
LEADER_SPEED=""         # blank -> use the node's own default
LEADER_AMPLITUDE=""
LEADER_WAVELENGTH=""

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
usage() {
  sed -n '2,20p' "$0" | sed 's/^# \?//'
  echo
  echo "Leaders: ${VALID_LEADERS[*]}"
  echo "Carousel options:"
  echo "  --delta on|off        follower Delta correction   (default: on)"
  echo "  --tag NAME            suffix for the logger CSV    (default: none)"
  echo "  --speed M_PER_S       leader ground speed"
  echo "  --amplitude M         sinusoid amplitude A"
  echo "  --wavelength M        sinusoid wavelength"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --n)          N_DRONES="$2"; shift 2 ;;
    --leader)     LEADER="$2";   shift 2 ;;
    --log-dir)    LOG_DIR="$2";  shift 2 ;;
    --rviz)       USE_RVIZ="true";  shift ;;
    --no-rviz)    USE_RVIZ="false"; shift ;;
    --delta)
      case "$2" in
        on|true|1)   ENABLE_DELTA="true" ;;
        off|false|0) ENABLE_DELTA="false" ;;
        *) echo "ERROR: --delta expects on|off, got '$2'" >&2; exit 1 ;;
      esac
      shift 2 ;;
    --tag)        LOG_TAG="$2";            shift 2 ;;
    --speed)      LEADER_SPEED="$2";       shift 2 ;;
    --amplitude)  LEADER_AMPLITUDE="$2";   shift 2 ;;
    --wavelength) LEADER_WAVELENGTH="$2";  shift 2 ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

if (( N_DRONES < 1 || N_DRONES > 4 )); then
  echo "ERROR: --n must be 1..4 (spawn table has 4 entries)" >&2
  exit 1
fi

# Fail fast on a bad leader name instead of dying inside ros2 launch, where
# the error is buried in swarm_launch.log.
leader_ok="false"
for l in "${VALID_LEADERS[@]}"; do
  [[ "$LEADER" == "$l" ]] && leader_ok="true"
done
if [[ "$leader_ok" != "true" ]]; then
  echo "ERROR: --leader must be one of: ${VALID_LEADERS[*]} (got '$LEADER')" >&2
  exit 1
fi

# Path-shape options only mean anything for the sinusoid leader.
if [[ "$LEADER" != "sinusoid" ]] \
   && { [[ -n "$LEADER_AMPLITUDE" ]] || [[ -n "$LEADER_WAVELENGTH" ]]; }; then
  echo "WARNING: --amplitude/--wavelength are ignored for leader '$LEADER'" >&2
fi

mkdir -p "$LOG_DIR"
echo "Logs -> $LOG_DIR"
echo "Leader: $LEADER | Delta correction: $ENABLE_DELTA${LOG_TAG:+ | tag: $LOG_TAG}"

# Record the run configuration alongside the logs. Without this, two CSVs from
# an A/B are indistinguishable a week later.
{
  echo "date=$(date -Is)"
  echo "leader=$LEADER"
  echo "n_drones=$N_DRONES"
  echo "enable_delta=$ENABLE_DELTA"
  echo "tag=$LOG_TAG"
  echo "speed=${LEADER_SPEED:-<node default>}"
  echo "amplitude=${LEADER_AMPLITUDE:-<node default>}"
  echo "wavelength=${LEADER_WAVELENGTH:-<node default>}"
  echo "git_rev=$(git -C "$ROS_WS/src/swarm_formation" rev-parse --short HEAD 2>/dev/null || echo unknown)"
} > "$LOG_DIR/run_config.txt"

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
  # PX4 runs inside a subshell; killing the subshell does not always reap the
  # px4 binary itself, and a survivor holds UDP 8888 / the Gazebo world and
  # makes the NEXT run fail in confusing ways. Belt and braces.
  pkill -f 'build/px4_sitl_default/bin/px4' 2>/dev/null || true
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

  # PX4's rc.gzsim only checks `[ -z "$PX4_GZ_STANDALONE" ]` -- a string-
  # emptiness test, not a boolean one. "0" is non-empty, so it still reads
  # as "standalone". Instance 1 (which must launch Gazebo) needs the var
  # truly unset/empty; only instances 2..N (which attach to it) set "1".
  standalone=1
  if (( i == 1 )); then
    standalone=""   # instance 1 owns the Gazebo server
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

# Build the launch argument list. Optional path args are only appended when
# set, so the node's own defaults stay authoritative otherwise.
LAUNCH_ARGS=(
  leader:="$LEADER"
  rviz:="$USE_RVIZ"
  n_drones:="$N_DRONES"
  log_dir:="$LOG_DIR"
  enable_delta:="$ENABLE_DELTA"
  log_tag:="$LOG_TAG"
)
[[ -n "$LEADER_SPEED"      ]] && LAUNCH_ARGS+=( leader_speed:="$LEADER_SPEED" )
[[ -n "$LEADER_AMPLITUDE"  ]] && LAUNCH_ARGS+=( leader_amplitude:="$LEADER_AMPLITUDE" )
[[ -n "$LEADER_WAVELENGTH" ]] && LAUNCH_ARGS+=( leader_wavelength:="$LEADER_WAVELENGTH" )

ros2 launch swarm_formation swarm.launch.py "${LAUNCH_ARGS[@]}" \
  > "$LOG_DIR/swarm_launch.log" 2>&1 &
PIDS+=($!)

echo
echo "Swarm running. Ctrl-C to stop."
echo "  tail -f $LOG_DIR/swarm_launch.log"
wait