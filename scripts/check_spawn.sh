#!/usr/bin/env bash
# Spawn one robot headless in the aisle, wait, report its Gazebo world pose, RTF and controllers, stop.
#   scripts/check_spawn.sh <robot> <face> [wait_s]     e.g. scripts/check_spawn.sh rby1 B 60
# Run in a fresh shell; it sources the right environment itself.

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robot=$1 face=$2 wait_s=${3:-60}
declare -A NAME=([vb2]=vb2 [g1]=galbot_g1 [rby1]=rby1 [moz1]=moz1)
source "$root/scripts/env.sh" harmonic || exit 1   # explicit: a bare `source` passes on our own $1

log="${LOG_DIR:-/tmp}/spawn_${robot}_${face}.log"
setsid ros2 launch givaudan_bringup robot_in_aisle.launch.py robot:="$robot" face:="$face" gui:=false \
    > "$log" 2>&1 &
pgid=$!
trap 'kill -INT -- -$pgid 2>/dev/null; sleep 5; kill -KILL -- -$pgid 2>/dev/null' EXIT
sleep "$wait_s"

msgs=gz.msgs
# own process group: `ros2 run` starts the bridge as a child, so stop the whole group
setsid ros2 run ros_gz_bridge parameter_bridge \
    "/world/shelf_aisle/dynamic_pose/info@tf2_msgs/msg/TFMessage[$msgs.Pose_V" \
    --ros-args -r /world/shelf_aisle/dynamic_pose/info:=/gz_poses > /dev/null 2>&1 &
bridge=$!
echo "== $robot face $face (log: $log)"
python3 "$root/scripts/measure_spawn.py" "${NAME[$robot]}" --seconds 10
kill -- -$bridge 2>/dev/null
echo "controllers:"; timeout 15 ros2 control list_controllers 2>&1 | sed 's/\x1b\[[0-9;]*m//g; s/^/  /'
echo "errors in log: $(grep -c -E '\[ERROR\]|process has died' "$log")"
grep -E '\[ERROR\]|process has died' "$log" | head -5
