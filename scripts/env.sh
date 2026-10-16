# Source, don't run:  source scripts/env.sh harmonic   (vb2, g1, rby1)
#                     source scripts/env.sh fortress   (moz1)
# Use a fresh terminal per Gazebo version: the two underlays must not mix
# (each provides its own ros_gz / gz_ros2_control for one Gazebo version).
_gas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "$1" in
  harmonic|fortress) ;;
  *) echo "usage: source scripts/env.sh harmonic|fortress" >&2; return 1 ;;
esac
if [ -n "${GIVAUDAN_GZ:-}" ] && [ "$GIVAUDAN_GZ" != "$1" ]; then
  echo "this shell already has the $GIVAUDAN_GZ environment; open a new terminal" >&2; return 1
fi
source /opt/ros/humble/setup.bash
source "$_gas_root/deps/$1/install/local_setup.bash"
source "$_gas_root/install/local_setup.bash"
export GIVAUDAN_GZ="$1"
unset _gas_root
