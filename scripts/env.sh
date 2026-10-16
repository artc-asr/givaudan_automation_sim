# Source, don't run:  source scripts/env.sh
# ROS 2 Humble + the Harmonic gz_ros2_control underlay + this workspace. (All robots run on
# Gazebo Harmonic since rochestra D19; `harmonic` is still accepted as an argument.)
_gas_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-harmonic}" in
  harmonic) ;;
  *) echo "usage: source scripts/env.sh   (Gazebo Harmonic only, rochestra D19)" >&2; return 1 ;;
esac
source /opt/ros/humble/setup.bash
source "$_gas_root/deps/harmonic/install/local_setup.bash"
source "$_gas_root/install/local_setup.bash"
unset _gas_root
