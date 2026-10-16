#!/usr/bin/env bash
# Import the pinned source dependencies and build everything.
#   deps/harmonic  gz_ros2_control for Gazebo Harmonic   (vb2, g1, rby1)
#   deps/fortress  ros_gz for Gazebo Fortress             (moz1)
#   .              this repo + rochestra (src/external)
# The two deps/ underlays are never sourced together: see scripts/env.sh.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash

mkdir -p "$root/src/external" "$root/deps/harmonic/src" "$root/deps/fortress/src"
# deps/ are separate workspaces: keep the main build (repo root) from building them too,
# or its Fortress ros_gz / gz_ros2_control copies shadow the Harmonic ones.
touch "$root/deps/COLCON_IGNORE"
vcs import --skip-existing "$root/src/external" < "$root/repos/main.repos"
vcs import --skip-existing "$root/deps/harmonic/src" < "$root/repos/harmonic.repos"
vcs import --skip-existing "$root/deps/fortress/src" < "$root/repos/fortress.repos"

(cd "$root/deps/harmonic" && GZ_VERSION=harmonic colcon build --symlink-install \
    --packages-up-to gz_ros2_control --cmake-args -DBUILD_TESTING=OFF)
(cd "$root/deps/fortress" && GZ_VERSION=fortress colcon build --symlink-install \
    --packages-up-to gps_msgs ros_gz_sim ros_gz_bridge ros_gz_image ros_gz_interfaces --cmake-args -DBUILD_TESTING=OFF)
(cd "$root" && colcon build --symlink-install --base-paths src --packages-up-to givaudan_bringup --cmake-args -DBUILD_TESTING=OFF)
echo "Built. Next: source scripts/env.sh harmonic   (or fortress for moz1)"
