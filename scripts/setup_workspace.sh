#!/usr/bin/env bash
# Import the pinned source dependencies and build everything.
#   deps/harmonic  gz_ros2_control for Gazebo Harmonic (Humble's apt build is Fortress)
#   .              this repo + rochestra (src/external)
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash

mkdir -p "$root/src/external" "$root/deps/harmonic/src"
# deps/ is a separate workspace: keep the main build (repo root) from building it too,
# or its copy shadows the underlay's.
touch "$root/deps/COLCON_IGNORE"
vcs import --skip-existing "$root/src/external" < "$root/repos/main.repos"
vcs import --skip-existing "$root/deps/harmonic/src" < "$root/repos/harmonic.repos"

(cd "$root/deps/harmonic" && GZ_VERSION=harmonic colcon build --symlink-install \
    --packages-up-to gz_ros2_control --cmake-args -DBUILD_TESTING=OFF)
(cd "$root" && colcon build --symlink-install --base-paths src --packages-up-to givaudan_bringup --cmake-args -DBUILD_TESTING=OFF)
echo "Built. Next: source scripts/env.sh"
