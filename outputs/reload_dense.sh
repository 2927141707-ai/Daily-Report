#!/usr/bin/env bash
set -eo pipefail
out="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -lt 1 ]; then
  printf 'Usage: bash outputs/reload_dense.sh /path/to/save_bundle [--map N]\n' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$out/setup_dense_slam3.sh"
source "$out/../work/dense_ros2_ws/install/local_setup.bash"
export ROS_LOG_DIR="$out/reload_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 run slam3_orbbec reload_map.py "$@"
