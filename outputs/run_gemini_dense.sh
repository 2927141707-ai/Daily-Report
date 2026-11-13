#!/usr/bin/env bash
set -eo pipefail
out="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
# This working copy includes the thread-joining shutdown fix required before final pose export.
source "$out/setup_dense_slam3.sh"
source "$out/../work/dense_ros2_ws/install/local_setup.bash"
export SLAM3_RUN_DIR="$out/dense_runs/$(date +%Y%m%d-%H%M%S-%N)"
export ROS_LOG_DIR="$SLAM3_RUN_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"
if [ -n "${DISPLAY:-}" ] && [ -z "${PANGOLIN_WINDOW_URI:-}" ]; then
  export PANGOLIN_WINDOW_URI="x11://"
fi
printf 'RGB-D recording: %s\nStop with Ctrl+C, then wait for keyframe trajectory export.\n' "$SLAM3_RUN_DIR"
exec ros2 launch slam3_orbbec live.launch.py record_dense:=true "$@"
