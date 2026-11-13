#!/usr/bin/env bash
set -eo pipefail
out="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$out/setup_dense_slam3.sh"
for dep in DBoW2 g2o; do
  cmake -S "$ORB_SLAM3_ROOT/Thirdparty/$dep" -B "$ORB_SLAM3_ROOT/Thirdparty/$dep/build_dense" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 '-DCMAKE_CXX_FLAGS=-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16'
  cmake --build "$ORB_SLAM3_ROOT/Thirdparty/$dep/build_dense" --parallel 2
done
cmake -S "$ORB_SLAM3_ROOT" -B "$ORB_SLAM3_ROOT/build" -DCMAKE_BUILD_TYPE=Release -DPangolin_DIR="$Pangolin_DIR"
cmake --build "$ORB_SLAM3_ROOT/build" --target ORB_SLAM3 --parallel 2
cd "$out/../work/dense_ros2_ws"
colcon --log-base ../dense-ros-build-logs build --packages-select slam3_orbbec --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPangolin_DIR="$Pangolin_DIR" -DORB_SLAM3_ROOT="$ORB_SLAM3_ROOT"
