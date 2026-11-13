#!/usr/bin/env bash
_dense_work="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../work" && pwd)"
export ORB_SLAM3_ROOT="$_dense_work/ORB_SLAM3_p06"
export Pangolin_DIR=/usr/local/lib/cmake/Pangolin
export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT/lib:$ORB_SLAM3_ROOT/Thirdparty/DBoW2/lib:$ORB_SLAM3_ROOT/Thirdparty/g2o/lib:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _dense_work
