# 2026年11月11日 ORB-SLAM3在Ubuntu24.04下的安装和运行、用奥比中光Gemini 2L RGB-D相机建图
主要参照https://qd127.github.io/qdblog/2026/04/18/ORB-SLAM3/
有部分步骤跳过或增添。

环境如下：
系统：Ubuntu 24.04 Jazzy
CMake：系统源安装
OpenCV：4.6.0
Pangolin：v0.6
ORB-SLAM3：官方仓库源码

1、准备工作
先建立一个统一目录，把所有源码放一起

mkdir -p /home/wmy/slam
cd /home/wmy/slam

2、基础依赖安装

sudo apt update
sudo apt install -y \
git cmake build-essential pkg-config \
libgl1-mesa-dev libglew-dev libegl1-mesa-dev \
libwayland-dev libxkbcommon-dev wayland-protocols \
libepoxy-dev libopencv-dev libeigen3-dev \
libboost-serialization-dev libssl-dev

2.1 OpenCV安装

2.2 Eigen安装

3、源码获取

cd /home/wmy/slam
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git

3.1 Pangolin v0.6安装和兼容性处理

*相比于网址步骤增添

cd ~/slam/Pangolin/build

cmake .. \
  -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF

make -j1

4、旧 Pangolin 的清理（若曾装过其他版本参照网址执行，若没有安装过直接跳过）

5、处理 OpenCV 遗留问题（若曾装过其他版本参照网址执行，若没有安装过直接跳过）

6、ORB-SLAM3 编译前的兼容处理

7、编译 ORB-SLAM3

8、编译器成功后的验证

9、运行前检查

***10、数据集运行示例
命令与网址有些不同

10.1先下载一段测试序列

mkdir -p ~/slam/datasets
cd ~/slam/datasets

wget -c --tries=10 --timeout=30 \
  https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz

下载完成后执行：

tar -xzf rgbd_dataset_freiburg1_xyz.tgz

在终端执行

mkdir -p ~/slam/results/tum_rgbd
cd ~/slam/results/tum_rgbd

~/slam/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
  ~/slam/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/slam/ORB_SLAM3/Examples/RGB-D/TUM1.yaml \
  ~/slam/datasets/rgbd_dataset_freiburg1_xyz \
  ~/slam/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt

检查：
  
ls -lh ~/slam/results/tum_rgbd/

应生成：
  
CameraTrajectory.txt：相机轨迹。
KeyFrameTrajectory.txt：关键帧轨迹。
