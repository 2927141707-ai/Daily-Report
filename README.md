# 2026.11.13 给ORB-SLAM3增加稠密点云建图，使用奥比中光Gemini 2L实现
<img width="1920" height="1080" alt="图片" src="https://github.com/user-attachments/assets/279af02c-8807-4cc9-bf0d-01edfd4b42c5" />
<img width="544" height="483" alt="图片" src="https://github.com/user-attachments/assets/a6a4f898-b62f-4dc9-b4dd-305e5e575a1c" />

## 本次修改

在 Ubuntu 24.04、ROS2 Jazzy、Pangolin 0.6、OpenCV 4.6 环境下，为已有 ORB-SLAM3 增加 Gemini 2 L RGB-D 接入、关键帧稠密彩色点云、ROS2 保存服务和地图重新加载功能。

- 从驱动 CameraInfo 读取实机内参，检查深度注册与尺度；同步 RGB-D，统一去畸变和缩放。
- 关键帧 RGB-D 使用有界队列后台写盘；独立节点执行体素降采样和离群点过滤。
- 从最新关键帧 Twc 快照重新融合，支持位姿修正后的地图更新、关键帧剔除及独立地图隔离；不把未建立变换的地图直接叠加。
- 发布 PointCloud2、位姿、轨迹、跟踪状态及必要 TF，提供 launch 和 RViz 配置。
- 支持运行中 `/orb_slam3/save_map` 服务保存，以及 Ctrl+C 后等待最终位姿并保存。
- 保存彩色 PLY、关键帧 TUM 轨迹、标定、运行配置和关键帧 RGB-D；使用独立目录防止覆盖。
- PLY 重载仅恢复地图可视化，未实现 Atlas 保存/加载或跨运行重定位。

## 文件目录

| 路径 | 内容 |
|---|---|
| [work/ORB_SLAM3_p06](work/ORB_SLAM3_p06) | 本次修改的7个核心/构建文件及原始LICENSE；不是完整ORB-SLAM3源码 |
| [patches/orbslam3-pangolin06-dense.patch](patches/orbslam3-pangolin06-dense.patch) | 相对原始提交的全部核心修改补丁 |
| [work/ros2_ws/src/slam3_orbbec](work/ros2_ws/src/slam3_orbbec) | ROS2接入节点、后台建图、保存/重载、launch、RViz配置 |
| [outputs](outputs) | 构建、启动、重载脚本和使用说明 |
| [work/checks](work/checks) | 几何、队列、保存及ROS2集成验证程序 |
| [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json) | 原始提交编号、修改文件清单和SHA256校验值 |



## 如何使用这些修改文件

这是对已配置工程的修改归档，不能把只含修改文件的 `work/ORB_SLAM3_p06` 直接当作完整源码编译。

原始核心基线：[UZ-SLAMLab/ORB_SLAM3 @ 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)。Pangolin、ROS2和系统依赖需已经安装；本仓库不重新安装整套环境。

如果已有本文对应的完整工作区，可对照清单将同路径修改文件放回工程。若在新的仓库检出目录恢复完整核心，执行下列命令（仅用于尚未准备完整核心的目录）：

```bash
# 当前目录为Daily-Report仓库根目录
PROJECT_DIR="$PWD"
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git work/orbslam3_base
git -C work/orbslam3_base checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
git -C work/orbslam3_base apply --check "$PROJECT_DIR/patches/orbslam3-pangolin06-dense.patch"
git -C work/orbslam3_base apply "$PROJECT_DIR/patches/orbslam3-pangolin06-dense.patch"

# 拷入应用补丁后的完整源码，包括随附第三方库及示例。
cp -a work/orbslam3_base/. work/ORB_SLAM3_p06/
tar -xf work/ORB_SLAM3_p06/Vocabulary/ORBvoc.txt.tar.gz \
  -C work/ORB_SLAM3_p06/Vocabulary

mkdir -p work/dense_ros2_ws/src
ln -s ../../ros2_ws/src/slam3_orbbec work/dense_ros2_ws/src/slam3_orbbec
bash outputs/build_dense_slam3.sh
```

完成后启动相机、SLAM和稠密点云显示：

```bash
bash outputs/run_gemini_dense.sh rviz:=true
```

扫描范围较大时可以将体素改为3厘米，减少点数和内存消耗，代价是减少细节：

```bash
bash outputs/run_gemini_dense.sh rviz:=true voxel:=0.03
```

运行中另开终端保存当前快照：

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /orb_slam3/save_map std_srvs/srv/Trigger '{}'
```

该服务不会停止建图。若只需要最终地图，可直接在建图终端按Ctrl+C，等待 `FINAL MAP SAVED`。若出现 `FINAL SAVE FAILED`，需先排查错误，不能视为保存成功。

重新加载与查看：

```bash
bash outputs/reload_dense.sh /完整路径/maps/save_...
# 或使用独立点云窗口
python3 outputs/view_dense.py --latest
```

RViz的Fixed Frame使用重载程序打印的 `slam_map_N`。重载节点只发布点云和关键帧轨迹，没有实时相机TF；在Fixed Frame与数据frame_id一致时，离线查看出现 `No tf data` 全局警告不妨碍点云显示。

## 验证及限制

- 核心、附带DBoW2/g2o和ROS2节点编译完成，实际链接Pangolin 0.6及OpenCV 4.6。统一Eigen 16字节对齐后，解决先前g2o释放内存时的崩溃。
- 几何、颜色、深度单位、队列上限、地图隔离/位姿更正及保存重载测试通过。
- TUM经ROS2输入真实核心：完成服务保存、最终保存和重新加载；另测launch同时SIGINT正常退出。详细结果见开发阶段验证报告。
- 开发阶段工具环境无法访问相机USB设备；之后用户在桌面终端进行了Gemini实机扫描，2026-11-13保存结果为190609点。检查保存文件不等于已量化实机定位精度。
- 实际闭环/多地图合并场景、长时间运行性能和精度仍需专项验证；已有合成测试验证这些变化输入后的融合逻辑。
- 当前采用CPU关键帧重新融合，预览会随地图规模增加而延迟。默认点数上限600000，超限明确报错，可用保留的RGB-D和快照调整体素后重新融合。

详细文档：[稠密建图说明](outputs/稠密建图说明.md)、[开发阶段验证报告](outputs/稠密建图验证报告.md)、[参考来源与依赖](outputs/参考来源与依赖.md)。其中本机绝对路径和测试日志路径记录的是开发环境，日志与扫描数据未纳入本次上传。

## 参考和许可证

核心修改保留ORB-SLAM3原有版权和GPL许可证，ROS2接入包标注GPL-3.0-or-later。[核心LICENSE](work/ORB_SLAM3_p06/LICENSE)。第三方组件保留各自声明。

参考高翔[slambook2的RGB-D点云拼接示例](https://github.com/gaoxiang12/slambook2/blob/master/ch5/rgbd/joinMap.cpp)及其[MIT许可证](https://github.com/gaoxiang12/slambook2/blob/master/LICENSE)，沿用反投影与位姿变换思路，自行适配ORB-SLAM3/ROS2，未直接搬用ROS1或ORB-SLAM2接口。
