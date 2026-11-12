# ROS 2 ORB-SLAM3 接口参考与实现说明

本文说明接口设计的参考来源及本仓库的实现位置。实际部署请阅读[真机部署与编译说明](real_camera_deployment.md)，操作小车请阅读[真车运行说明](../docs/真车运行.md)。

## 1. 接口设计参考

实现本项目接口时参考了以下 GitHub 项目的接口组织方式：

- [zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)：参考 RGB-D 图像订阅、`message_filters` 同步、`cv_bridge` 和 `TrackRGBD` 的调用关系；对应[RGB-D 节点源码](https://github.com/zang09/ORB_SLAM3_ROS2/blob/main/src/rgbd/rgbd-slam-node.cpp)。
- [X-IOT-study/orbslam3_ros](https://github.com/X-IOT-study/orbslam3_ros)：参考 ROS 2 Humble 接口与独立编译的算法核心之间的组织方式，以及 RGB-D 输入、位姿和状态输出。
- [suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker)：参考 ROS 2 Humble 的 RGB-D 接口集成方式；本项目不采用其 Docker 安装流程。

本仓库接口针对 ROSMASTER X3 实现，并非整体引入上述项目；未将这些参考项目的源码复制到接口实现中。

## 2. 本仓库的数据处理方式

本项目保留 ORB-SLAM3 核心，接口使用所选的系统 OpenCV，主要处理流程为：

1. 使用 `SensorDataQoS` 订阅彩色图像、深度图像和 `CameraInfo`。
2. 使用近似时间同步配对 RGB-D 图像。
3. 根据 `CameraInfo` 生成 ORB-SLAM3 相机配置。
4. 保留一个可被最新图像对替换的待处理位置，避免堆积过时图像。
5. 将 `16UC1` 深度转换为米，或直接使用以米为单位的 `32FC1` 深度。
6. 调用 `TrackRGBD`，并在独立的 `slam_map` 坐标系下发布位姿。

里程计、IMU、激光和参考真值话题均不作为该 RGB-D 接口的定位输入。

## 3. 源码与脚本位置

| 文件或目录 | 用途 |
| --- | --- |
| `ros2_ws/src/orbslam3_x3_ros2/` | ROS 2 RGB-D 接口包，可执行程序名为 `rgbd` |
| `scripts/build_real_slam.sh` | 真机构建入口 |
| `scripts/start_real_slam.sh` | 真机启动入口 |
| `scripts/run_real_slam.py` | 创建顺序编号的结果目录，等待算法退出并保存文件，再调用报告程序 |
| `scripts/evaluate_real_run.py` | 检查输入、跟踪和保存的轨迹，生成运行报告 |
| `src/ORB_SLAM3/src/System.cc` | 包含退出收尾修改：等待算法线程结束，并在等待前释放已停止的 Viewer |

真机运行报告检查的是输入、跟踪和记录状态。若要计算 ATE/APE、RPE 或 RMSE 定位精度，必须额外提供独立、同步且坐标关系明确的参考真值。
