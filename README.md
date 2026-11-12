# ROSMASTER X3 ORB-SLAM3 ROS 2

An unofficial ROS 2 Humble integration of
[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) for the ROSMASTER X3
mobile robot.

本项目基于 UZ-SLAMLab 的 ORB-SLAM3 修改，面向 ROSMASTER X3，提供 Astra
RGB-D 真机接入、轨迹保存和运行状态评估。

## 主要功能

- ROS 2 Humble RGB-D 图像接入；
- Astra RGB-D 相机真机接入；
- 发布相机位姿和跟踪状态；
- 根据 `CameraInfo` 生成 ORB-SLAM3 运行配置；
- 保存相机轨迹、关键帧轨迹、逐帧状态和运行统计；
- 真机结果顺序编号，并在正常退出后自动生成报告；
- 支持无窗口运行和可选 Map Viewer。

## 已验证环境

| 项目      | 环境                         |
| --------- | ---------------------------- |
| 操作系统  | Ubuntu 22.04                 |
| ROS       | ROS 2 Humble                 |
| 真机平台  | ROSMASTER X3、Jetson Orin NX |
| 相机      | Astra RGB-D                  |
| SLAM 模式 | RGB-D                        |
| OpenCV    | 4.10.0（已验证真机环境）     |

## 文档

- [真机部署和编译说明](config/real_camera_deployment.md)
- [已部署小车的真机运行说明](docs/真车运行.md)

真机文档面向已经完成部署和编译的车辆，不是通用的新车安装教程。

## 与官方 ORB-SLAM3 的主要区别

本项目保留 ORB-SLAM3 作为核心算法，主要增加：

1. ROS 2 Humble RGB-D 接口；
2. ROSMASTER X3 真机运行入口；
3. Astra `16UC1` 深度图转换和 RGB-D 近似时间同步；
4. 运行时相机配置生成；
5. 位姿和跟踪状态 ROS 话题；
6. 轨迹、逐帧状态和运行统计保存；
7. 真机结果自动编号和保存后自动评估。

## 仓库结构

```text
.
├── config/                         # 真机部署说明和编译依赖检查
├── docs/                           # 中文真车运行文档
├── ros2_ws/src/orbslam3_x3_ros2/  # ROS 2 RGB-D 接口
├── scripts/                        # 真机构建、启动、保存和报告入口
├── src/ORB_SLAM3/                  # 修改后的 ORB-SLAM3 核心
├── tests/                          # 回归测试
└── third_party/Pangolin/            # 地图可视化依赖
```

构建目录、运行结果、日志、视频和解压后的 `ORBvoc.txt` 不在仓库中。
在真机工程根目录执行以下命令检查依赖并编译，构建脚本会自动解压词袋：

```bash
bash scripts/build_real_slam.sh --check
bash scripts/build_real_slam.sh
```

## 当前验证状态和限制

已经验证 RGB-D 输入、地图初始化、在线跟踪、轨迹保存和运行报告生成。

真机测试没有独立参考真值，因此不提供可信的 ATE、RPE 或 RMSE 精度结论。
移动轨迹精度、回环精度、地图几何精度和 Atlas 地图复用尚未完成验收。
真机默认也不录制原始 RGB-D 图像。

## 上游项目和许可证

本项目基于 [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)，
仓库根目录的 `LICENSE` 保留其 GNU General Public License v3 文本。
原项目的版权和许可证声明均应保留。

第三方组件仍分别遵循各自的许可证，详见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 及对应源码目录中的许可证文件。

## 
