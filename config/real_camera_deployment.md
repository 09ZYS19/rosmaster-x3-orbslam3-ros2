# ROSMASTER X3 / Astra RGB-D 真机部署与编译

本文适用于 Ubuntu 22.04、ROS 2 Humble 和 Astra RGB-D 相机。请在实际运行的小车上编译源码，电脑上编译出的程序不能直接复制到 Jetson 使用。

仓库包含修改后的 ORB-SLAM3 核心、Pangolin、ROS 2 RGB-D 接口，以及真机构建、启动、保存和报告脚本。已经部署好的小车可直接阅读[真车运行说明](../docs/真车运行.md)。

## 1. 工程目录与共享小车注意事项

以下以 `/home/jetson/ORBSLAM3/` 为工程目录示例，实际路径不同时请相应替换。

首次部署时创建独立的新目录，避免覆盖厂家工程或其他人的工作区。例如：

```bash
mkdir /home/jetson/ORBSLAM3
```

如果提示目录已经存在，应先确认其中内容，不要直接覆盖。若使用源码压缩包，可在确认新目录为空后，使用 `--strip-components=1 --keep-old-files` 解压：前者去掉压缩包最外层目录，后者防止覆盖已有同名文件。

构建和启动脚本使用当前工程内的目录保存文件：

- `runtime/`：ROS 日志、编译缓存、临时文件和 colcon 状态；
- `local/`：本工程编译的 Pangolin；
- `results/real/`：每轮真车运行的轨迹、统计和报告。

脚本不修改终端启动文件、系统服务或相机标定文件，也不执行 `sudo`、`apt`、`ldconfig` 或系统级安装。缺少系统依赖时会停止，请先与设备维护者确认解决方式。环境变量的调整只作用于脚本及其子进程。

SLAM 默认输出话题为 `/zys_orbslam3/camera_pose` 和 `/zys_orbslam3/tracking_state`。底盘和相机驱动需要按真车运行文档单独启动，SLAM 脚本不发布运动指令。

独立目录仍会占用同一台小车的 CPU、内存、磁盘和相机。共用设备时，请协调相机使用，避免在别人运行任务时同时编译。

## 2. 相机输入要求

| 输入 | 默认话题 | 要求 |
| --- | --- | --- |
| 彩色图像 | `/camera/color/image_raw` | `rgb8` 或 `bgr8` |
| 深度图像 | `/camera/depth/image_raw` | 已对齐彩色图像的 `16UC1` 深度，或以米为单位的 `32FC1` 深度 |
| 彩色相机参数 | `/camera/color/camera_info` | 与彩色图像对应的针孔相机内参；畸变为空，或提供 5 个有限的 `plumb_bob` 畸变系数 |

彩色图像和深度图像的尺寸、光学坐标系名称必须一致。Astra 驱动需启用：

```text
depth_registration=true
color_depth_synchronization=true
use_sim_time=false
```

其中 `use_sim_time=false` 表示使用真实设备时间。

请确认 `CameraInfo` 中的 `K`、`D` 确实对应当前彩色图像，且深度图已经对齐到彩色图。`D` 全为零不能单独证明图像已经去畸变。节点使用彩色图像的 `K`、`D` 生成算法配置，不使用 `R`、`P` 自动配准原始图像。

## 3. 在 Jetson 上检查依赖并编译

进入实际工程目录，先检查依赖，再编译：

```bash
cd /home/jetson/ORBSLAM3
bash scripts/build_real_slam.sh --check
bash scripts/build_real_slam.sh
```

`--check` 只检查编译依赖，不执行编译或安装。完整构建脚本会编译仓库中的依赖、ORB-SLAM3 和 ROS 2 接口包，并自动解压词袋。

默认使用 2 个编译任务。内存紧张时可改为单任务：

```bash
BUILD_JOBS=1 bash scripts/build_real_slam.sh
```

所需依赖包括 OpenCV 4.4 或更高版本、Eigen、Boost serialization、OpenSSL 开发文件、OpenGL/GLEW 开发文件，以及 `package.xml` 中列出的 ROS 2 依赖。不要求安装 RealSense SDK。缺少依赖时，CMake 会提示并停止。

编译产物位于 `build-real/`、`local/`、`src/ORB_SLAM3/build/`、`src/ORB_SLAM3/Thirdparty/DBoW2/build/`，以及 `ros2_ws/` 下的 `build-real/`、`install-real/`、`log-real/`。词袋解压到 `src/ORB_SLAM3/Vocabulary/`，请为这些文件预留磁盘空间。

### OpenCV 版本选择

同一台设备上的不同软件包可能使用不同版本的 OpenCV。本接口不依赖 `cv_bridge`，而是将 ROS 图像数据复制到独立的 OpenCV 矩阵，处理行间填充和字节序；必要时将 BGR 转为 RGB，并对整数深度进行一次单位转换。

构建脚本依次检查以下位置的 OpenCV 配置：

1. `/usr/lib/<multiarch>/cmake/opencv4`，其中 `<multiarch>` 由编译器查询；
2. `/usr/lib/cmake/opencv4`；
3. `/usr/local/lib/cmake/opencv4`；
4. `/usr/local/share/OpenCV`。

DBoW2、ORB-SLAM3 和 ROS 2 接口统一使用选中的 OpenCV 配置。需要指定其他已安装版本时，可将 `ORB_REAL_OPENCV_DIR` 设置为包含 `OpenCVConfig.cmake` 的目录。脚本不修改系统已有的 OpenCV 或 `cv_bridge`。

## 4. 启动、保存与自动报告

相机驱动运行后，在 ROS 通信域和网络设置一致的另一个终端执行：

```bash
bash /home/jetson/ORBSLAM3/scripts/start_real_slam.sh
```

默认不显示图形窗口。每轮结果保存在 `results/real/1/`、`results/real/2/`、`results/real/3/` 等顺序编号目录中。

结束时先让小车停车，再在 SLAM 终端按一次 `Ctrl+C`，保持终端打开，等待轨迹保存和自动报告完成。报告检查输入、跟踪状态及保存文件；定位精度指标需要独立的参考轨迹。

## 5. 时间同步、深度单位与状态含义

真机默认使用 RGB-D 近似时间同步，最大时间戳差为 20 ms，可以通过参数调整。这个上限只限制配对的时间差，不代表相机曝光已实现硬件同步；超过上限的图像对不会用于处理。

`16UC1` 深度默认乘以 `0.001` 转换为米，且只转换一次；已经以米为单位的 `32FC1` 深度不再缩放。零值或非有限深度视为无效。畸变系数会写入 ORB-SLAM3 配置，其正确性依赖相机驱动与标定。

`paired`、`processed` 应持续增加；`tracked` 增加、`tracking_state=2` 表示正常跟踪。它们用于判断运行状态，不能代替独立的定位精度测量。`frames.csv` 记录每帧“深度时间戳减去彩色时间戳”的差值，单位为毫秒。

节点默认使用真机输入配置和 `/camera` 话题。驾驶、窗口显示、退出及报告查看步骤详见[真车运行说明](../docs/真车运行.md)。
