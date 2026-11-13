# 编译与运行

本文汇总仓库内所有组件的构建、测试和运行方式。各模块的设计、接口和实测数据见
对应主题文档：

- 实机传感器转发：[`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
- Gazebo + Betaflight SITL：[`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
- 里程计：[`odometry/overview.md`](odometry/overview.md)
- 开发镜像与 CI：[`development/docker_image.md`](development/docker_image.md)
- 项目进度与指标：[`status/project_status.md`](status/project_status.md)

所有命令默认在仓库根目录执行。

## 1. 环境

### 1.1 主机依赖

| 组件 | 需要 |
|---|---|
| 基础 | CMake ≥ 3.16、GCC/Clang 支持 C++17、`make` 或 `ninja` |
| `libflyrat` | Eigen3、OSQP、pthread |
| ROS 2 | ROS 2 Jazzy、`colcon`、`ros-dev-tools` |
| 仿真 | Gazebo Harmonic、`pkg-config gz-sim8`、Python 3 |
| 实机交叉编译 | `aarch64-linux-gnu-gcc/g++`、目标设备 `adb` |
| 开发容器 | Docker、Docker Compose、可选 buildx |

Ubuntu 上安装基础依赖：

```sh
sudo apt install build-essential cmake pkg-config libeigen3-dev libosqp-dev
```

### 1.2 环境限制

- ROS 2 的 `setup.bash` 必须在 `bash` 中 source。zsh 下直接 source 会因
  `BASH_SOURCE` 未定义而失败；使用 `bash -lc '...'`。
- 泰山派（RK3566，Buildroot）没有 OpenCV，内存约 2 GB；ROS 2 里程计节点和
  SchurVINS 目前只在开发机运行。
- 仿真和实机使用不同的 DDS/网络环境时，先确认 `ROS_DOMAIN_ID` 一致。

## 2. 一键构建

```sh
.script/build_all.sh
```

顺序：`libflyrat` → `flyrat_pilot` → Betaflight SITL → Gazebo bridge plugin →
ROS 2 工作区。

常用选项：

```sh
.script/build_all.sh --clean      # 先清理再构建
.script/build_all.sh --no-sim     # 跳过 SITL 和 Gazebo 插件
.script/build_all.sh --no-ros     # 跳过 ROS 2
```

其他脚本：

| 脚本 | 作用 |
|---|---|
| `.script/build_libflyrat.sh [--clean] [--debug] [--no-tests]` | 构建核心库，默认跑 ctest |
| `.script/build_flyrat_pilot.sh [--clean] [--debug]` | 构建实机程序与传感器转发 |
| `.script/build_ros2.sh [--clean] [-- <colcon 参数>]` | 构建 ROS 2 工作区 |
| `.script/build_betaflight.sh` | 构建 Betaflight SITL |
| `.script/build_plugin.sh` | 构建 Gazebo Betaflight 插件 |
| `.script/clean_build.sh [--dry-run] [--keep-sim]` | 清理所有构建产物 |
| `.script/export_compile_commands.sh` | 生成根目录 `compile_commands.json` |
| `.script/foxglove.sh [--port N]` | 启动 Foxglove bridge，默认 8765 |
| `.script/camera.sh [--prefix /flyrat/camera]` | 桥接 Gazebo 相机和 `/clock` |

## 3. libflyrat

### 3.1 构建与测试

```sh
.script/build_libflyrat.sh
```

等价于：

```sh
cmake -S libflyrat -B libflyrat/build -DCMAKE_BUILD_TYPE=Release
cmake --build libflyrat/build -j
ctest --test-dir libflyrat/build --output-on-failure
```

独立构建默认打开测试和工具；作为 `flyrat_pilot` 的子目录时默认关闭。

常用 CMake 选项：

| 选项 | 默认 | 说明 |
|---|---|---|
| `FLYRAT_BUILD_TESTS` | 独立构建 ON | 构建单元测试 |
| `FLYRAT_BUILD_TOOLS` | 独立构建 ON | 构建 bench 工具 |

找不到 OSQP 时配置会直接报错，需要安装 `libosqp-dev` 或提供
`osqpConfig.cmake`。

### 3.2 测试

```sh
ctest --test-dir libflyrat/build --output-on-failure
```

共 10 个测试；其中 `test_odometry_api` 覆盖接口契约，`test_schurvins_integration`
覆盖 SchurVINS 初始化、跟踪和轨迹输出。

### 3.3 工具

工具位于 `libflyrat/build/`。

`msp_interact_bench`：MSP 链路压测，可连串口或 TCP。

```sh
./libflyrat/build/msp_interact_bench --device /dev/ttyS3 --baud 921600 --seconds 30
./libflyrat/build/msp_interact_bench --host 127.0.0.1 --port 5761 --seconds 30
```

常用参数：`--imu-hz`（默认 200）、`--rc-hz`（默认 200）、`--enable-rc`。

`tof_bench`：4 颗 VL53L5CX 验收。

```sh
./libflyrat/build/tof_bench --device /dev/i2c-2 --seconds 10 \
  --resolution 4 --hz 60 --sensors 4
```

可选 `--interrupts`、`--lpn`、`--int`、`--poll-ms`。ToF 地址固定为
`0x60/0x62/0x64/0x66`，不支持通过参数修改。

`stereo_bench`：YUYV 双目相机采集。

```sh
./libflyrat/build/stereo_bench --device /dev/video10 \
  --width 1280 --height 480 --fps 30 --seconds 5
```

`--gray` 输出 GRAY8，`--save PREFIX` 指定左右 PGM 的文件名前缀。MJPEG 采集
和解码分别由 `StereoJpegSource` 与 `MppJpegDecoder` 负责。

`decode_bench`：MJPEG 硬件解码连续运行验收，只使用实时相机，交叉编译时由
`.script/build_pilot_arm.sh` 一并生成到 `build-arm/decode_bench`。

```sh
adb push build-arm/decode_bench /tmp/
adb shell '/tmp/decode_bench --camera /dev/video10 --mode both --seconds 60'
```

`--mode mpp|cpu|both` 选择硬件、软件或两者同时解码；`--verify-every N`
每 N 帧用 OpenCV 对比一次 Y 平面。输出包含帧率、超过 100 ms 的间隔、
平均/p99/最大解码耗时、进程 CPU、MPP 格式与 info-change 次数，最后给出
`PASS`/`FAIL`。

## 4. flyrat_pilot 与传感器转发

### 4.1 本机构建

```sh
.script/build_flyrat_pilot.sh
```

产物：

```text
flyrat_pilot/build/flyrat_pilot
flyrat_pilot/build/flyrat_sensor_forwarder
```

运行主程序（机载 SLAM；需要相机、飞控串口和标定文件）：

```sh
./flyrat_pilot/build/flyrat_pilot \
  --camera-config libflyrat/configs/camera/uvc_640x480.yaml \
  --imu-config flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml
```

默认使用 CPU `cv::imdecode`。机载 RK3566 可加 `--decode mpp` 启用硬件
解码（`MppJpegDecoder`，需 `librockchip_mpp.so`）；MPP 初始化失败或连续
10 帧解码失败会自动回退到 CPU，并在统计行累加 `decode_fallbacks`。

加 `--trace DIR` 可写出逐帧耗时：`DIR/trace_pilot.csv`（排队、解码、IMU
等待、收集、`track()`、发布、帧年龄）和 `DIR/trace_frontend.csv`（SVO 的
`pyramid_creation`、`sparse_img_align`、`reproject`、`pose_optimizer`、
`point_optimizer`、`frontend_time`）。随后用
`python3 flyrat_pilot/tools/analyze_trace.py DIR` 汇总 p50/p95/p99 和阶段
占比。不传 `--trace` 时不产生文件。

录制/回放机载传感器（相机 JPEG + IMU）：

```sh
# 录制到 <DIR>/sensor_bag.bin
./flyrat_pilot/build/flyrat_pilot ... --record /tmp/bag1 --max-frames 600
# 回放：不打开相机和 MSP，按原始时间间隔重放
./flyrat_pilot/build/flyrat_pilot ... --replay /tmp/bag1
```

回放同样支持 `--trace`、`--trajectory`、`--decode` 和 `--max-frames`，播完
最后一个图像帧后自动退出。`--record` 与 `--replay` 互斥。bringup 脚本可用
`record:=DIR` / `replay:=DIR`。

只校验标定、不打开硬件：

```sh
./flyrat_pilot/build/flyrat_pilot --check-config \
  --camera-config libflyrat/configs/camera/uvc_640x480.yaml \
  --imu-config flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml
```

完整参数见 [`flyrat_pilot/README.md`](../../flyrat_pilot/README.md)。

运行传感器转发：

```sh
./flyrat_pilot/build/flyrat_sensor_forwarder --listen 127.0.0.1:5000
```

完整参数：

```text
--listen ADDR:PORT      默认 127.0.0.1:5000
--msp-device PATH       默认 /dev/ttyS3
--msp-baud N            默认 921600
--msp-raw-rc-hz N       MSP 覆盖前 RC 查询频率，默认 50，0 关闭
--camera-device PATH    默认 /dev/video10
--camera-width N        默认 1280（左右拼接总宽）
--camera-height N       默认 480
--camera-fps N          交付帧率，默认 60；采集始终使用传感器最大 fps
--no-imu --no-tof --no-stereo
--help
```

### 4.2 交叉编译到泰山派

`flyrat_sensor_forwarder` 不依赖 OpenCV。完整交叉构建
（`.script/build_pilot_arm.sh`）会同时生成 `build-arm/flyrat_sensor_forwarder`；
也可以只用下面的脚本单独构建。两者功能一致，产物不要同时推送：

```sh
.script/build_sensor_forwarder_arm.sh
```

单独构建的产物是 `build-arm/flyrat_sensor_forwarder`（AArch64 静态链接）；
`build_pilot_arm.sh` 生成的同名文件是动态链接版本。

`flyrat_pilot` 需要 OpenCV、yaml-cpp 和 glog。目标 rootfs 没有这些库，因此
使用 `.script/build_pilot_arm.sh` 链接 sysroot 中的静态依赖：

```sh
.script/build_pilot_arm.sh
```

产物是 `build-arm/flyrat_pilot`（约 11 MB，`readelf -d` 只依赖
`libc/libm/ld-linux`）。脚本参数：

```text
--clean                 先删除 build-arm/pilot-cmake 和旧二进制
FLYRAT_SYSROOT=...      默认 /opt/sysroots/arm64
BUILD_DIR=...           默认 <repo>/build-arm
JOBS=...                默认 4，避免构建内存不足
```

### 4.2.1 最小静态 OpenCV

`/opt/sysroots/arm64` 需要一份关闭 OpenGL/TBB/GTK 的静态 OpenCV，安装到
`$FLYRAT_SYSROOT/usr/local`。参考配置（OpenCV 4.10.0）：

```sh
cmake -S opencv-4.10.0 -B opencv-arm-build \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/.script/toolchains/aarch64-linux-gnu.cmake \
  -DCMAKE_INSTALL_PREFIX=/opt/sysroots/arm64/usr/local \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_LIST=core,imgproc,imgcodecs,highgui,video,calib3d \
  -DWITH_OPENGL=OFF -DWITH_TBB=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF \
  -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_OPENCL=OFF -DWITH_LAPACK=OFF \
  -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_V4L=OFF \
  -DWITH_JASPER=OFF -DWITH_WEBP=OFF -DWITH_TIFF=OFF \
  -DWITH_OPENJPEG=OFF -DWITH_OPENEXR=OFF -DWITH_GDAL=OFF -DWITH_GDCM=OFF \
  -DWITH_JPEG=ON -DBUILD_JPEG=ON -DWITH_PNG=ON -DBUILD_PNG=ON \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_python3=OFF \
  -DCPU_BASELINE=NEON -DCPU_DISPATCH=
cmake --build opencv-arm-build -j4
sudo cmake --install opencv-arm-build
sudo aarch64-linux-gnu-ar rcs \
  /opt/sysroots/arm64/usr/local/lib/opencv4/3rdparty/libade.a
```

最后一条创建空 `libade.a`，因为 OpenCV 导出的链接列表引用了它，但 G-API
没有构建，实际不会链接。glog、yaml-cpp、gflags 使用 sysroot
`/usr/lib/aarch64-linux-gnu` 下的静态库。

### 4.2.2 手动推送

```sh
adb -s 4c24d56142f3d0e3 push build-arm/flyrat_pilot /tmp/
adb -s 4c24d56142f3d0e3 shell chmod +x /tmp/flyrat_pilot
adb -s 4c24d56142f3d0e3 push build-arm/flyrat_sensor_forwarder /tmp/
adb -s 4c24d56142f3d0e3 shell chmod +x /tmp/flyrat_sensor_forwarder
```

`.script/bringup_real.sh` 和 `.script/bringup_pilot_slam.sh` 每次启动都会
自动推送对应二进制，因此重启后 `/tmp` 被清空也不需要手动处理。

### 4.3 转发到 ROS 2

```sh
adb -s 4c24d56142f3d0e3 forward tcp:5000 tcp:5000

cd flyrat_play/ros2_ws
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install \
  --packages-select flyrat_sensors_interfaces flyrat_sensors && \
  source install/setup.bash && \
  ros2 launch flyrat_sensors sensors.launch.py'
```

或直接运行：

```sh
ros2 run flyrat_sensors sensor_receiver_node
```

可执行文件名是 `sensor_receiver_node`。话题：

```text
/flyrat/sensors/imu
/flyrat/sensors/tof_0 … /flyrat/sensors/tof_3
/flyrat/sensors/stereo/left/image_raw
/flyrat/sensors/stereo/right/image_raw
/flyrat/sensors/stereo/compressed
```

`stereo/compressed` 是左右拼接的原始 MJPEG（`CompressedImage`，`format=jpeg`），
`header.stamp` 与左右 `Image` 完全相同，录制时只录这一条即可避免双目丢帧
不一致。回放时用 `stereo_splitter_node` 拆回左右话题。

默认 `stereo_color=false`，发布 mono8；需要彩色时设置
`stereo_color:=true`，发布 bgr8。

### 4.4 机载 SLAM 与位姿回传

`flyrat_pilot` 在设备上本地运行 SchurVINS，只通过 TCP 5001 回传 64 字节
位姿；主机 `flyrat_pilot_bridge` 发布 ROS 2 话题和 TF。这样相机/IMU 数据不再
经过 adb，主机只负责可视化和录包。

```sh
# 推送 + adb forward + 设备 SLAM + 主机 bridge + Foxglove
.script/bringup_pilot_slam.sh start_foxglove:=true

# 只跑 25 fps，不启动 Foxglove，跑到手动 Ctrl-C
.script/bringup_pilot_slam.sh fps:=25
```

可选参数：

```text
fps:=N              相机帧率，默认 25
max_frames:=N       处理 N 帧后退出，0 表示一直运行
decode:=mpp|cpu     MJPEG 解码器，默认 mpp（失败自动回退 cpu）
trace:=DIR          设备端开启分阶段计时，退出时拉到 trace/DIR
slam_cpu:=N         已忽略；并行重投影需要多个核
trajectory:=PATH    设备端 TUM 轨迹，默认 /tmp/flyrat_pilot.tum
camera_config:=PATH 主机标定文件，同时推送到设备
imu_config:=PATH    IMU 标定文件，同时推送到设备
start_foxglove:=true 启动 foxglove_bridge
world_frame:=...    bridge 的 world frame，默认 odometry_map
body_frame:=...     bridge 的 body frame，默认 imu_link
FLYRAT_PUSH_PILOT=0 跳过二进制推送
```

主机话题：

```text
/flyrat/odometry/odometry   nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path       nav_msgs/Path
/flyrat/odometry/status     diagnostic_msgs/DiagnosticStatus，1 Hz
```

静态 TF 为 `imu_link -> camera_left_optical_frame` 和
`camera_left_optical_frame -> camera_right_optical_frame`，动态 TF 为
`odometry_map -> imu_link`。Foxglove 中把 Fixed frame 设为 `odometry_map`，
添加 Odometry、Path 和 TF 面板即可。

性能基线（RK3566，640x480/眼，静态近距桌面）：

| 模式 | CPU | RSS | `track()` | 帧年龄 | 处理帧率 |
|---|---:|---:|---:|---:|---:|
| 25 fps（NEON + 并行重投影） | 约 1.23 核 | 约 22 MB | 16–17 ms | 约 50 ms | 25.0 fps |
| 30 fps（NEON + 并行重投影） | 约 1.44 核 | 约 22 MB | 15–16 ms | 约 43 ms | 30.0 fps |
| 25 fps（P0 优化后） | 约 1.3 核 | 18–19 MB | 25–30 ms | 约 51 ms | 25.0 fps |
| 25 fps（优化前） | 约 1.3 核 | 27 MB | 约 28 ms | 约 47 ms | 25.1 fps |
| 30 fps（优化前） | 1.5–2.0 核 | 27–30 MB | 18–21 ms | 约 41 ms | 29.9 fps |
| 20 fps | 约 1.6 核 | 22 MB | 约 35 ms | 约 57 ms | 19.7 fps |

当前版本启用 NEON 与双目并行重投影后，25 fps 下 `track_ms` 约 16–17 ms，
40 ms 帧预算余量约 23 ms；30 fps 约 15–16 ms，预算 33 ms，余量约 17 ms。
默认保持 **25 fps**，飞行需要更大余量时可用 `fps:=20`。相机始终按传感器最高
帧率采集（1280x480 为 60 fps），表中的帧率是交付帧率；多余帧在拷贝 JPEG 前丢弃，
同时把自动曝光上限限制为 1/60 s。端到端（采集到主机收到）约
70–100 ms，其中设备时钟比主机快 55–64 ms，`ros2 topic delay` 需要先校正。
pilot 每 2 s 输出的 `camera_fps/slam_fps/track_ms/age_ms/cam_throttled/cpu/rss_mb` 可用于
复测；`track_ms` 是单帧后端耗时，`age_ms` 含 10 ms IMU lookahead 等待。

设备位于 `RECENTLY_LOST` 时不发布无效位姿，因此 Foxglove 上的轨迹会暂停；
状态话题中的 `state` 会显示当前状态。跟踪失败后发送 `adb shell kill -USR1
$(pidof flyrat_pilot)` 可重建前端并重新初始化。

## 5. ROS 2

### 5.1 构建

```sh
.script/build_ros2.sh
```

或手动：

```sh
cd flyrat_play/ros2_ws
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

`flyrat_odometry` 的节点会链接 libflyrat 的 SchurVINS 后端，并使用 OpenCV
转换图像消息和解析标定文件。

每次运行前 source：

```sh
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /workspaces/flyrat/flyrat_play/ros2_ws/install/setup.bash && \
  ros2 topic list'
```

### 5.2 包

| 包 | 作用 |
|---|---|
| `flyrat_sensors_interfaces` | `TofZone`、`TofFrame` 自定义消息 |
| `flyrat_sensors` | TCP 接收、MJPEG 解码、ROS 发布 |
| `flyrat_odometry` | 里程计节点（SchurVINS 双目惯性后端、TF、状态） |
| `flyrat_pilot_bridge` | 接收设备位姿流，发布 Odometry/Path/TF |
| `flyrat_msp` | MSP 协议接入 |
| `flyrat_control` | 控制节点与 SITL launch |

`sensors.launch.py` 参数：`host`（默认 127.0.0.1）、`port`（默认 5000）、
`stereo_color`、`acc_lsb_per_g`、`gyro_lsb_per_dps`、`imu_accel_axis`（默认
`none`）、`imu_gyro_axis`（默认 `none`）。当前实机加速度和陀螺仪原始数据
都已是 ROS FLU，实测左偏航 `angular_velocity.z > 0`。只有偏航为顺时针正的
飞控才需要 `bf_to_flu`（翻 z）；真正的 FRD 数据源用 `frd_to_flu`。

启动时还会加载 `flyrat_sensors/config/imu_intrinsics.yaml`：打开
`imu_intrinsics_enable` 后，节点按 Kalibr 的 scale/misalignment 模型修正
加速度和陀螺仪，并从 IMU 时间戳中扣除 `imu_time_offset`（相机到 IMU 的
固定时间差，约 3.8 ms）。YAML 中 `T_i_b` 为单位阵；零偏不写入配置，由
SchurVINS 在线估计。关闭该功能后发布的是未修正的原始刻度数据。

## 6. Gazebo + Betaflight SITL

### 6.1 准备

```sh
git submodule update --init --recursive
.script/build_betaflight.sh
.script/build_plugin.sh
```

要求 `pkg-config --exists gz-sim8` 成功。SITL 产物：

```text
flyrat_play/gazebo/third_party/betaflight/obj/main/betaflight_SITL.elf
```

插件产物：

```text
flyrat_play/gazebo/third_party/aeroloop_gazebo/plugins/build/libBetaflightPlugin.so
```

### 6.2 启动

```sh
flyrat_play/gazebo/scripts/launch_sim.sh
HIDE_GAZEBO=1 flyrat_play/gazebo/scripts/launch_sim.sh   # 无界面
```

首次或配置丢失时：

```sh
flyrat_play/gazebo/scripts/configure_fc.sh
```

自检：

```sh
python3 flyrat_play/gazebo/scripts/bringup_test.py
python3 flyrat_play/gazebo/scripts/bringup_test.py --takeoff
```

### 6.3 ROS 2 控制

```sh
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source flyrat_play/ros2_ws/install/setup.bash && \
  ros2 launch flyrat_control flyrat_sitl.launch.py auto_takeoff:=true'
```

MPC 示例：

```sh
ros2 launch flyrat_control flyrat_sitl.launch.py \
  controller:=ommpc auto_takeoff:=true flight_seconds:=120 msp_override:=false
```

仿真细节、端口和话题见
[`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)。

## 7. 里程计

ORB-SLAM3 已从仓库移除。`libflyrat` 提供 `flyrat::odometry` 接口，默认后端为
**SchurVINS**（SVO 2.0 前端 + Schur complement EKF），ROS 2 侧对应
`flyrat_odometry` 包。算法和接口细节见
[`odometry/overview.md`](odometry/overview.md)。

SchurVINS 及其内置依赖为 **GPL-3.0**；链接后的 `odometry_node` 组合二进制
受 GPL 约束，libflyrat 自身源文件仍为 MIT。

### 7.1 构建与测试

`flyrat_odometry` 目标默认随 libflyrat 构建，并默认启用 SchurVINS：

```sh
.script/build_libflyrat.sh
ctest --test-dir libflyrat/build --output-on-failure
```

SchurVINS 需要 OpenCV、yaml-cpp、glog 和 gflags。只构建接口或缺少依赖时：

```sh
cmake -S libflyrat -B libflyrat/build -DFLYRAT_WITH_SCHURVINS=OFF
cmake --build libflyrat/build -j
```

测试共 10 个；`test_odometry_api` 覆盖状态枚举、参数校验、空图像/尺寸不匹配
拒绝、无 IMU 时的返回值和重复 `shutdown`，`test_schurvins_integration` 覆盖
初始化、跟踪和 TUM 轨迹写入。

ROS 2 节点：

```sh
cd flyrat_play/ros2_ws
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select flyrat_sensors flyrat_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

节点使用 OpenCV 将 ROS 图像消息转换为 `ImageView`，并用 `cv::FileStorage`
读取标定文件。

### 7.2 标定配置

`libflyrat/configs/camera/uvc_640x480.yaml` 保存 DECXIN 双目 UVC 相机和
Betaflight IMU 的实机标定：

- `camera.left` / `camera.right`：640x480 每目内参和 plumb-bob 畸变；
- `camera.T_left_right`：左目 ← 右目，`p_left = T_left_right * p_right`；
- `imu.T_body_camera`：机体/IMU ← 相机，`p_body = T_body_camera * p_camera`；
- `imu.*`：200 Hz、Allan 噪声和随机游走。

数值来自 `kalibr_ws/`，不要随意改写。ROS 包安装时会把该文件复制到
`share/flyrat_odometry/config/`，也可以通过 `camera_config_path` 指向源码树
或其他文件。

### 7.3 C++ API

头文件 `libflyrat/include/flyrat/odometry/odometry.hpp`：

```cpp
flyrat::odometry::Options options;
options.mode = flyrat::odometry::Mode::kImuStereo;
options.camera_config_path = ".../uvc_640x480.yaml";

flyrat::odometry::Odometry odometry(options);
auto result = odometry.track(left, right, timestamp, imu_samples);
```

`ImageView` 是非拥有型的 8 位灰度视图。门面在
`libflyrat/src/odometry/odometry.cpp`，SchurVINS 适配在
`libflyrat/src/odometry/schurvins_backend.cpp`；启用后端时只接受
`Mode::kImuStereo`，`track()` 输出左目 `T_world_camera`。

IMU 必须覆盖帧时间戳两侧：适配层只在 `first_imu <= frame.timestamp <
last_imu` 时调用前端，否则返回 `kRecentlyLost`。ROS 节点会自动等待
`imu_lookahead`（默认 10 ms）内的帧后样本。

SchurVINS 的有效深度范围已按近场设为 0.25–20 m，初始化阈值为 40 个特征、
40 个跟踪点、30 个内点、视差 10 px。更远的室外场景需要重新调整
`schurvins_backend.cpp` 中的 `stereo.*_depth_inv` 和 `initialization.*`；
否则点会被深度范围拒绝，初始化延迟期间纯 IMU 积分会造成明显漂移。

`updated seeds are unconverged` 是深度 seed 未收敛的提示，不是崩溃；内置代码
已节流为每 100 次一条。`Downsizing storage not implemented` 已通过实现
`Frame::resizeFeatureStorage()` 的缩容分支消除。逐帧调试可用
`-DFLYRAT_SCHURVINS_VERBOSE=ON` 重新打开。

### 7.4 ROS 2 节点

先按第 4.3 节启动设备转发和 `flyrat_sensors`，再运行：

```sh
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /workspaces/flyrat/flyrat_play/ros2_ws/install/setup.bash && \
  ros2 launch flyrat_odometry odometry.launch.py'
```

`camera_config_path` 默认使用包内安装的 `uvc_640x480.yaml`；
`body_frame:=imu_link` 是默认机体坐标系；`world_frame` 默认
`odometry_map`。节点会发布静态 `imu_link -> camera_left_optical_frame` 和
`camera_left_optical_frame -> camera_right_optical_frame`（来自
`camera.T_left_right`），跟踪成功后发布动态 `odometry_map -> imu_link`。
其余参数见 `flyrat_odometry/config/odometry.yaml`。

SchurVINS 必须有 IMU，`mode:=stereo` 只对关闭后端后的桩实现有意义；启用
SchurVINS 时传 `mode:=stereo` 会因模式校验失败而退出。

记录 TUM 轨迹：

```sh
ros2 launch flyrat_odometry odometry.launch.py trajectory_path:=/tmp/flyrat.tum
```

输出：

```text
/flyrat/odometry/odometry   nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path       nav_msgs/Path，机体轨迹
/flyrat/odometry/status     diagnostic_msgs/DiagnosticStatus，1 Hz
/flyrat/odometry/reset      std_srvs/Trigger
/tf                         动态 odometry_map -> imu_link 与静态相机 TF
```

Foxglove 预览：

```sh
.script/foxglove.sh
```

默认 WebSocket 是 `ws://localhost:8765`。把固定坐标系设为 `odometry_map` 或
`imu_link`，添加 3D 面板显示 `/tf`，再添加 Diagnostic 面板查看
`/flyrat/odometry/status`。后端输出位姿后，可继续添加
`/flyrat/odometry/path` 和 `/flyrat/odometry/odometry`。

IMU 的 scale/misalignment 在 `sensor_receiver_node` 发布前按
`flyrat_sensors/config/imu_intrinsics.yaml` 修正，IMU 时间戳扣除
`imu_time_offset`（约 3.8 ms）。这些预处理与具体里程计算法无关，继续保留。
大段 IMU 断流后若跟踪停在 `kPaused`，调用 `/flyrat/odometry/reset` 重新初始化。

### 7.5 一键 bringup

`bringup.launch.py` 把 adb forward、设备端 forwarder、`sensor_receiver_node` 和
`odometry_node` 串成一条启动链：

```sh
.script/bringup_real.sh                         # 传感器 + 里程计
.script/bringup_real.sh trajectory_path:=/tmp/flyrat.tum
.script/bringup_real.sh start_foxglove:=true    # 再加 Foxglove bridge
.script/bringup_real.sh start_odometry:=false   # 只看传感器
.script/bringup_real.sh start_adb_forward:=false start_device:=false  # 复用已有转发
```

脚本会 source ROS 2 和 `flyrat_play/ros2_ws/install/setup.bash`，然后执行：

```sh
ros2 launch flyrat_odometry bringup.launch.py
```

默认行为：

- 把 `build-arm/flyrat_sensor_forwarder` 推到设备 `/tmp` 并加执行权限
  （设置 `FLYRAT_PUSH_FORWARDER=0` 可跳过）；
- 建立 `adb forward tcp:5000 tcp:5000`；
- 先结束设备上旧的 `flyrat_sensor_forwarder`，再启动
  `/tmp/flyrat_sensor_forwarder --listen 127.0.0.1:5000`；
- 设备 forwarder 意外退出时整个 launch 停止，不会让接收节点无限重连；
- 1 秒后启动 `sensor_receiver_node`（ToF 初始化约 7 秒，接收节点会自动重连）；
- 1 秒后启动 `odometry_node`；
- 使用脚本时，`Ctrl-C` 会停止设备端 forwarder 并删除 adb forward；直接
  `ros2 launch` 不会做这两步清理。

`camera_config_path` 默认优先使用源码树的
`libflyrat/configs/camera/uvc_640x480.yaml`，所以改外参后不需要重新
`colcon build`。也可以显式指定其他文件。

常用参数：

```text
adb_serial=            # 多设备时指定；默认用唯一连接的设备
device_binary=/tmp/flyrat_sensor_forwarder
local_port=5000
device_port=5000
start_device=true
start_adb_forward=true
start_sensors=true
start_odometry=true
mode=imu_stereo        # 纯双目用 mode:=stereo
camera_fps=60          # 交付帧率；设备始终以最大 fps 采集（1280x480 为 60），录制默认 30
camera_config_path=    # 空则自动查找源码树/包内标定
record=false           # true 时用 ros2 bag record 录传感器
record_path=           # 空则 /tmp/flyrat_bags/<时间戳>
record_topics=/flyrat/sensors/imu /flyrat/sensors/stereo/compressed
record_imu_only=false  # true 时只录 IMU（Allan 方差）
record_storage=mcap
record_duration=0      # 秒数或 30m/3h；>0 时倒计时并自动停止录制
record_warmup=8        # 先等设备启动，再开始倒计时
start_foxglove=false
foxglove_port=8765
stereo_color=false
imu_accel_axis=none
imu_gyro_axis=none
trajectory_path=
```

`start_foxglove:=true` 需要安装 `ros-jazzy-foxglove-bridge`；否则保持默认，用
`.script/foxglove.sh` 单独启动。

录制由 `.script/bringup_real.sh` 启动，和 `ros2 launch` 平级，按 `Ctrl-C` 时
脚本会给 `ros2 bag record` 发 `SIGTERM` 并等待它写完 MCAP。只录 IMU 做 Allan
方差：

```sh
.script/bringup_real.sh start_odometry:=false record:=true record_imu_only:=true \
  record_duration:=3h
```

录双目 + IMU：

```sh
.script/bringup_real.sh start_odometry:=false record:=true
```

默认录 `/flyrat/sensors/stereo/compressed`（左右拼接的原始 MJPEG）而不是两路
独立 raw 图像，并把 `camera_fps` 自动降到 30。bag 回放时用 splitter 生成左右
话题；做 Kalibr 时再限速到 20 Hz：

```sh
ros2 bag play /tmp/flyrat_bags/<时间戳>
ros2 run flyrat_sensors stereo_splitter_node --ros-args -p max_rate:=20.0
ros2 bag record -o /tmp/flyrat_bags/kalibr /flyrat/sensors/stereo/left/image_raw \
  /flyrat/sensors/stereo/right/image_raw /flyrat/sensors/imu
```

`stereo_splitter_node` 用同一条压缩消息的 header stamp 同时发布左右图，输出为
reliable QoS；它只用于回放，不要与实时 `sensor_receiver_node` 同时运行。

默认写到 `/tmp/flyrat_bags/<时间戳>`，用 `record_path:=/path/to/bag` 指定；
`record_duration` 结束后自动给录制进程发 SIGTERM 并写完 MCAP；录制停止后
bringup 仍在运行，按 `Ctrl-C` 结束。`ros2 bag info /path/to/bag` 可检查消息数和
频率。`record:=true` 不会传给 `bringup.launch.py`，它只是脚本参数。

## 8. Docker 与 CI

拉取预构建镜像：

```sh
docker pull mide233/flyrat-dev-full:latest
```

在容器中启动：

```sh
docker compose up -d rat-dev-full
docker exec -it rat-dev-full bash
```

本机构建镜像、代理参数、GitHub Actions 工作流和镜像发布方式见
[`development/docker_image.md`](development/docker_image.md)。

## 9. 清理

```sh
.script/clean_build.sh --dry-run    # 先看会删什么
.script/clean_build.sh              # 清理全部构建产物
.script/clean_build.sh --keep-sim   # 保留 SITL 与 Gazebo 插件
```

不会删除源码、子模块和 `flyrat_play/gazebo/config/eeprom.bin`。

## 10. 产物路径

| 产物 | 路径 |
|---|---|
| 核心库 | `libflyrat/build/libflyrat*.a` |
| 核心测试 | `libflyrat/build/test_*` |
| 硬件工具 | `libflyrat/build/{tof_bench,stereo_bench,msp_interact_bench}` |
| 实机主程序 | `flyrat_pilot/build/flyrat_pilot` |
| 传感器转发 | `flyrat_pilot/build/flyrat_sensor_forwarder` |
| SITL | `flyrat_play/gazebo/third_party/betaflight/obj/main/betaflight_SITL.elf` |
| Gazebo 插件 | `.../aeroloop_gazebo/plugins/build/libBetaflightPlugin.so` |
| ROS 2 工作区 | `flyrat_play/ros2_ws/{build,install,log}` |

## 11. 常见问题

| 现象 | 处理 |
|---|---|
| `BASH_SOURCE: parameter not set` | 用 `bash -lc` source ROS 2，不要用 zsh |
| 找不到 OSQP | 安装 `libosqp-dev`，或提供 `osqpConfig.cmake` |
| `gz-sim8` 找不到 | 安装 Gazebo Harmonic 开发包，确认 `pkg-config --exists gz-sim8` |
| SITL ELF 不存在 | 先 `git submodule update --init` 再 `.script/build_betaflight.sh` |
| 插件未构建 | 子模块未初始化，执行 `git submodule update --init --recursive` |
| `adb: no devices` | 检查 USB、`adb devices`，确认 serial |
| ROS 收不到数据 | 先确认 `adb forward --list`，再确认设备 forwarder 已监听 5000 |
| ToF 全部 `distance_mm=0` | 确认 C/C++ 没有单独传 `VL53L5CX_DISABLE_*`，重新完整编译 |
| ToF 初始化失败 | 确认四颗 LPn/INT 接线和 i2c2，检查地址是否被其他器件占用 |
| 里程计未输出位姿 | 先看 `/flyrat/odometry/status`；确认 `mode:=imu_stereo`、IMU 在发布、`imu_lookahead` 为正，且图像尺寸匹配 640x480 |
| 里程计数秒后才输出、轨迹漂移数米 | 场景距离超出配置的深度范围（0.25–20 m），初始化一直失败；按实际距离调整 `stereo.*_depth_inv` 和 `initialization.*` |
| 里程计配置错误 | 检查 `camera_config_path` 指向的 YAML 是否存在、`T_body_camera` 是否为合法等距变换 |
