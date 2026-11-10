# 编译与运行

本文汇总仓库内所有组件的构建、测试和运行方式。各模块的设计、接口和实测数据见
对应主题文档：

- 实机传感器转发：[`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
- Gazebo + Betaflight SITL：[`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
- 里程计骨架：[`odometry/overview.md`](odometry/overview.md)
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
- 泰山派（RK3566，Buildroot）没有 OpenCV，内存约 2 GB；ROS 2 里程计节点目前
  只在开发机运行，后端算法尚未实现。
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

共 9 个测试；其中 `test_odometry_api` 覆盖里程计接口、参数校验和当前桩行为。

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
  --resolution 8 --hz 15 --sensors 4
```

可选 `--interrupts`、`--lpn`、`--int`、`--poll-ms`。ToF 地址固定为
`0x60/0x62/0x64/0x66`，不支持通过参数修改。

`stereo_bench`：旧双目采集工具；当前实机已改为单目，仅保留给旧硬件。

```sh
./libflyrat/build/stereo_bench --device /dev/video10 \
  --width 1280 --height 480 --fps 60 --seconds 5 --mjpeg
```

`--gray` 输出 GRAY8，`--mjpeg` 使用 MPP 硬解，`--save-nv12` 保存原始 NV12，
`--save DIR` 保存左右图。

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

运行主程序（目前仍是占位逻辑）：

```sh
./flyrat_pilot/build/flyrat_pilot
```

运行传感器转发：

```sh
./flyrat_pilot/build/flyrat_sensor_forwarder --listen 127.0.0.1:5000
```

完整参数：

```text
--listen ADDR:PORT      默认 127.0.0.1:5000
--msp-device PATH       默认 /dev/ttyS3
--msp-baud N            默认 921600
--camera-device PATH    默认 /dev/video10
--camera-width N        默认 640
--camera-height N       默认 480
--camera-fps N          默认 60
--tof-resolution N      默认 8（4 或 8）
--tof-rate HZ           默认 15（4x4 最高 60，8x8 最高 15）
--no-imu --no-tof --no-camera
--help
```

### 4.2 交叉编译到泰山派

`flyrat_pilot` 的 CMake 会链接依赖 Eigen/OSQP 的 `flyrat::flyrat`，目前没有为
aarch64 提供完整 toolchain。传感器转发程序用脚本交叉编译：

```sh
.script/build_sensor_forwarder_arm.sh
```

产物是 `build-arm/flyrat_sensor_forwarder`（AArch64 静态链接）。脚本参数：

```text
--clean                 先删除 build-arm/
CROSS_COMPILE=...       工具链前缀，默认 aarch64-linux-gnu-
BUILD_DIR=...           输出目录
```

编译 C 和 C++ 时不要额外传 `VL53L5CX_DISABLE_*`；这些宏统一定义在
`libflyrat/third_party/vl53l5cx/platform/platform.h`，只给 C 文件传会导致结果
结构体布局不一致、距离全 0。

手动推送：

```sh
adb -s 4c24d56142f3d0e3 push build-arm/flyrat_sensor_forwarder /tmp/
adb -s 4c24d56142f3d0e3 shell chmod +x /tmp/flyrat_sensor_forwarder
```

`.script/bringup_real.sh` 每次启动都会自动把 `build-arm/flyrat_sensor_forwarder`
推到设备，因此重启后 `/tmp` 被清空也不需要手动处理。设备上缺少二进制且本地也
没有编译产物时，脚本会直接报错并提示先运行上面的交叉编译命令。

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
/flyrat/sensors/camera/image_raw
```

单目相机发布 `mono8`，默认 640x480；`camera_frame_id` 默认为
`camera_front_optical_frame`。

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

`flyrat_odometry` 的节点会链接 libflyrat 的里程计骨架，并使用 OpenCV 转换
图像消息和解析标定文件。

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
| `flyrat_odometry` | 默认 TIO-Former ToF-惯性节点；旧立体骨架保留为 legacy |
| `flyrat_msp` | MSP 协议接入 |
| `flyrat_control` | 控制节点与 SITL launch |

`sensors.launch.py` 参数：`host`（默认 127.0.0.1）、`port`（默认 5000）、
`acc_lsb_per_g`、`gyro_lsb_per_dps`、`imu_accel_axis`（默认 `none`）、
`imu_gyro_axis`（默认 `none`）。当前实机加速度和陀螺仪原始数据
都已是 ROS FLU，实测左偏航 `angular_velocity.z > 0`。只有偏航为顺时针正的
飞控才需要 `bf_to_flu`（翻 z）；真正的 FRD 数据源用 `frd_to_flu`。

启动时还会加载 `flyrat_sensors/config/imu_intrinsics.yaml`：打开
`imu_intrinsics_enable` 后，节点按 Kalibr 的 scale/misalignment 模型修正
加速度和陀螺仪，并从 IMU 时间戳中扣除 `imu_time_offset`（相机到 IMU 的
固定时间差，约 3.8 ms）。YAML 中 `T_i_b` 为单位阵；零偏不写入配置，由
里程计后端在线估计（待实现）。关闭该功能后发布的是未修正的原始刻度数据。

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

当前 `bringup_real.sh` 默认启动 TIO-Former ToF-IMU-相机后端
`tio_odometry_node`（4×8×8 ToF + 200 Hz IMU + 160×120 mono8 相机，需要
libtorch，见 7.6）；`tio_camera:=false` 回退 ToF+IMU 模型。
`libflyrat` 同时保留不依赖 OpenCV 的旧立体接口 `flyrat::odometry` 和
`odometry_node`，但它已是 legacy 骨架：只做输入校验、静态 TF 和状态发布，
不输出有效位姿。TIO 细节见 [`odometry/tio_former.md`](odometry/tio_former.md)，
旧接口见 [`odometry/overview.md`](odometry/overview.md)。

### 7.1 构建与测试

`flyrat_odometry` 目标默认随 libflyrat 构建：

```sh
.script/build_libflyrat.sh
ctest --test-dir libflyrat/build --output-on-failure
```

测试共 9 个，其中 `test_odometry_api` 覆盖状态枚举、参数校验、空图像/尺寸
不匹配拒绝、桩返回值、重复 `shutdown` 和未实现轨迹保存。

ROS 2 节点：

```sh
cd flyrat_play/ros2_ws
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select flyrat_sensors flyrat_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

节点使用 OpenCV 将 ROS 图像消息转换为 `ImageView`，并用 `cv::FileStorage`
读取标定文件。

### 7.2 legacy 标定配置

`libflyrat/configs/camera/uvc_640x480.yaml` 保存旧 DECXIN 双目 UVC 相机和
Betaflight IMU 的实机标定；当前单目相机不使用该文件：

- `camera.left` / `camera.right`：640x480 每目内参和 plumb-bob 畸变；
- `camera.T_left_right`：左目 ← 右目，`p_left = T_left_right * p_right`；
- `imu.T_body_camera`：机体/IMU ← 相机，`p_body = T_body_camera * p_camera`；
- `imu.*`：200 Hz、Allan 噪声和随机游走。

数值来自 `kalibr_ws/`，不要随意改写。ROS 包安装时会把该文件复制到
`share/flyrat_odometry/config/`，也可以通过 `camera_config_path` 指向源码树
或其他文件。

### 7.3 legacy C++ API

头文件 `libflyrat/include/flyrat/odometry/odometry.hpp`：

```cpp
flyrat::odometry::Options options;
options.mode = flyrat::odometry::Mode::kImuStereo;
options.camera_config_path = ".../uvc_640x480.yaml";

flyrat::odometry::Odometry odometry(options);
auto result = odometry.track(left, right, timestamp, imu_samples);
```

`ImageView` 是非拥有型的 8 位灰度视图，因此核心库不依赖 OpenCV。实现桩在
`libflyrat/src/odometry/odometry.cpp`，用 TODO 标出了后端接入位置；`track()`
当前只做输入校验并返回 `State::kNotInitialized`。

### 7.4 legacy 立体 ROS 2 节点

先按第 4.3 节启动设备转发和 `flyrat_sensors`，再运行：

```sh
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /workspaces/flyrat/flyrat_play/ros2_ws/install/setup.bash && \
  ros2 launch flyrat_odometry odometry.launch.py'
```

该节点只用于旧立体路线；默认 bringup 不会启动它。`camera_config_path` 默认
使用包内安装的 `uvc_640x480.yaml`；`body_frame:=imu_link` 是默认机体坐标系；
`world_frame` 默认 `odometry_map`。节点会发布静态 `imu_link -> camera_left_optical_frame` 和
`camera_left_optical_frame -> camera_right_optical_frame`（来自
`camera.T_left_right`）。其余参数见 `flyrat_odometry/config/odometry.yaml`。

纯双目启动：

```sh
ros2 launch flyrat_odometry odometry.launch.py mode:=stereo
```

`mode:=stereo` 时节点不订阅 IMU；标定文件中的 IMU 参数不会被使用，但
`imu.T_body_camera` 仍用于静态 TF 和机体位姿换算。

后端实现后，节点会发布动态 `odometry_map -> imu_link`；当前骨架没有有效
位姿，因此只有静态 TF。

输出：

```text
/flyrat/odometry/odometry   nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path       nav_msgs/Path，机体轨迹
/flyrat/odometry/status     diagnostic_msgs/DiagnosticStatus，1 Hz
/flyrat/odometry/reset      std_srvs/Trigger
/tf                         静态相机 TF（后端实现后增加动态机体 TF）
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

### 7.5 一键 bringup

`bringup.launch.py` 把 adb forward、设备端 forwarder、`sensor_receiver_node` 和
选定的里程计后端串成一条启动链：

```sh
.script/bringup_real.sh                         # 传感器 + TIO 相机后端
.script/bringup_real.sh tio_camera:=false       # ToF+IMU 模型回退
.script/bringup_real.sh odometry_backend:=stereo # 旧立体骨架
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
- 1 秒后启动默认的 `tio_odometry_node`；
- 使用脚本时，`Ctrl-C` 会停止设备端 forwarder 并删除 adb forward；直接
  `ros2 launch` 不会做这两步清理。

设备端默认相机为 `/dev/video10`、640x480 MJPEG（请求 60 Hz，主机实测约
30 Hz）；可用 `camera_device`、
`camera_width`、`camera_height`、`camera_fps` 覆盖。旧立体路线的
`camera_config_path` 只有 `odometry_backend:=stereo` 时才需要。

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
odometry_backend=tio   # tio（默认）或 stereo（legacy）
camera_device=/dev/video10
camera_width=640
camera_height=480
camera_fps=60          # 请求值；主机实测约 30 Hz
record=false           # true 时用 ros2 bag record 录传感器
record_path=           # 空则 /tmp/flyrat_bags/<时间戳>
record_topics=/flyrat/sensors/imu /flyrat/sensors/camera/image_raw /flyrat/sensors/tof_0 /flyrat/sensors/tof_1 /flyrat/sensors/tof_2 /flyrat/sensors/tof_3
record_imu_only=false  # true 时只录 IMU（Allan 方差）
record_storage=mcap
record_duration=0      # 秒数或 30m/3h；>0 时倒计时并自动停止录制
record_warmup=8        # 先等设备启动，再开始倒计时
start_foxglove=false
foxglove_port=8765
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

录单目 + IMU，相机降到 20 Hz：

```sh
.script/bringup_real.sh start_odometry:=false camera_fps:=20 record:=true
```

默认会录 IMU、相机和 4 路 ToF；TIO-Former 的回放必须包含
`/flyrat/sensors/tof_0..3`，否则只有 IMU/相机，无法复现里程计。默认写到
`/tmp/flyrat_bags/<时间戳>`，用 `record_path:=/path/to/bag` 指定；
`record_duration` 结束后自动给录制进程发 SIGTERM 并写完 MCAP；录制停止后
bringup 仍在运行，按 `Ctrl-C` 结束。`ros2 bag info /path/to/bag` 可检查消息数和
频率。`record:=true` 不会传给 `bringup.launch.py`，它只是脚本参数。

### 7.6 TIO-Former ToF 后端（实验）

TIO-Former 模型训练使用前/左/右/下 4 颗 8×8 ToF 和 200 Hz IMU，模型从
`research/tio_former/` 导出为 TorchScript。默认构建不会编译它；需要 CPU
libtorch：

```sh
cd research/tio_former
python3 -m venv .venv-cpu
.venv-cpu/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
cd ../..
TORCH_CMAKE="$PWD/research/tio_former/.venv-cpu/lib/python3.12/site-packages/torch/share/cmake"
```

libflyrat 后端与基准：

```sh
cmake -S libflyrat -B libflyrat/build-tio \
  -DFLYRAT_BUILD_TIO=ON -DFLYRAT_BUILD_TESTS=OFF -DFLYRAT_BUILD_TOOLS=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$TORCH_CMAKE"
cmake --build libflyrat/build-tio -j"$(nproc)"
ctest --test-dir libflyrat/build-tio --output-on-failure -R tio
libflyrat/build-tio/tio_bench libflyrat/models/tio_former_window64.pt 40
libflyrat/build-tio/tio_bench libflyrat/models/tio_former_window64_camera.pt 80 camera
```

ROS 节点：

```sh
bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd flyrat_play/ros2_ws
  colcon build --symlink-install \
    --packages-select flyrat_sensors_interfaces flyrat_sensors flyrat_odometry \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DFLYRAT_BUILD_TIO=ON \
      -DCMAKE_PREFIX_PATH='"$TORCH_CMAKE"'
  source install/setup.bash
  ros2 launch flyrat_odometry tio_odometry.launch.py'
```

默认一键 bringup 已经是 TIO：

```sh
.script/bringup_real.sh
```

它和旧立体节点使用同一组 `/flyrat/odometry/*` 话题与
`odometry_map -> imu_link` TF，因此 Foxglove 配置不需要改。节点需要 64 帧
（约 4.3 秒）ToF 才输出第一个有效位姿。参数名是 `start_odometry`（默认
`true`），不是旧的 `start_slam`。

2026-11-08 旧布局真机验证结果：4 颗 ToF 设备端实测 14.8–15.2 Hz，TIO
节点能进入 `TRACKING` 并发布 odometry/TF；但无人机静止放桌面 20 s 漂移约
0.98 m，主要受下视贴桌面、旧映射中 front 槽位接后向数据以及仿真域差异影响。
当前不能用于飞行闭环。机头重定义后新映射尚未做净空静止/飞行验证。完整记录见
[`odometry/handoff.md`](odometry/handoff.md)。

实机 ToF 默认配置已改为 **8×8 @ 15 Hz**。`flyrat_sensor_forwarder` 的
`--tof-resolution` / `--tof-rate` 可覆盖，8×8 最高 15 Hz。机头在原来后向
ToF 一端，当前 INT/LPn 对应关系为：传感器 0=下、1=前、2=右、3=左；TIO
节点配置 `sensor_order: [1,3,2,0]`，即模型 front/left/right/down ← 1/3/2/0。
ToF 视角顺序、`sensor_order`、参数和限制见
[`odometry/tio_former.md`](odometry/tio_former.md)。

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
| 里程计未输出位姿 | 默认 TIO 需等 64 帧 ToF；旧 `stereo` 骨架只会发布 `NOT_INITIALIZED` |
| 相机无图像 | 确认设备 `/dev/video10` 存在，且 bringup 未传 `--no-camera` |
| ToF 方向不对 | 检查 `sensor_order: [1,3,2,0]`（前/左/右/下 ← 1/3/2/0） |
| 里程计配置错误 | 检查 `camera_config_path` 指向的 YAML 是否存在、`T_body_camera` 是否为合法等距变换 |
