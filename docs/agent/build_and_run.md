# 编译与运行

本文汇总仓库内所有组件的构建、测试和运行方式。各模块的设计、接口和实测数据见
对应主题文档：

- 实机传感器转发：[`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
- Gazebo + Betaflight SITL：[`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
- ORB-SLAM3：[`slam/orb_slam3.md`](slam/orb_slam3.md)
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
| ORB-SLAM3 | OpenCV 4（core/imgproc/features2d/calib3d/imgcodecs/videoio）、Boost.Serialization、OpenSSL |
| 实机交叉编译 | `aarch64-linux-gnu-gcc/g++`、目标设备 `adb` |
| 开发容器 | Docker、Docker Compose、可选 buildx |

Ubuntu 上安装基础依赖：

```sh
sudo apt install build-essential cmake pkg-config libeigen3-dev libosqp-dev
```

### 1.2 环境限制

- ROS 2 的 `setup.bash` 必须在 `bash` 中 source。zsh 下直接 source 会因
  `BASH_SOURCE` 未定义而失败；使用 `bash -lc '...'`。
- 泰山派（RK3566，Buildroot）没有 OpenCV，内存约 2 GB，ORB-SLAM3 只在开发机
  构建和运行。
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
| `FLYRAT_BUILD_SLAM` | OFF | 构建 ORB-SLAM3 与 `slam_bench` |
| `FLYRAT_ORB_VOCAB` | 空 | ORB 词袋路径，供测试使用 |

找不到 OSQP 时配置会直接报错，需要安装 `libosqp-dev` 或提供
`osqpConfig.cmake`。

### 3.2 测试

```sh
ctest --test-dir libflyrat/build --output-on-failure
```

不启用 SLAM 时为 8 个测试；启用 SLAM 后增加 `test_slam_api`，共 9 个。

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

`stereo_bench`：双目相机采集。

```sh
./libflyrat/build/stereo_bench --device /dev/video10 \
  --width 1280 --height 480 --fps 60 --seconds 5 --mjpeg
```

`--gray` 输出 GRAY8，`--mjpeg` 使用 MPP 硬解，`--save-nv12` 保存原始 NV12，
`--save DIR` 保存左右图。

`slam_bench`：ORB-SLAM3 回放与实时测试，构建方式见第 7 节。

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
--camera-width N        默认 1280（左右拼接总宽）
--camera-height N       默认 480
--camera-fps N          默认 60
--no-imu --no-tof --no-stereo
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
/flyrat/sensors/stereo/left/image_raw
/flyrat/sensors/stereo/right/image_raw
```

默认 `stereo_color=false`，发布 mono8；需要彩色时设置
`stereo_color:=true`，发布 bgr8。

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

`flyrat_slam` 在配置时会强制 `FLYRAT_BUILD_SLAM=ON`，因此完整构建 ROS 2 包
还需要 OpenCV、Boost.Serialization 和 OpenSSL。

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
| `flyrat_slam` | ORB-SLAM3 IMU-stereo 节点、轨迹、TF 和状态 |
| `flyrat_msp` | MSP 协议接入 |
| `flyrat_control` | 控制节点与 SITL launch |

`sensors.launch.py` 参数：`host`（默认 127.0.0.1）、`port`（默认 5000）、
`stereo_color`、`acc_lsb_per_g`、`gyro_lsb_per_dps`、`imu_accel_axis`（默认
`none`）、`imu_gyro_axis`（默认 `none`）。当前实机加速度和陀螺仪原始数据
都已是 ROS FLU，实测左偏航 `angular_velocity.z > 0`。只有偏航为顺时针正的
飞控才需要 `bf_to_flu`（翻 z）；真正的 FRD 数据源用 `frd_to_flu`。

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

## 7. ORB-SLAM3

### 7.1 依赖与构建

```sh
sudo apt install libopencv-dev libboost-serialization-dev libssl-dev

cmake -S libflyrat -B libflyrat/build-slam \
  -DFLYRAT_BUILD_SLAM=ON \
  -DFLYRAT_ORB_VOCAB=/path/to/ORBvoc.txt \
  -DCMAKE_BUILD_TYPE=Release
cmake --build libflyrat/build-slam -j
```

`FLYRAT_ORB_VOCAB` 可省略，运行时通过 `Options::vocabulary_path` 或
`slam_bench --vocab` 指定。词袋文件不入库。

### 7.2 测试

```sh
ctest --test-dir libflyrat/build-slam --output-on-failure
```

配置了 `FLYRAT_ORB_VOCAB` 时，`test_slam_api` 会实际加载词袋并构造
`ORB_SLAM3::System`；未配置时只验证参数校验和状态枚举。

### 7.3 slam_bench

```sh
./libflyrat/build-slam/slam_bench \
  --vocab /path/to/ORBvoc.txt \
  --settings libflyrat/configs/orb_slam3/uvc_640x480.yaml \
  --left /path/to/left --right /path/to/right \
  --fps 30 --save-traj /tmp/traj.txt
```

也支持左右拼接视频 `--video FILE` 和实时 V4L2 `--sbs /dev/video0`。
IMU 模式加 `--mode imu-stereo`。配置模板里的内参、基线、IMU 外参均为占位值，
正式使用前必须标定。

接口与限制见 [`slam/orb_slam3.md`](slam/orb_slam3.md)。

### 7.4 ROS 2 SLAM 节点

`flyrat_slam` 把实机转发的双目和 IMU 直接接入 `flyrat::slam::StereoSlam`，
默认使用 IMU-stereo 模式，也可以用 `mode:=stereo` 只跑纯双目。先按第 4.3 节
启动设备转发和 `flyrat_sensors`。

构建：

```sh
cd flyrat_play/ros2_ws
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select flyrat_sensors flyrat_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

启动：

```sh
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /workspaces/flyrat/flyrat_play/ros2_ws/install/setup.bash && \
  ros2 launch flyrat_slam slam.launch.py \
  vocabulary_path:=/workspaces/flyrat/playground/ORB_SLAM3/Vocabulary/ORBvoc.txt'
```

`vocabulary_path` 必填，词袋文件不入库。`settings_path` 默认是包内安装的
`uvc_640x480_imu.yaml`；`body_frame:=imu_link` 是默认机体坐标系；
`trajectory_path:=/tmp/traj.txt` 可在退出时保存 TUM 轨迹。节点会发布动态
`slam_map -> imu_link`、静态 `imu_link -> camera_left_optical_frame`，以及
`camera_left_optical_frame -> camera_right_optical_frame`（来自
`Stereo.T_c1_c2`）。其余参数见 `flyrat_slam/config/slam.yaml`。

纯双目启动：

```sh
ros2 launch flyrat_slam slam.launch.py mode:=stereo \
  vocabulary_path:=/workspaces/flyrat/playground/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

`mode:=stereo` 时节点不订阅 IMU，ORB-SLAM3 使用 `Stereo` 模式。默认的
`uvc_640x480_imu.yaml` 仍可用，IMU 噪声参数会被忽略；`IMU.T_b_c1` 仍用于
静态 TF 和机体位姿。纯双目下 `slam_map` 会重定基到第一帧的 `imu_link`
位姿，即 FLU（x 前、y 左、z 上），所以 Foxglove 里前进对应 +x、左移 +y、
上升 +z。

Foxglove 中建议把固定坐标系设为 `slam_map` 或 `imu_link`。`camera_left_optical_frame`
是光学系，占位外参是倒置、光轴正对机体前方；直接看它的轴时，机体滚转对应
光学系的偏航，这是坐标系分解，不是 SLAM 错误。

输出：

```text
/flyrat/slam/odometry   nav_msgs/Odometry，slam_map -> imu_link
/flyrat/slam/path       nav_msgs/Path，机体轨迹
/flyrat/slam/status     diagnostic_msgs/DiagnosticStatus，1 Hz
/flyrat/slam/reset      std_srvs/Trigger
/tf                     slam_map -> camera_left_optical_frame
```

Foxglove 预览：

```sh
.script/foxglove.sh
```

默认 WebSocket 是 `ws://localhost:8765`。在 Foxglove 中打开该连接，添加 3D 面板
显示 `/tf`、`/flyrat/slam/path` 和 `/flyrat/slam/odometry`，再添加两个 Image
面板显示左右目话题，以及 Diagnostic 面板查看 `/flyrat/slam/status`。布局文件
不入库，按需在本地保存。

当前配置的内参、双目外参和 `IMU.T_b_c1` 都是占位值；静止桌面场景不足以稳定
初始化，需要实际运动和有纹理的视场验证跟踪。

### 7.5 一键 bringup

`bringup.launch.py` 把 adb forward、设备端 forwarder、`sensor_receiver_node` 和
`slam_node` 串成一条启动链，不再需要多个终端：

```sh
.script/bringup_real.sh                        # 传感器 + SLAM
.script/bringup_real.sh mode:=stereo            # 纯双目，不订阅 IMU
.script/bringup_real.sh start_foxglove:=true   # 再加 Foxglove bridge
.script/bringup_real.sh start_slam:=false      # 只看传感器
.script/bringup_real.sh start_adb_forward:=false start_device:=false  # 复用已有转发
```

脚本会 source ROS 2 和 `flyrat_play/ros2_ws/install/setup.bash`，然后执行：

```sh
ros2 launch flyrat_slam bringup.launch.py
```

也可以直接运行上面的 `ros2 launch`。默认行为：

- 把 `build-arm/flyrat_sensor_forwarder` 推到设备 `/tmp` 并加执行权限
  （设置 `FLYRAT_PUSH_FORWARDER=0` 可跳过）；
- 建立 `adb forward tcp:5000 tcp:5000`；
- 先结束设备上旧的 `flyrat_sensor_forwarder`，再启动
  `/tmp/flyrat_sensor_forwarder --listen 127.0.0.1:5000`；
- 设备 forwarder 意外退出时整个 launch 停止，不会让接收节点无限重连；
- 1 秒后启动 `sensor_receiver_node`（ToF 初始化约 7 秒，接收节点会自动重连）；
- 1 秒后启动 `slam_node`；
- 使用脚本时，`Ctrl-C` 会停止设备端 forwarder 并删除 adb forward；直接
  `ros2 launch` 不会做这两步清理。

词袋默认从 `FLYRAT_ORB_VOCAB` 或
`playground/ORB_SLAM3/Vocabulary/ORBvoc.txt` 查找；`settings_path` 默认优先使用
源码树的 `libflyrat/configs/orb_slam3/uvc_640x480_imu.yaml`，所以改外参后不需要
重新 `colcon build`。找不到词袋时用 `vocabulary_path:=...` 显式指定。

常用参数：

```text
adb_serial=            # 多设备时指定；默认用唯一连接的设备
device_binary=/tmp/flyrat_sensor_forwarder
local_port=5000
device_port=5000
start_device=true
start_adb_forward=true
start_sensors=true
start_slam=true
mode=imu_stereo        # 纯双目用 mode:=stereo
camera_fps=60          # 标定录制用 20
record=false           # true 时用 ros2 bag record 录传感器
record_path=           # 空则 /tmp/flyrat_bags/<时间戳>
record_topics=/flyrat/sensors/imu /flyrat/sensors/stereo/left/image_raw /flyrat/sensors/stereo/right/image_raw
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
.script/bringup_real.sh start_slam:=false record:=true record_imu_only:=true \
  record_duration:=3h
```

录双目 + IMU 做 Kalibr，相机降到 20 Hz：

```sh
.script/bringup_real.sh start_slam:=false camera_fps:=20 record:=true
```

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
| SLAM 库 | `libflyrat/build-slam/libflyrat_slam.a` |
| SLAM 工具 | `libflyrat/build-slam/slam_bench` |
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
| SLAM 报词袋缺失 | 用 `--vocab` 或 `FLYRAT_ORB_VOCAB` 指向 `ORBvoc.txt` |
| SLAM 配置错误 | 检查 `--settings` 的 YAML，当前实现会抛异常而不是退出进程 |
