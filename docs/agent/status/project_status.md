# 项目状态

最后更新：2026-11-11

## 1. 项目目标

flyrat 是一个面向 Betaflight 飞控的四旋翼自驾栈：

- `libflyrat/`：不依赖 ROS 的 C++17 核心库，包含 MSP 通信、IMU 读取、姿态控制、
  MPC、状态估计和硬件抽象。
- `flyrat_pilot/`：实机端程序。已支持机载 SchurVINS 和位姿回传；飞控主逻辑仍在实现中。
- `flyrat_play/`：Gazebo + Betaflight SITL 仿真与 ROS 2 适配层。
- `docs/`：构建、仿真、MSP 链路和传感器转发文档。

当前重点是把里程计从开发机搬到实机：RK3566 本地运行 SchurVINS，通过 TCP 回传
位姿，主机只负责可视化与录包；传感器转发路径继续保留用于调试和录包。

## 2. 已完成模块

### 2.1 核心库

- MSP v1 异步客户端 `flyrat::MspInteract`：TCP/串口、轮询、重连、回调与最新快照。
- `flyrat::hardware::Transport`：串口和 TCP 字节流抽象，单测可注入假传输。
- 姿态保持 `AttitudeHold`、on-manifold MPC `Ommpc`、`LocalEstimator`、`RcReceiver`。
- 单元测试 10 个，其中 `test_odometry_api` 覆盖里程计接口契约，`test_schurvins_integration` 覆盖 SchurVINS 初始化、跟踪与轨迹输出。

### 2.2 实机硬件

- `hardware::TofArray`：4 颗 VL53L5CX 共用 i2c2，4x4@60 Hz，裁剪结果块 128 B/帧。
  - 地址固定为 `0x60/0x62/0x64/0x66`，不再支持单独配置。
  - 每次启动逐颗释放 LPn，自动探测残留地址并改回固定地址。
  - 不做任何 I2C 访问前先把四路 LPn 拉低，避免上电都处于 0x52 时冲突。
  - 输出裁剪宏统一定义在 `platform/platform.h`，保证 C 驱动和 C++ 读取端
    的 `VL53L5CX_ResultsData` 布局一致。
- `hardware::Imu`：读取 `MspInteract` 的最新 `MSP_RAW_IMU` 快照。
- `hardware::StereoCamera`：UVC 双目相机 V4L2 采集。
  - `kYuyv`：原始 YUYV，可输出 YUYV 或 GRAY8。
  - `kMjpeg`：Rockchip MPP 硬解，输出 GRAY8 或 NV12。
- `hardware::StereoJpegSource`：不解码的原始 MJPEG 采集，供 adb 转发使用。

### 2.3 实机传感器转发

完整启动、参数和话题说明见
[`hardware/sensor_forwarding.md`](../hardware/sensor_forwarding.md)。

设备端 `flyrat_sensor_forwarder`：

- IMU：`MspInteract`，`MSP_RAW_IMU` 200 Hz。
- ToF：4 颗 VL53L5CX，每颗约 57–58 Hz。
- 双目：相机原生 MJPEG，整幅左右拼接帧，不在设备端解码。
- TCP 服务：`127.0.0.1:5000`，单客户端；双目队列满时丢旧帧，IMU/ToF 不阻塞。

开发机 ROS 2：

- `flyrat_sensors_interfaces`：`TofZone` / `TofFrame` 自定义消息。
- `flyrat_sensors`：`sensor_receiver_node`，负责 TCP 接收、协议解析、图像解码和发布。
  IMU 默认 `imu_accel_axis=none`、`imu_gyro_axis=none`：加速度和陀螺仪原始
  数据都已是 ROS FLU（实测左偏航 `angular_velocity.z > 0`）。
- 默认话题：
  - `/flyrat/sensors/imu`
  - `/flyrat/sensors/tof_0` … `/flyrat/sensors/tof_3`
  - `/flyrat/sensors/stereo/left/image_raw`
  - `/flyrat/sensors/stereo/right/image_raw`

### 2.4 里程计后端

完整接口、配置和接入方式见
[`odometry/overview.md`](../odometry/overview.md)。

- 默认后端为 **SchurVINS**（SVO 2.0 前端 + Schur complement EKF，CVPR 2024），
  源码内置在 `libflyrat/third_party/schurvins/`，许可证 **GPL-3.0**。
- `flyrat::odometry::Odometry`：ROS 无关的 C++17 接口，启用 SchurVINS 时仅支持
  `kImuStereo`；状态为 `kNotInitialized/kTracking/kRecentlyLost/kLost`。
- 输入使用非拥有型 `ImageView`（8 位灰度）和 `ImuSample` 序列，因此接口头文件
  不依赖 OpenCV；适配层和第三方代码依赖 OpenCV。
- 适配层 `libflyrat/src/odometry/schurvins_backend.cpp` 解析 Kalibr 标定、构造
  `svo::FrameHandlerStereo`、喂入 IMU、输出 `T_world_camera`。
- `CMake` 选项 `FLYRAT_WITH_SCHURVINS` 默认 `ON`；关闭后退化为桩实现，
  `track()` 返回 `kNotInitialized`，`saveTrajectory()` 抛异常。
- 标定文件 `libflyrat/configs/camera/uvc_640x480.yaml` 保留实机 Kalibr 数值
  （双目内参/畸变、`camera.T_left_right`、`imu.T_body_camera`、IMU 噪声）。
- 会导致进程退出的 IMU 时序已处理：只有帧时间戳落在 IMU 样本区间内才调用前端，
  ROS 节点提供 `imu_lookahead` 帧后样本。
- `test_odometry_api` 覆盖接口契约，`test_schurvins_integration` 覆盖初始化、
  跟踪和轨迹输出；10 个测试全部通过。
- 实机 28 s 桌面 bag（静止 → 升高 35 cm → 回原位）回放验证：首帧开始跟踪，
  z 轨迹 0.008 → 0.364 → 0.016 m，最大水平偏移约 2.5 cm，结束时漂移约
  1.4 cm，输出约 39–45 Hz。
- 深度范围已按近场调整为 0.25–20 m，初始化阈值为 40/40/30/10 px；上游默认
  1–50 m 和 60/80/70/30 在 0.4–1 m 桌面场景会初始化失败并导致纯 IMU 漂移。

### 2.5 ROS 2 里程计节点

坐标链和标定调整见 [`odometry/coordinates.md`](../odometry/coordinates.md)。

- `flyrat_play/ros2_ws/src/flyrat_odometry/`：双目/IMU 同步、IMU 缓冲、worker
  线程、TF、轨迹和状态，后端为 SchurVINS。
- 订阅实机转发的左右目和 IMU，双目用 `ApproximateTime` 同步，IMU 缓存约 1 s；
  每帧取上一帧到 `帧时间 + imu_lookahead` 的 IMU，首帧带约 100 ms 历史。
  跟踪线程忙时丢旧帧。
- 发布 `/flyrat/odometry/odometry`、`/flyrat/odometry/path`、
  `/flyrat/odometry/status`、`/flyrat/odometry/reset`；静态 TF
  `imu_link -> camera_left_optical_frame` 和
  `camera_left_optical_frame -> camera_right_optical_frame`，以及动态
  `odometry_map -> imu_link`。
- 参数 `camera_config_path` 指向标定文件，默认使用源码树
  `libflyrat/configs/camera/uvc_640x480.yaml` 或包内安装副本；`body_frame`
  默认 `imu_link`，`world_frame` 默认 `odometry_map`。
- `trajectory_path` 非空时，节点退出前写 TUM 轨迹。
- `bringup.launch.py` / `.script/bringup_real.sh`：一条命令拉起 adb forward、
  设备 forwarder、接收节点和 `odometry_node`，可选 Foxglove；退出时自动清理。
- 已用实机 28 s 桌面 bag 验证短时轨迹：0.35 m 升降、水平漂移约 1 cm；
  尚未做飞行、长距离和长期稳定性验证。
- IMU 的 scale/misalignment 仍由 `flyrat_sensors/config/imu_intrinsics.yaml`
  在发布前修正；IMU 时间戳扣除约 3.8 ms 的相机–IMU 固定时间差。
- `imu.T_body_camera` 使用 `kalibr_ws/` 标定的相机→IMU 外参：相机倒置安装，
  相机原点在 IMU 前方 0.0403 m、右侧 0.0446 m、上方 0.0089 m，基线 0.06475 m。

### 2.6 机载 SLAM 与位姿回传

- `flyrat_pilot`：设备端解码双目 MJPEG（1280x480@30，左右各 640x480）、读取
  200 Hz MSP IMU、按 `imu_intrinsics.yaml` 做 scale/misalignment 和时间偏移
  修正，在本地运行 SchurVINS。
- 位姿语义为 `T_world_body`，body 为 `imu_link`；只通过 TCP 5001 发送 64 字节
  `kPose` 包（位置、四元数、valid、state、epoch 时间戳）。
- 新增 `libflyrat` 协议消息 `kPose` 和 `PosePayload`；IMU 转换、TCP 位姿流分别
  在 `flyrat_pilot/src/slam/` 下实现。
- 新增主机 ROS 2 包 `flyrat_pilot_bridge`：发布
  `/flyrat/odometry/odometry`、`/flyrat/odometry/path`、
  `/flyrat/odometry/status`，以及 `odometry_map -> imu_link` 动态 TF 和相机
  静态 TF。
- `.script/build_pilot_arm.sh`：用最小静态 OpenCV + sysroot 静态 glog/
  yaml-cpp/gflags 交叉编译，`build-arm/flyrat_pilot` 约 11 MB，只依赖
  `libc/libm/ld-linux`。
- `.script/bringup_pilot_slam.sh`：推送二进制和标定、adb forward 5001、
  启动设备 SLAM、主机 bridge 和可选 Foxglove，退出时自动清理。
- 实机 RK3566 实测：30 fps 处理 250 帧全部成功，RSS 约 90 MB；随后 450 帧
  连续运行中初始化成功，静态场景下跟踪会间歇进入 `RECENTLY_LOST`。
- SchurVINS 的初始化器原先固定使用 1–50 m 深度范围，已改为读取适配层的
  0.25–20 m 近场参数，解决实机近距桌面场景无法初始化的问题。

## 3. 实测指标

| 项目 | 实测值 | 说明 |
|---|---:|---|
| IMU | 200.07 Hz | MSP UART3，921600 |
| ToF | 每颗 57–58 Hz | 4 颗 4x4，i2c2 750 kHz |
| ToF 初始化 | 约 7.2 s | 从进程启动到 TCP 监听 |
| 双目 | 60.04 Hz | 1280x480@60 MJPEG，单眼 640x480 |
| adb 吞吐 | 约 30 MB/s | USB2 链路，`adb exec-out` 实测 |
| 双目 MJPEG 带宽 | 约 3.2 MB/s | 1280x480@60，适合 adb |
| 双目原始 NV12 | 约 55 MB/s | 不适合当前 adb，不用于转发 |
| 里程计后端 | SchurVINS 已接入 | 实机 28 s bag：0.35 m 升降、结束水平漂移约 1.4 cm |
| ROS 2 IMU 输入 | 200.3–200.8 Hz | 开发机接收节点 |
| ROS 2 双目输入 | 60.0 Hz | 左右各 640x480 mono8 |
| ROS 2 里程计状态发布 | 1 Hz | `diagnostic_msgs` |
| 机载 SLAM | 30 fps，RSS 约 90 MB | RK3566，640x480/眼，静态桌面场景 |
| 机载二进制 | 约 11 MB | 静态 OpenCV/glog/yaml-cpp，只依赖 libc/libm |

## 4. 构建与运行

所有组件的编译、测试和运行命令见
[`build_and_run.md`](../build_and_run.md)。

## 5. 已知限制

- ToF 地址是易失的：断电后回到 0x52。当前程序会自动探测并恢复，但四颗初始化
  约 7.2 秒，其中主要是 ULD 固件下载和参数配置。
- ToF 地址扫描会遍历 0x08–0x77；如果总线上还有其他 I2C 器件，应避免地址冲突。
- 双目原生 MJPEG 有损；需要原始画质时要改用 YUYV 或 NV12，但 adb 带宽不足。
- 双目与 ToF 没有硬件同步，时间戳来自各自采集/接收路径。
- 双目与 IMU 没有硬件同步；ROS 节点只用 1 Hz `TIME` 帧估计设备/主机偏移，
  避免把 JPEG 解码时间算进图像时间戳。
- ROS 节点使用设备时间戳 + 主机滑动偏移；不是硬实时同步。
- IMU 原始计数默认按 `acc_lsb_per_g=2048`、`gyro_lsb_per_dps=16.4` 换算；
  当前实机已用 Kalibr 标定 scale/misalignment 并在发布前修正，但这两个
  原始量程常数仍需与飞控实际配置一致。
- `flyrat_pilot` 目前只实现机载 SLAM 和位姿回传，飞控主逻辑（MSP 指令、
  失效保护、位置控制）仍未实现。
- adb 链路约 30 MB/s，不能直接转发 640x480/眼@60 的原始 NV12。
- 里程计已用单个实机桌面 bag 验证短时相对尺度和漂移，尚未验证飞行、长距离
  和长期稳定性。
- 实机左目约 55.7 Hz、右目 60 Hz，左目有约 8% 丢帧且更模糊、更亮；当前
  场景仍可跟踪，但高速运动时的鲁棒性需要继续观察。
- SchurVINS 为 GPL-3.0，链接后的 `odometry_node` 组合二进制受 GPL 约束；
  libflyrat 自身源文件仍为 MIT。
- 大段 IMU 断流或长时间跟踪失败后，SVO 可能停在 `kPaused`；需要手动调用
  `/flyrat/odometry/reset` 重新初始化，尚未实现自动重初始化。
- 里程计参数（`kfselect_*`、`max_n_kfs` 等）目前写死在适配层，尚未暴露为
  YAML 参数；实机调参需要重新编译。
- 实机标定数值已保留并迁移到 `configs/camera/`，并已在新后端上完成桌面
  短时验证。
- 泰山派没有系统 OpenCV；机载版本依赖交叉编译的最小静态 OpenCV，构建前缀
  默认为 `/opt/sysroots/arm64/usr/local`。
- 机载 SLAM 只在静态桌面场景短时验证；完全静止时跟踪质量会下降，进入
  `RECENTLY_LOST` 后需要移动恢复或发送 `SIGUSR1` 重建前端。

## 6. 最近提交

```text
23b7974 refactor: remove ORB-SLAM3 and add odometry scaffold
9f09ae4 docs: move all project documentation under docs/agent
3f24857 docs: reorganize by topic and centralize build instructions
60a8483 docs: describe ORB-SLAM3 module and measurements
73a13b1 feat(slam): vendor headless ORB-SLAM3 with stereo facade
f2ea59b fix(tof): define result-trimming macros in platform header
39f5406 docs: record sensor forwarding status and ToF recovery
2420f7e refactor(tof): hardcode the four sensor addresses
9ec445d fix(hardware): recover ToF sensors at retained I2C addresses
390874c feat(sensors): forward real IMU, ToF and stereo over adb
```

## 7. 下一步

1. 用已录制的实机双目/IMU 数据回放 SchurVINS，验证轨迹尺度、漂移、跟踪
   耗时和长期稳定性；根据结果调整 keyframe/feature 参数。
2. 补充双目/IMU 时间对齐与曝光延迟标定，检查 `imu_lookahead` 是否足够。
3. 评估 ToF、双目、IMU 的时间对齐策略，必要时增加设备端统一时间戳。
4. 在 `flyrat_pilot` 中加入飞控主逻辑、失效保护和链路健康状态；位姿接口
   已经具备，可直接接到位置环。
5. 视 adb 带宽和算法需求，评估双目降到 30 Hz 或使用硬件编码的长期方案。
6. 在实机飞行/手持移动场景验证机载 SchurVINS 的尺度、漂移和实时率。
7. 后端输出稳定位姿后，补充跟踪耗时、RSS、漂移等实测指标。
