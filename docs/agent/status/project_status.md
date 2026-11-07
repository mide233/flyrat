# 项目状态

最后更新：2026-11-04

## 1. 项目目标

flyrat 是一个面向 Betaflight 飞控的四旋翼自驾栈：

- `libflyrat/`：不依赖 ROS 的 C++17 核心库，包含 MSP 通信、IMU 读取、姿态控制、
  MPC、状态估计和硬件抽象。
- `flyrat_pilot/`：实机端程序。主飞行逻辑仍在实现中；传感器转发程序已可用。
- `flyrat_play/`：Gazebo + Betaflight SITL 仿真与 ROS 2 适配层。
- `docs/`：构建、仿真、MSP 链路和传感器转发文档。

当前重点是把实机传感器通过 adb 转发到开发机 ROS 2，用真实 IMU、ToF 和双目数据
驱动算法测试，同时保持与仿真相同的 libflyrat 代码路径。

## 2. 已完成模块

### 2.1 核心库

- MSP v1 异步客户端 `flyrat::MspInteract`：TCP/串口、轮询、重连、回调与最新快照。
- `flyrat::hardware::Transport`：串口和 TCP 字节流抽象，单测可注入假传输。
- 姿态保持 `AttitudeHold`、on-manifold MPC `Ommpc`、`LocalEstimator`、`RcReceiver`。
- 单元测试 8 个；启用 SLAM 后另有 `test_slam_api`，共 9 个。

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

### 2.4 VINS-Fusion 双目惯性模块

完整构建、配置和 API 说明见 [`slam/vins_fusion.md`](../slam/vins_fusion.md)。

- `libflyrat/third_party/vins_fusion/`：vendored VINS-Fusion 核心（GPLv3），
  固定上游 commit `be55a93`，只保留 estimator、feature tracker、factor、
  initial 和 camera_models 运行时。
- `flyrat::slam::StereoSlam`：VINS-only 门面 API，只支持双目 + IMU。
  输出 `T_world_body`，状态为 `kNotInitialized/kTracking/kLost`。
- `slam_bench`：支持左右图像序列和左右拼接视频，需要 IMU CSV；输出每帧
  耗时、状态统计和峰值 RSS。实时采集由 ROS 2 节点负责。
- 配置：`configs/vins_fusion/uvc_640x480_imu.yaml`、
  `cam0_pinhole.yaml`、`cam1_pinhole.yaml`（已按 `kalibr_ws/` 实机标定）。
- 不再需要视觉词袋；默认 `FLYRAT_BUILD_SLAM=OFF`，开启后链接的程序整体按
  GPLv3 分发。

### 2.5 ROS 2 SLAM 节点

完整构建、参数和 Foxglove 预览见 [`slam/vins_fusion.md`](../slam/vins_fusion.md)；
坐标链和变换调整见 [`slam/coordinates.md`](../slam/coordinates.md)。

- `flyrat_play/ros2_ws/src/flyrat_slam/`：ROS 2 包，只支持双目 + IMU。
- 订阅实机转发的左右目和 IMU，双目用 `ApproximateTime` 同步，IMU 缓存约 1 s，
  跟踪线程忙时丢旧帧。
- 发布 `/flyrat/slam/odometry`、`/flyrat/slam/path`、`/flyrat/slam/status`、
  `/flyrat/slam/reset`；动态 TF `slam_map -> imu_link`，静态 TF
  `imu_link -> camera_left_optical_frame` 和
  `imu_link -> camera_right_optical_frame`。
- 参数 `vins_config` 指向 VINS YAML，默认使用包内
  `uvc_640x480_imu.yaml`；`body_frame` 默认 `imu_link`；可选
  `trajectory_path` 退出时保存 TUM 轨迹。
- `bringup.launch.py` / `.script/bringup_real.sh`：一条命令拉起 adb forward、
  设备 forwarder、接收节点和 SLAM，可选 Foxglove；退出时自动清理。
  VINS 配置默认从 `FLYRAT_VINS_CONFIG`、源码树或安装目录解析。
- `body_T_cam0` / `body_T_cam1` 来自 `kalibr_ws/` 的相机→IMU 外参：
  相机倒置安装，左目原点在 IMU 前方 0.0403 m、右侧 0.0446 m、上方 0.0089 m，
  基线 0.06475 m。
- IMU 的 scale/misalignment 由 `flyrat_sensors/config/imu_intrinsics.yaml`
  在发布前修正；IMU 时间戳扣除约 3.8 ms 的相机–IMU 固定时间差，因此 VINS
  配置使用 `td=0`。
- VINS 需要约 10 帧窗口和足够的运动/IMU 激励才能初始化；静止桌面通常会停留
  在 `NOT_INITIALIZED`，这不是标定错误。

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
| VINS-Fusion 编译/构造 | 通过 | 开发机，OpenCV 4.6 + Ceres 2.2 |
| VINS-Fusion 合成回放 | 初始化后 4/4 帧 TRACKING | 15 帧合成双目 + 200 Hz IMU |
| VINS-Fusion ROS 2 合成流 | 27 条 odometry | 40 帧合成双目 + 200 Hz IMU |
| VINS-Fusion 真实数据跟踪 | 待实测 | 需要带 IMU 的实机双目序列 |
| VINS-Fusion 单帧耗时 | 约 11–18 ms（合成数据） | 开发机，Release |
| ROS 2 IMU 输入 | 200.3–200.8 Hz | 开发机接收节点 |
| ROS 2 双目输入 | 60.0 Hz | 左右各 640x480 mono8 |
| ROS 2 SLAM 状态发布 | 1 Hz | `diagnostic_msgs` |

## 4. 构建与运行

所有组件的编译、测试和运行命令见
[`build_and_run.md`](../build_and_run.md)。

## 5. 已知限制

- ToF 地址是易失的：断电后回到 0x52。当前程序会自动探测并恢复，但四颗初始化
  约 7.2 秒，其中主要是 ULD 固件下载和参数配置。
- ToF 地址扫描会遍历 0x08–0x77；如果总线上还有其他 I2C 器件，应避免地址冲突。
- 双目原生 MJPEG 有损；需要原始画质时要改用 YUYV 或 NV12，但 adb 带宽不足。
- 双目与 ToF 没有硬件同步，时间戳来自各自采集/接收路径。
- ROS 节点使用设备时间戳 + 主机滑动偏移；不是硬实时同步。
- IMU 原始计数默认按 `acc_lsb_per_g=2048`、`gyro_lsb_per_dps=16.4` 换算；
  当前实机已用 Kalibr 标定 scale/misalignment 并在发布前修正，但这两个
  原始量程常数仍需与飞控实际配置一致。
- 主飞行程序 `flyrat_pilot` 仍是占位 `main()`；目前实机功能集中在
  `flyrat_sensor_forwarder`。
- adb 链路约 30 MB/s，不能直接转发 640x480/眼@60 的原始 NV12。
- VINS-Fusion 只在开发机验证，未在泰山派 aarch64 构建；泰山派没有
  OpenCV/Ceres，内存也只有 2 GB。
- SLAM 配置的内参、双目外参、IMU 外参和 IMU 随机游走已按 `kalibr_ws/`
  标定；白噪声密度使用飞行振动余量（见 `slam/vins_fusion.md`），不能用
  静止 Allan 底噪。真实运动回放已能保持有界轨迹，但长距离尺度和精度仍待
  进一步验证。
- VINS-Fusion 需要约 10 帧窗口和足够的运动/IMU 激励才能初始化；静止桌面
  通常停留在 `NOT_INITIALIZED`。
- VINS 核心不含回环/位姿图，长距离会漂移。
- VINS 参数是进程级全局变量，一个进程只能有一个 `StereoSlam` 实例。
- 双目与 IMU 没有硬件同步；ROS 节点只用 1 Hz `TIME` 帧估计设备/主机偏移，
  避免把 JPEG 解码时间算进图像时间戳。
- SLAM 模块为 GPLv3，链接后整体程序需按 GPLv3 分发。

## 6. 最近提交

```text
9f09ae4 docs: move all project documentation under docs/agent
3f24857 docs: reorganize by topic and centralize build instructions
f2ea59b fix(tof): define result-trimming macros in platform header
39f5406 docs: record sensor forwarding status and ToF recovery
2420f7e refactor(tof): hardcode the four sensor addresses
9ec445d fix(hardware): recover ToF sensors at retained I2C addresses
390874c feat(sensors): forward real IMU, ToF and stereo over adb
```

## 7. 下一步

1. 已应用双目内参、双目外参、IMU–相机外参和 IMU 随机游走；白噪声密度已
   加入飞行振动余量，真实运动回放不再发散。下一步补充 ROS `CameraInfo`，
   并继续评估长距离尺度和轨迹精度。
2. 用更长、更剧烈的真实飞行验证 VINS-Fusion 持续 `TRACKING`，在 Foxglove
   中检查轨迹、TF、`ba` 和状态，再评估精度和漂移。
3. 评估 ToF、双目、IMU 的时间对齐策略，必要时增加设备端统一时间戳。
4. 将传感器转发和 SLAM 整合进正式 `flyrat_pilot` 运行时，加入失效保护与
   链路健康状态。
5. 视 adb 带宽和算法需求，评估双目降到 30 Hz 或使用硬件编码的长期方案。
6. 评估 SLAM 在泰山派上的可行性（需要 OpenCV/Ceres 和更大内存），或维持开发机运算。
