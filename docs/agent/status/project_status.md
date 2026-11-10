# 项目状态

最后更新：2026-11-08

## 1. 项目目标

flyrat 是一个面向 Betaflight 飞控的四旋翼自驾栈：

- `libflyrat/`：不依赖 ROS 的 C++17 核心库，包含 MSP 通信、IMU 读取、姿态控制、
  MPC、状态估计和硬件抽象。
- `flyrat_pilot/`：实机端程序。主飞行逻辑仍在实现中；传感器转发程序已可用。
- `flyrat_play/`：Gazebo + Betaflight SITL 仿真与 ROS 2 适配层。
- `docs/`：构建、仿真、MSP 链路和传感器转发文档。

当前重点是把实机传感器通过 adb 转发到开发机 ROS 2，用真实 IMU、ToF 和单目
黑白相机数据驱动算法测试，同时保持与仿真相同的 libflyrat 代码路径。

## 2. 已完成模块

### 2.1 核心库

- MSP v1 异步客户端 `flyrat::MspInteract`：TCP/串口、轮询、重连、回调与最新快照。
- `flyrat::hardware::Transport`：串口和 TCP 字节流抽象，单测可注入假传输。
- 姿态保持 `AttitudeHold`、on-manifold MPC `Ommpc`、`LocalEstimator`、`RcReceiver`。
- 单元测试 9 个，其中 `test_odometry_api` 覆盖里程计骨架接口和桩行为；
  开启 `FLYRAT_BUILD_TIO=ON` 后另加 `test_tio_api`。

### 2.2 实机硬件

- `hardware::TofArray`：4 颗 VL53L5CX 共用 i2c2；支持 4×4@60 Hz 或
  8×8@15 Hz。转发器默认 8×8@15 Hz，裁剪结果块 128 B/帧。
  - 地址固定为 `0x60/0x62/0x64/0x66`，不再支持单独配置。
  - 每次启动逐颗释放 LPn，自动探测残留地址并改回固定地址。
  - 不做任何 I2C 访问前先把四路 LPn 拉低，避免上电都处于 0x52 时冲突。
  - 输出裁剪宏统一定义在 `platform/platform.h`，保证 C 驱动和 C++ 读取端
    的 `VL53L5CX_ResultsData` 布局一致。
- `hardware::Imu`：读取 `MspInteract` 的最新 `MSP_RAW_IMU` 快照。
- `hardware::MonoJpegSource`：USB 黑白相机 V4L2 MJPEG 采集，默认
  `/dev/video10`、640x480@60；设备端不解码，转发原生 JPEG。
- `hardware::StereoCamera`：旧双目采集实现，当前实机已移除双目，保留为 legacy。

### 2.3 实机传感器转发

完整启动、参数和话题说明见
[`hardware/sensor_forwarding.md`](../hardware/sensor_forwarding.md)。

设备端 `flyrat_sensor_forwarder`：

- IMU：`MspInteract`，`MSP_RAW_IMU` 200 Hz。
- ToF：4 颗 VL53L5CX，每颗 8×8 @ 15 Hz（转发器默认；旧 4×4 实测约 57–58 Hz）。
- 单目：`/dev/video10` 原生 MJPEG，640x480@60，不在设备端解码。
- TCP 服务：`127.0.0.1:5000`，单客户端；相机队列满时丢旧帧，IMU/ToF 不阻塞。

开发机 ROS 2：

- `flyrat_sensors_interfaces`：`TofZone` / `TofFrame` 自定义消息。
- `flyrat_sensors`：`sensor_receiver_node`，负责 TCP 接收、协议解析、图像解码和发布。
  IMU 默认 `imu_accel_axis=none`、`imu_gyro_axis=none`：加速度和陀螺仪原始
  数据都已是 ROS FLU（实测左偏航 `angular_velocity.z > 0`）。
- 默认话题：
  - `/flyrat/sensors/imu`
  - `/flyrat/sensors/tof_0` … `/flyrat/sensors/tof_3`
  - `/flyrat/sensors/camera/image_raw`（mono8，640x480）

### 2.4 legacy 里程计骨架

完整接口、配置和接入方式见
[`odometry/overview.md`](../odometry/overview.md)。

- `flyrat::odometry::Odometry`：ROS 无关的 C++17 接口，支持 `kStereo` 和
  `kImuStereo` 两种模式，状态为 `kNotInitialized/kTracking/kRecentlyLost/kLost`。
- 输入使用非拥有型 `ImageView`（8 位灰度）和 `ImuSample` 序列，因此核心库
  不依赖 OpenCV。
- 当前为桩实现：构造时校验标定文件，`track()` 校验图像尺寸和关闭状态后返回
  `kNotInitialized`，不输出有效位姿；`saveTrajectory()` 明确抛出“未实现”。
- 标定文件 `libflyrat/configs/camera/uvc_640x480.yaml` 保留实机 Kalibr 数值
  （双目内参/畸变、`camera.T_left_right`、`imu.T_body_camera`、IMU 噪声）。
- `flyrat_odometry` 目标默认随 libflyrat 构建，无需额外的视觉算法、Boost 或
  OpenSSL 依赖；`test_odometry_api` 覆盖接口与桩行为。

### 2.5 legacy 立体 ROS 2 节点

坐标链和标定调整见 [`odometry/coordinates.md`](../odometry/coordinates.md)。

- `flyrat_play/ros2_ws/src/flyrat_odometry/`：默认入口是 TIO-Former 节点；
  旧的左右目/IMU 节点保留为 legacy 骨架。
- legacy 节点订阅左右目和 IMU，双目用 `ApproximateTime` 同步，IMU 缓存约 1 s，
  跟踪线程忙时丢旧帧。当前实机已无双目，该路径不再默认启动。
- 发布 `/flyrat/odometry/odometry`、`/flyrat/odometry/path`、
  `/flyrat/odometry/status`、`/flyrat/odometry/reset`；静态 TF
  `imu_link -> camera_left_optical_frame` 和
  `camera_left_optical_frame -> camera_right_optical_frame`。后端输出有效位姿后
  再发布动态 `odometry_map -> imu_link`。
- 参数 `camera_config_path` 指向标定文件，默认使用源码树
  `libflyrat/configs/camera/uvc_640x480.yaml` 或包内安装副本；`body_frame`
  默认 `imu_link`，`world_frame` 默认 `odometry_map`。
- `bringup.launch.py` / `.script/bringup_real.sh`：一条命令拉起 adb forward、
  设备 forwarder、接收节点和默认 TIO 节点，可选 Foxglove；退出时自动清理。
- 当前节点启动后发布 `NOT_INITIALIZED` 状态并提示后端尚未实现，没有有效位姿
  或轨迹输出。
- IMU 的 scale/misalignment 仍由 `flyrat_sensors/config/imu_intrinsics.yaml`
  在发布前修正；约 3.8 ms 的相机–IMU 时间偏移来自旧双目标定，当前单目没有
  新标定。
- `imu.T_body_camera` 使用 `kalibr_ws/` 标定的相机→IMU 外参：相机倒置安装，
  相机原点在 IMU 前方 0.0403 m、右侧 0.0446 m、上方 0.0089 m，基线 0.06475 m。

### 2.6 TIO-Former 实验后端

完整说明见 [`odometry/tio_former.md`](../odometry/tio_former.md)。

- `research/tio_former/` 的仿真模型已导出为
  `libflyrat/models/tio_former_window64.pt`（TorchScript，4.6 MB）。
- `flyrat::tio::TioOdometry` 在 C++ 中维护 64 帧 ToF/IMU 滑动窗口，调用
  libtorch 推理并积分机体增量；模型输入为前/左/右/下 4 个 8×8 距离图和
  16 个 IMU 样本/帧。机头已改到原后向 ToF 一端，实机传感器 ID 为
  0=下、1=前、2=右、3=左，`sensor_order: [1,3,2,0]`。
- `tio_odometry_node` 订阅 `/flyrat/sensors/tof_0..3` 和
  `/flyrat/sensors/imu`，发布与旧立体节点相同的 `/flyrat/odometry/*` 和
  `odometry_map -> imu_link` TF。
- TIO 目标默认关闭；需要 `FLYRAT_BUILD_TIO=ON` 和 CPU libtorch。开发机已用
  仿真序列验证端到端推理和 ROS 发布，`test_tio_api` 覆盖接口、warm-up、
  reset 和推理冒烟。
- 2026-11-08 旧布局真机验证：四颗 ToF 设备端实测 14.8–15.2 Hz，0 读取错误；
  `tio_odometry_node` 能进入 `TRACKING` 并发布 odometry/TF。
- 旧布局下真机静态放桌面 20 s 漂移约 0.98 m，当前不能用于飞行。主要原因是
  下视传感器距桌面 0–5 mm、当时映射 front 槽位接后向以及仿真域差异。机头重定义
  后的新映射尚未做净空静止/飞行验证。
  完整记录见 [`odometry/handoff.md`](../odometry/handoff.md)。
- 尚未完成实机精度/噪声验证、RK3566 交叉编译和机载性能评估。
- 模型改进 v2（`hybrid` / `gated_fifo` / `hybrid_gated`）已完成 15/40-epoch
  筛选：40-epoch `hybrid_gated` ATE 0.4122 m，优于原 full 的 0.4316 m，
  但仍未超过 `naive_concat`（0.4013 m）和 `kv_only_no_fifo`（0.3925 m），
  且 RPER 0.757° 明显差于 full 的 0.257°。位置/旋转双分支 `hybrid_split`
  15-epoch ATE 0.4350 m、RPER 0.837°，没有改善。结论为负结果，当前导出
  模型仍为原 full 模型。
- 数据集 v3.1（2026-11-09）：修正 train/val/test 速度分布、关闭合成
  status=10、加入 dropout 和垂直运动，共 894 序列 / 22,927.5 s /
  6,229.5 m / 344,829 帧。3 seeds 取得测试 ATE 0.0879 ± 0.0025 m、
  RPEp 0.00363 m、RPER 0.175°、EDR 2.52%，路径比约 1.00，
  三个 seed 的 |speed–ATE| 均在 0.34–0.47，v3 gate 全部通过。结果见
  [`research/tio_former/report/v3_model/analysis.md`](../../research/tio_former/report/v3_model/analysis.md)。
  该结果仍为四个 Gazebo 场景的仿真值，实机域差距尚未验证。
- P1 前向单目相机：仿真数据集可记录 320×240@15 Hz 单目图像（存为
  160×120 JPEG），模型增加 CameraEncoder 与可靠性 gate。完整数据集
  891 序列 / 23,361 s，相机模型 3 seeds 测试 ATE 0.0868 ± 0.0047 m；
  相机置零 0.0945 m，全部 ToF 丢弃时 0.174 m（无相机 0.205 m）。
  `libflyrat/models/tio_former_window64_camera.pt` 已导出（窗口 64、
  160×120、1,278,937 参数），并设为 `bringup_real.sh` 的默认 TIO 模型；
  `tio_camera:=false` 可回退旧 ToF+IMU 模型。真实相机仍未标定，实机
  精度尚未复测。

## 3. 实测指标

| 项目 | 实测值 | 说明 |
|---|---:|---|
| IMU | 200.07 Hz | MSP UART3，921600 |
| ToF | 每颗 8×8 @ 15 Hz（配置） | 4 颗共用 i2c2 750 kHz；旧 4×4 实测约 57–58 Hz |
| ToF 设备端实测 | 14.8–15.2 Hz | 2026-11-08，四颗各 10 s，0 读取错误 |
| ToF 单帧读取 | 约 4.7 ms | 设备端 `tof_bench`，8×8 |
| ToF 初始化 | 约 7.2 s | 从进程启动到 TCP 监听 |
| 单目相机 | 640x480@30（实测） | `/dev/video10` MJPEG，请求 60 Hz |
| adb 吞吐 | 约 30 MB/s | USB2 链路，`adb exec-out` 实测 |
| 单目 MJPEG 带宽 | 通常 <1 MB/s | 640x480@60 黑白，适合 adb |
| 里程计骨架状态 | `NOT_INITIALIZED` | 节点可启动，后端尚未实现 |
| ROS 2 IMU 输入 | 200.3–200.8 Hz | 开发机接收节点 |
| ROS 2 单目输入 | 约 30 Hz（实测） | `/flyrat/sensors/camera/image_raw` mono8 640x480 |
| ROS 2 里程计状态发布 | 1 Hz | `diagnostic_msgs` |
| TIO-Former ROS 链路 | `TRACKING` | 真机 ToF+IMU，开发机 CPU libtorch |
| TIO-Former 实机静态零漂 | 约 0.98 m / 20 s | 2026-11-08 旧布局桌面测试；未通过飞行要求 |
| TIO-Former v3.1 仿真测试 | ATE 0.088 m / EDR 2.52% | 3 seeds，各 141 条 test，四个 Gazebo 场景 |

## 4. 构建与运行

所有组件的编译、测试和运行命令见
[`build_and_run.md`](../build_and_run.md)。

## 5. 已知限制

- ToF 地址是易失的：断电后回到 0x52。当前程序会自动探测并恢复，但四颗初始化
  约 7.2 秒，其中主要是 ULD 固件下载和参数配置。
- ToF 地址扫描会遍历 0x08–0x77；如果总线上还有其他 I2C 器件，应避免地址冲突。
- 单目相机原生 MJPEG 有损；当前没有单目内参/畸变标定。
- 相机与 ToF、IMU 没有硬件同步，时间戳来自各自采集/接收路径。
- ROS 节点只用 1 Hz `TIME` 帧估计设备/主机偏移，避免把 JPEG 解码时间算进
  图像时间戳。
- ROS 节点使用设备时间戳 + 主机滑动偏移；不是硬实时同步。
- IMU 原始计数默认按 `acc_lsb_per_g=2048`、`gyro_lsb_per_dps=16.4` 换算；
  当前实机已用 Kalibr 标定 scale/misalignment 并在发布前修正，但这两个
  原始量程常数仍需与飞控实际配置一致。
- 主飞行程序 `flyrat_pilot` 仍是占位 `main()`；目前实机功能集中在
  `flyrat_sensor_forwarder`。
- adb 链路约 30 MB/s；当前 640x480@60 单目 MJPEG 带宽足够。
- 里程计后端尚未实现：`flyrat::odometry` 和 ROS 2 节点只提供接口、同步、
  TF 和状态，不输出有效位姿或轨迹。
- 实机标定数值已保留并迁移到 `configs/camera/`，但尚未在新后端上验证尺度、
  漂移和长期稳定性。
- 里程计 ROS 节点依赖 OpenCV，目前在开发机运行；泰山派没有 OpenCV，内存也
  只有 2 GB，能否上机仍需评估。
- TIO-Former 后端依赖 libtorch，只在 Gazebo 前/左/右/下 4 向 ToF 数据上
  训练和验证；实机 ToF/IMU 链路已通，新布局也是前/左/右/下，但静态零漂
  约 0.98 m / 20 s（旧映射）仍待在新布局下复测，真实噪声、安装方向复核、
  RK3566 交叉编译与机载延迟仍未完成。

## 6. 最近提交

```text
6aad1be refactor(tio): harden input handling and add handoff docs
4593a72 feat(sensors): configure real ToF 8x8@15Hz and direction mapping
c9edd8d feat(research): add TIO model variants and C++/ROS backend
3621ad7 feat(research): complete M5 seeds, LOSO and technical report
877a554 feat(research): complete M4 training, ablations and robustness
d4c7b77 feat(research): implement TIO-Former model and overfit smoke test
4c73327 feat(research): generate full TIO-Former simulation dataset
077b097 feat(research): scaffold TIO-Former simulation and M1 dataset pipeline
910f78d refactor: remove ORB-SLAM3 and add odometry scaffold
```

## 7. 下一步

### P0：TIO-Former 实机基线

1. 把无人机抬到离地约 1 m、四周无近距遮挡处，静止 30–60 s 重测零漂，确认
   桌面 0.98 m/20 s 的漂移有多少来自下视贴桌面。
2. 逐个遮挡/靠近传感器，确认 0/1/2/3 与 下/前/右/左 的物理方向对应。
3. 录一段真实 8×8 ToF + IMU 序列（含静止和已知运动），作为域适配/微调数据。
4. 用新布局重测 TIO；若仍有明显域差异，再用实机数据微调/重训并重新导出。

### P1：工程化与机载可行性

5. 优化 C++ 推理：当前每帧重算 64 帧窗口，开发机 CPU 约 9 ms；评估增量
   streaming cache、线程/批处理和内存占用。
6. 评估 libtorch 在 RK3566/Buildroot 上的交叉编译、延迟和 RSS。
7. 统一 ToF/IMU 时间基准；确认 15 Hz ToF 与 200 Hz IMU 的时间对齐。
8. 将传感器转发和里程计整合进正式 `flyrat_pilot`，加入 `TRACKING` 失效、
   IMU 超时、ToF 丢帧过多时的保护。

### P2：单目与 legacy 立体

9. 为当前黑白单目做内参/畸变标定，确定 `camera_front_optical_frame` 与
   `imu_link` 的外参；在需要时再接入视觉/视觉惯性后端。
10. legacy `flyrat::odometry` 仍是桩实现；当前没有双目硬件，暂不继续扩展，
    仅保留接口和旧数据回放能力。

### P3：指标与文档

11. 后端输出稳定位姿后，补充跟踪耗时、RSS、漂移等实测指标，并替换当前
    “里程计骨架”占位状态。

完整的代码地图、接口契约和调试清单见
[`odometry/handoff.md`](../odometry/handoff.md)。
