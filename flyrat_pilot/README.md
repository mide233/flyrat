# flyrat_pilot

实机端程序。包含两个可执行文件：

- `flyrat_pilot`：在设备本地解码双目 MJPEG、读取 MSP IMU，运行 SchurVINS
  双目惯性里程计，并通过 TCP 把 `T_world_body` 位姿发给主机。
- `flyrat_sensor_forwarder`：把原始 IMU、ToF 和双目 MJPEG 转发给主机 ROS 2，
  用于录包和开发机算法验证。

两者都不依赖 ROS。主机侧 `flyrat_pilot_bridge` 把位姿流转换成
`nav_msgs/Odometry`、`nav_msgs/Path` 和 TF，供 Foxglove 查看。

## 1. 本机构建与运行

```bash
.script/build_flyrat_pilot.sh
```

产物：

```text
flyrat_pilot/build/flyrat_pilot
flyrat_pilot/build/flyrat_sensor_forwarder
```

在开发机上做配置自检（不打开相机和串口）：

```bash
./flyrat_pilot/build/flyrat_pilot --check-config \
  --camera-config libflyrat/configs/camera/uvc_640x480.yaml \
  --imu-config flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml
```

主要参数：

```text
--camera-config PATH   双目/IMU 标定 YAML
--imu-config PATH      IMU scale/misalignment YAML
--msp-device DEV       飞控串口，默认 /dev/ttyS3
--msp-baud N           默认 921600
--camera-device DEV    V4L2 MJPEG 设备，默认 /dev/video10
--width/--height N     左右拼接图尺寸，默认 1280x480
--fps N                相机帧率，默认 25
--decode MODE          MJPEG 解码器：cpu（默认）或 mpp
--opencv-threads N    OpenCV 工作线程数，默认 2
--slam-cpu N          把 SLAM 主线程绑定到 CPU N（默认不绑定）
--warmup SEC          启动后丢弃相机帧的预热时间，默认 1.0
--pose-address ADDR    位姿流监听地址，默认 127.0.0.1
--pose-port PORT       位姿流端口，默认 5001
--trajectory PATH      TUM 轨迹，空字符串关闭
--imu-lookahead SEC    每帧之后的 IMU 样本，默认 0.010
--imu-history SEC      首帧 IMU 历史，默认 0.1
--max-frames N         处理 N 帧后退出，0 表示一直运行
--trace DIR            写出逐帧耗时 CSV 到 DIR
--check-config         只校验配置并退出
```

运行中发送 `SIGUSR1` 会重建 SchurVINS 前端（等价于一次 reset），`SIGINT`
或 `SIGTERM` 会停止采集、关闭位姿服务并保存轨迹。

## 2. 交叉编译到泰山派

设备是 RK3566 + Buildroot，没有 OpenCV/glog/yaml-cpp。先准备带最小静态
OpenCV 的 aarch64 sysroot，再执行：

```bash
.script/build_pilot_arm.sh
```

产物是 `build-arm/flyrat_pilot`，只依赖 `libc`、`libm` 和动态加载器。
脚本默认使用 `/opt/sysroots/arm64`，可用 `FLYRAT_SYSROOT` 覆盖；默认
`JOBS=4`，避免在内存较小的构建机上触发 OOM。

最小静态 OpenCV 的构建方法见
[`docs/agent/build_and_run.md`](../docs/agent/build_and_run.md) 的
“机载 SLAM”。

传感器转发仍然用：

```bash
.script/build_sensor_forwarder_arm.sh
```

## 3. 机载 SLAM 与位姿回传

一条命令完成推送、adb forward、设备端 SLAM、主机 bridge 和可选 Foxglove：

```bash
.script/bringup_pilot_slam.sh start_foxglove:=true
```

只启动指定帧率，不启动 Foxglove：

```bash
.script/bringup_pilot_slam.sh fps:=25 max_frames:=0
```

脚本会：

1. 把 `build-arm/flyrat_pilot`、相机标定和 IMU 标定推到设备 `/tmp/flyrat`；
2. 建立 `adb forward tcp:5001 tcp:5001`；
3. 在设备上启动 `/tmp/flyrat_pilot`；
4. 在主机启动 `ros2 launch flyrat_pilot_bridge pilot_bridge.launch.py`；
5. 退出时停止设备进程并移除 adb forward。

主机话题：

```text
/flyrat/odometry/odometry   nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path       nav_msgs/Path
/flyrat/odometry/status     diagnostic_msgs/DiagnosticStatus，1 Hz
```

静态 TF 为 `imu_link -> camera_left_optical_frame` 和
`camera_left_optical_frame -> camera_right_optical_frame`，动态 TF 为
`odometry_map -> imu_link`。

手动运行主机 bridge：

```bash
adb forward tcp:5001 tcp:5001
ros2 launch flyrat_pilot_bridge pilot_bridge.launch.py \
  camera_config_path:=$PWD/libflyrat/configs/camera/uvc_640x480.yaml
```

## 4. 位姿协议

设备只回传 64 字节 `PosePayload`，外层复用
`libflyrat/include/flyrat/sensors/protocol.hpp` 的 28 字节帧头：

```text
uint8  type = 6 (kPose)
double position[3]       T_world_body 平移，米
double orientation[4]    x, y, z, w
uint8  valid
uint8  state             0..3，对应 odometry::State
uint8  reserved[6]
```

`device_time_ns` 是 Unix epoch 纳秒。位姿语义是左目相机外参换算后的
`T_world_body`，body 为 `imu_link`（FLU）；world 原点在首次初始化位置。
服务只保留最新位姿，主机断线重连不会补发历史帧。

## 5. 设备实测与限制

### 5.1 性能基线

`libflyrat` 提供 `MppJpegDecoder`：通过 `dlopen` 调用 `librockchip_mpp` 的
异步 `decode_put_packet` / `decode_get_frame` 接口，输出 NV12/NV16 的 Y 平面。
`decode_bench` 在实机 25 fps 下连续 60 s：1496 帧、0 解码错误、无超过
100 ms 的间隔，硬件解码平均 11.3 ms、p99 12.1 ms，进程 CPU 约 27%，
而 CPU `cv::imdecode` 约 56%；输出与 OpenCV 平均差 0.01 LSB。

机载 `flyrat_pilot` 默认仍使用 CPU `cv::imdecode`；硬件解码需显式启用：

```text
flyrat_pilot --decode mpp ...
```

`--decode mpp` 已在完整 SchurVINS 进程中连续验证 6 次 600 帧（25/30 fps 各
3 次）：无 UVC 停流、`camera_drops=0`、`decode_fallbacks=0`。25 fps 进程 CPU
约 126–130%（CPU 解码约 135%），30 fps 约 136–140%（CPU 解码约 142%）；
由于解码（约 11 ms）位于帧预算内、不在关键路径上，帧年龄基本不变。若 MPP
初始化失败或连续 10 帧解码失败，pilot 会自动回退到 `cv::imdecode`，并在
统计行累加 `decode_fallbacks`。数据、`decode_bench` 用法和后续步骤见
[`docs/agent/odometry/mjpeg_decode_research.md`](../docs/agent/odometry/mjpeg_decode_research.md)。

解码线程只消费 `StereoJpegSource::takeLatest()` 的最新 JPEG，不再在 V4L2 采集
回调里解码；`StereoJpegSource` 在无回调时也改为移动帧而不是复制。

RK3566（4 核 A55，2 GB），640x480/眼，静态近距桌面场景：

| 模式 | CPU | 进程 RSS | `track()` | 帧年龄 | 处理帧率 |
|---|---:|---:|---:|---:|---:|
| 30 fps，60 s | 1.5–2.0 核（135–200%） | 27–30 MB | 18–21 ms | 约 41 ms | 29.9 fps |
| 30 fps，600 帧 | 约 1.85 核 | 28 MB | 约 33 ms | 约 52 ms | 28.5 fps |
| 25 fps，582 帧（P0 优化后） | 约 1.3 核（133%） | 18–19 MB | 25–30 ms | 约 51 ms | 25.0 fps |
| 25 fps，600 帧，`--decode mpp` | 约 1.28 核（126–130%） | 20–22 MB | 25–26 ms | 约 50 ms | 25.0 fps |
| 30 fps，600 帧，`--decode mpp` | 约 1.38 核（136–140%） | 20–22 MB | 21–23 ms | 42–43 ms | 30.0 fps |
| 25 fps，600 帧，NEON + 并行重投影 | 约 1.23 核（123%） | 约 22 MB | **16–17 ms** | 约 50 ms | 25.0 fps |
| 30 fps，600 帧，NEON + 并行重投影 | 约 1.44 核（144%） | 约 22 MB | **15–16 ms** | 约 43 ms | 30.0 fps |
| 25 fps，150 帧（优化前） | 约 1.3 核（133%） | 27 MB | 约 28 ms | 约 47 ms | 25.1 fps |
| 20 fps，400 帧 | 约 1.6 核 | 22 MB | 约 35 ms | 约 57 ms | 19.7 fps |

- `track()` 是 SchurVINS 单帧耗时，`age` 是图像时间戳到开始跟踪的延迟（含
  10 ms IMU lookahead 等待）。
- P0 优化后，解码已从采集回调移到独立线程，只解码最新帧；JPEG 不再双重拷贝；
  主机 bridge 改为收到即发布；默认 IMU lookahead 降到 10 ms。25 fps 下
  `track_ms` 25–30 ms，帧预算 40 ms，仍有约 10 ms 余量。
- 30 fps 的帧预算是 33 ms，单帧稳态已占 18–21 ms，CPU 峰值接近 2 核，
  留给飞控和其他线程的余量较小。默认保持 **25 fps**；需要更大余量时可降到 20 fps。
- 20 fps 的帧预算是 50 ms，单帧约 35 ms，实时裕量约 15 ms；代价是帧间位移
  变大，快速运动时跟踪难度上升。
- 端到端（采集 → 主机 ROS 2 收到）约 **70–100 ms**。设备与主机系统时钟有
  **+55～64 ms** 偏差，`ros2 topic delay` 的原始值需要先扣除该偏差。
- `camera_drops=0`、`skipped=0`，RSS 稳定在 22–30 MB，无内存压力；SoC 热区
  约 60–61°C。
- NEON 路径与双目并行重投影把 `track()` 从 25–27 ms 降到 16–17 ms；注意
  启用并行重投影时不要用 `--slam-cpu` 固定 SLAM 线程（工作线程会继承单核
  亲和性）。细节见
  [`docs/agent/odometry/performance_profiling.md`](../docs/agent/odometry/performance_profiling.md)。

### 5.2 分阶段耗时（`--trace`）

`--trace DIR` 在 `DIR` 下写两个逐帧 CSV；不传该参数时不创建文件，计时分支
也不会进入。

- `trace_pilot.csv`：
  `seq,stamp_s,queue_ms,decode_ms,imu_wait_ms,collect_ms,track_ms,publish_ms,age_ms,state,valid`
  — 解码完成到 SLAM 取帧的排队、解码、IMU 等待、IMU 收集、`track()` 和发布耗时。
- `trace_frontend.csv`：SVO 内置计时器，单位为秒。`pyramid_creation` 包含
  `imgs[i].clone()` 和金字塔构建；`sparse_img_align` 对应 SchurVINS `Forward`；
  `pose_optimizer` 对应 `Backward`；其余为 `reproject`、`point_optimizer` 和
  `frontend_time`。

```bash
adb shell '/tmp/flyrat_pilot --decode mpp --trace /tmp/trace --max-frames 1500'
adb pull /tmp/trace .
python3 flyrat_pilot/tools/analyze_trace.py trace
```

分析脚本输出各列的 p50/p95/p99/max，并给出 `frontend_time` 与各阶段的占比和
未计入阶段的时间。两个 CSV 按行号对齐（每个成功跟踪的帧一行）；启动阶段建议
加 `--skip 100`。

首次实测（25 fps、600 帧、`--decode mpp`、跳过前 100 帧）：`track()` p50
27.5 ms，其中 `reproject` 12.7 ms（53%）、`pose_optimizer` 6.8 ms（26%）、
`pyramid_creation` 2.9 ms（12%）、`sparse_img_align` 1.0 ms；解码 7.0 ms
不在关键路径上，帧年龄 48.7 ms。完整数据和 30 fps 对比见
[`docs/agent/odometry/performance_profiling.md`](../docs/agent/odometry/performance_profiling.md)。

### 5.3 行为与限制

- 静态近距桌面场景能完成初始化并输出位姿；长时间完全静止时跟踪质量会下降，
  状态可能进入 `RECENTLY_LOST`，轻微移动后恢复。
- 30 fps 长跑（1800 帧）时静态轨迹三轴范围约 1–2 mm，累计路径 3.9 cm；
  600 帧样本因跟踪状态变化出现约 5 cm 的 z 波动，不能作为精度指标。
- 只在桌面短时场景验证，尚未做飞行、长距离和长期漂移验证。
- 左目约 55.7 Hz、右目 60 Hz，左目有约 8% 丢帧且画面更模糊、更亮。
- SchurVINS 为 **GPL-3.0**，`flyrat_pilot` 链接后组合二进制受 GPL 约束；
  libflyrat 自身源文件仍为 MIT。

## 6. 预留的飞控接入点

- `flyrat::MspInteract`：MSP v1 异步客户端，串口/TCP、轮询、重连和最新快照。
- `flyrat::hardware::TofArray`：4 颗 VL53L5CX，4x4@60 Hz。
- `flyrat::hardware::Imu`：MSP IMU 快照。
- `flyrat::hardware::StereoCamera` / `StereoJpegSource`：UVC 双目采集。
- `flyrat::RcReceiver`：UDP 9004 虚拟接收机（仿真）。
- `flyrat::Autopilot` / `AttitudeHold` / `Ommpc`：姿态与位置控制。
- 建议的线程模型：固定频率指令发送线程 + 独立的遥测/算法线程，二者不共享
  MSP socket 的阻塞路径。
