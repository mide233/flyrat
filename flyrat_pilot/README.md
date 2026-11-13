# flyrat_pilot

实机端程序，不依赖 ROS。包含三个可执行文件：

- `flyrat_pilot`：在设备本地解码双目 MJPEG、读取 MSP IMU，运行 SchurVINS
  双目惯性里程计，通过 TCP 把 `T_world_body` 位姿发给主机。
- `flyrat_sensor_forwarder`：把原始 IMU、ToF 和双目 MJPEG 转发给主机 ROS 2，
  用于录包和开发机算法验证。
- `flyrat_raw_recorder`：为重新标定直接录制 MSP IMU 原始计数和原生 MJPEG，
  不应用旧内参、轴映射、时间偏移或去畸变。

主机侧 `flyrat_pilot_bridge` 把位姿流转换成 `nav_msgs/Odometry`、
`nav_msgs/Path` 和 TF，供 Foxglove 查看。

## 1. 快速构建

宿主机构建（libflyrat + pilot + forwarder + raw recorder）：

```bash
./.script/build_all.sh --lib --pilot
```

产物：

```text
build/host/pilot/flyrat_pilot
build/host/pilot/flyrat_sensor_forwarder
build/host/pilot/flyrat_raw_recorder
```

配置自检（不打开相机和串口）：

```bash
./build/host/pilot/flyrat_pilot --check-config \
  --camera-config libflyrat/configs/camera/uvc_640x480.yaml \
  --imu-config flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml
```

## 2. 交叉编译到泰山派

设备是 RK3566 + Buildroot，没有 OpenCV/glog/yaml-cpp。先准备带最小静态
OpenCV 的 aarch64 sysroot，再执行：

```bash
./.script/build_pilot_arm.sh
```

产物：

```text
build/arm/flyrat_pilot
build/arm/decode_bench
build/arm/flyrat_sensor_forwarder
build/arm/flyrat_raw_recorder
```

`build/arm/static/flyrat_sensor_forwarder` 是 `--static-forwarder` 生成的全静态
版本。脚本默认使用 `/opt/sysroots/arm64`，可用 `FLYRAT_SYSROOT` 覆盖；默认
`JOBS=4`，避免在内存较小的构建机上触发 OOM。完整构建和 sysroot 说明见
[`docs/agent/build_and_run.md`](../docs/agent/build_and_run.md)。

## 3. 机载 SLAM

一条命令完成推送、adb forward、设备端 SLAM、主机 bridge 和可选 Foxglove：

```bash
./.script/bringup.sh slam start_foxglove:=true
```

常用参数：

```text
fps:=N             交付相机帧率，默认 25；采集始终为传感器最大 fps
decode:=cpu|mpp    解码器，脚本默认 mpp，失败自动回退 CPU
max_frames:=N      处理 N 帧后退出，0 表示一直运行
record:=DIR        同时录制相机 JPEG + IMU
replay:=DIR        跳过相机/MSP，重放录制的 bag
trace:=DIR         写出逐帧耗时 CSV，退出时自动拉回主机
```

运行中发送 `SIGUSR1` 会重建 SchurVINS 前端，`SIGINT`/`SIGTERM` 会停止采集、
关闭位姿服务并保存轨迹。

传感器录制与回放（`--record` / `--replay`）用于在设备上复现算法问题；bag 是
`<DIR>/sensor_bag.bin`，回放自动重映射时间并退出。格式和限制见
[`docs/agent/hardware/sensor_forwarding.md`](../docs/agent/hardware/sensor_forwarding.md#传感器录制与回放)。

主机话题：

```text
/flyrat/odometry/odometry   nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path       nav_msgs/Path
/flyrat/odometry/status     diagnostic_msgs/DiagnosticStatus，1 Hz
```

## 4. 原始标定录制

```bash
./build/host/pilot/flyrat_raw_recorder --self-test
.script/bringup.sh raw check:=true
.script/bringup.sh raw duration:=10m fps:=20 capture_fps:=60 export:=true
# 边录边用 Foxglove 预览
.script/bringup.sh raw duration:=10m preview:=true
```

相机始终按 `capture_fps`（默认 60）请求采集，再按 `fps` 丢帧交付。输出
`raw_sensors.bin`，用 `python3 tools/raw_export.py` 导出 Kalibr 目录。加
`preview:=true` 会同时转发到主机，默认 `imu_corrections:=false`，不影响
bag。完整流程、参数和导出格式以
[`docs/agent/calibration/raw_recording.md`](../docs/agent/calibration/raw_recording.md)
为准。

## 5. 位姿协议

设备回传 64 字节 `PosePayload`，外层复用
`libflyrat/include/flyrat/sensors/protocol.hpp` 的 28 字节帧头：

```text
uint8  type = 6 (kPose)
double position[3]       T_world_body 平移，米
double orientation[4]    x, y, z, w
uint8  valid
uint8  state             0..3，对应 odometry::State
uint8  reserved[6]
```

`device_time_ns` 是 Unix epoch 纳秒。body 为 `imu_link`（FLU），world 原点在
首次初始化位置。服务只保留最新位姿，主机断线重连不会补发历史帧。

## 6. 行为与限制

- 相机始终按传感器支持的最高帧率采集（1280x480 MJPEG 为 60 fps），再按
  `--fps` 丢弃多余帧；自动曝光上限因此是 1/60 s，暗处不会退到长曝光。
  左右目来自同一帧、时间戳相同。
- 默认交付 25 fps。当前 NEON + 并行重投影下 `track()` 约 16–17 ms，帧预算
  40 ms；30 fps 约 15–16 ms。性能数据、`--trace` 和 MPP 解码细节见
  [`docs/agent/odometry/performance.md`](../docs/agent/odometry/performance.md)。
- 启用并行重投影时不要用 `--slam-cpu` 固定 SLAM 线程，工作线程会继承单核
  亲和性。
- 只在桌面短时场景验证，尚未做飞行、长距离和长期漂移验证。
- SchurVINS 为 **GPL-3.0**，`flyrat_pilot` 链接后组合二进制受 GPL 约束；
  libflyrat 自身源文件仍为 MIT。

## 7. 更多文档

- 构建、ROS 2、SITL、实机启动：[`docs/agent/build_and_run.md`](../docs/agent/build_and_run.md)
- 传感器转发、rosbag 录制、splitter、Kalibr：[`docs/agent/hardware/sensor_forwarding.md`](../docs/agent/hardware/sensor_forwarding.md)
- 性能与解码调研：[`docs/agent/odometry/performance.md`](../docs/agent/odometry/performance.md)
- 项目状态与实测指标：[`docs/agent/status/project_status.md`](../docs/agent/status/project_status.md)
