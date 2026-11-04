# flyrat_pilot

实机端程序的骨架与传感器转发程序。

- 不依赖 ROS，链接与仿真共用的 `libflyrat`。
- 主飞行逻辑仍是占位 `main()`（MSP 指令下发、遥测、失效保护、算法接入尚未实现）。
- `flyrat_sensor_forwarder` 已实现实机传感器转发，接入方法见
  [`docs/zh-cn/sensor_forwarding.md`](../docs/zh-cn/sensor_forwarding.md)。
- 项目整体进度与实测指标见
  [`docs/zh-cn/project_status.md`](../docs/zh-cn/project_status.md)。

## 构建

```bash
cmake -S flyrat_pilot -B flyrat_pilot/build -DCMAKE_BUILD_TYPE=Release
cmake --build flyrat_pilot/build -j
./flyrat_pilot/build/flyrat_pilot
```

## 预留的接入点

- `flyrat::MspInteract`：异步 MSP v1 客户端，支持 TCP（SITL）与串口（实机）；
  内部 I/O 线程按配置轮询姿态 / IMU / 状态 / 高度 / GPS，专用分发线程执行回调，
  同时提供每类数据的最新快照。断线自动重连，运行期错误通过 `LinkStats` 与
  link 回调返回；`send_rc()` 只写单槽邮箱，不等待 ACK。
- `flyrat::hardware::TofArray`：4 颗 VL53L5CX 共用 i2c2 的用户态驱动。
  默认 4x4@60 Hz、只保留 distance/target_status/nb_target（128 B/帧），
  地址硬编码为 `0x60/0x62/0x64/0x66`，不再单独配置；LPn 默认
  `108/114/101/98`，INT 默认 `107/104/100/97`（下/后/左/右）。
  每次启动逐颗释放 LPn，自动探测传感器残留的旧地址并改回固定地址；
  四颗从启动到监听约 7.2 秒。
- `flyrat::hardware::Imu`：读取 `MspInteract` 的 IMU 最新快照，可带最大年龄检查。
- `flyrat::hardware::StereoCamera`：UVC 双目相机 V4L2 采集，按左右拼接帧拆成两个
  同时间戳的视图。`StereoFormat::kYuyv` 为原始 YUYV（默认 `1280x480@15`，左右各
  640x480）；`StereoFormat::kMjpeg` 使用 Rockchip MPP 硬解，推荐 `1280x480@60`
  （左右各 640x480）或 `640x240@60`。MJPEG 的 `grayscale=true` 输出左右 GRAY8，
  `grayscale=false` 输出左右 NV12 彩色（Y 平面后接 UV 平面）。MJPEG 解码通过运行时
  `dlopen` 加载 GStreamer，需要目标端有 `mppjpegdec`，因此使用该模式的程序需动态链接。
- `flyrat::hardware::Transport` / `make_serial_transport()` / `make_tcp_transport()`：
  MSP 使用的字节流后端，单测可注入假传输。
- `flyrat::RcReceiver`：UDP 9004 虚拟接收机（仅仿真用）。
- `flyrat::Autopilot` / `flyrat::AttitudeHold`：姿态保持实现。
- `flyrat::Ommpc`：on-manifold MPC（9 维误差状态 + 4 维输入，LTV + OSQP，默认
  20 × 10 ms 前瞻），另有 `flyrat::ThrustModel` / `flyrat::RcMapping` 做执行器建模。
- 位置环在实机上接 VSLAM：按 ENU、`frame_id=map` 提供位置与速度即可，仿真里
  这一路是 Gazebo 真值，语义完全一致。
- 建议的线程模型：固定频率的指令发送线程 + 独立的遥测/算法线程，二者不共享 MSP socket 的阻塞路径
  （仿真侧 `msp_node` 就是这么做的，可直接参考）。

`flyrat_pilot/CMakeLists.txt` 已经链接 `flyrat::flyrat`、`flyrat::flyrat_msp_interact`
和 `flyrat::flyrat_hardware`，添加源文件即可开始实现。
