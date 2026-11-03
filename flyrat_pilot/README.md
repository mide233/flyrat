# flyrat_pilot

实机端程序的**占位骨架**，具体实现留待后续补充。

- 不依赖 ROS，链接与仿真共用的 `libflyrat`。
- 目前只有一个可编译、可运行的 `main()`，运行时逻辑（MSP 链路、50 Hz 指令下发、遥测与失效保护、算法接入）尚未实现。

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
- `flyrat::hardware::Transport` / `make_serial_transport()` / `make_tcp_transport()`：
  MSP 使用的字节流后端，msp_interact 通过抽象接口访问，单测可注入假传输。
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
