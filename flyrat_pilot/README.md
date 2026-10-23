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

- `flyrat::MspClient`：TCP（SITL）或串口（实机）MSP 通信。
- `flyrat::RcReceiver`：UDP 9004 虚拟接收机（仅仿真用）。
- `flyrat::Autopilot` / `flyrat::AttitudeHold`：姿态保持实现。
- `flyrat::Ommpc`：on-manifold MPC（9 维误差状态 + 4 维输入，LTV + OSQP，默认
  20 × 10 ms 前瞻），另有 `flyrat::ThrustModel` / `flyrat::RcMapping` 做执行器建模。
- 位置环在实机上接 VSLAM：按 ENU、`frame_id=map` 提供位置与速度即可，仿真里
  这一路是 Gazebo 真值，语义完全一致。
- 建议的线程模型：固定频率的指令发送线程 + 独立的遥测/算法线程，二者不共享 MSP socket 的阻塞路径
  （仿真侧 `msp_node` 就是这么做的，可直接参考）。

`flyrat_pilot/CMakeLists.txt` 已经链接 `flyrat::flyrat`，添加源文件即可开始实现。
