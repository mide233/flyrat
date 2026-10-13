# Betaflight SITL × Gazebo：仿真与自动驾驶联调

本文介绍如何在开发容器里把 **Betaflight SITL**、**Gazebo Harmonic** 和我们
自己的 **自动驾驶算法** 串成一个闭环，用于验证飞控算法。底层仿真工程的细节
见 [`sim/README.md`](../../sim/README.md)，本文侧重"从零到跑起来"的完整流程
以及踩过的坑。

## 1. 总体结构

```
              ┌──────────────────────── 我们的代码 ────────────────────────┐
              │                                                             │
              │   autopilot/             (纯 C++17，无 ROS 依赖)            │
              │   ros2_ws/src/flyrat_control  (控制节点，调用算法库)         │
              │   ros2_ws/src/flyrat_msp      (MSP 桥接 + 虚拟接收机)        │
              └───────────────┬───────────────────────────┬─────────────────┘
                              │ MSP_SET_RAW_RC (AETR)      │ RC 帧 (ARM / MSP OVERRIDE)
                              ▼                            ▼
                        TCP 5761 (UART1)              UDP 9004 (RX_UDP)
                              │                            │
                    ┌─────────┴────────────────────────────┴─────────┐
                    │              Betaflight SITL (ELf)              │
                    └─────────┬────────────────────────────┬─────────┘
                    电机 0..1 │ UDP 9002                    │ FDM 状态 UDP 9003
                              ▼                             │
                    ┌─────────────────────────────────────┴─────────┐
                    │        Gazebo Harmonic + 桥接插件              │
                    │        (libBetaflightPlugin.so)                │
                    └────────────────────────────────────────────────┘
```

控制模型采用 **真·MSP OVERRIDE**：

- 虚拟接收机（UDP 9004）持有 **ARM** 与 **MSP OVERRIDE** 两个开关，属于安全
  关键通道，永远由接收机（我们的 9004 推送）决定，**绝不**放到 MSP 侧。
- 通道 1–4（roll/pitch/yaw/throttle）由伴飞计算机经 `MSP_SET_RAW_RC` 覆盖。
  飞控侧通过 `set msp_override_channels_mask = 15` 只开放这 4 个通道。
- 反馈只使用 Betaflight 自己的估计状态（MSP 遥测），不引入 Gazebo 真值。
  Gazebo 位姿只在 bring-up 自检里当作独立的地面真值参考。

## 2. 前置依赖

- Gazebo Harmonic（`gz sim --version` 显示 `8.x`）、`cppzmq-dev`（编译桥接
  插件）、`python3`、`websockify`；ROS 2 Jazzy。
- 以上都已写入仓库根目录的 `Dockerfile`，构建镜像后即有。

## 3. 构建仿真

```bash
sim/scripts/build_all.sh
```

产物：

- `sim/third_party/betaflight/obj/main/betaflight_SITL.elf`
- `sim/third_party/aeroloop_gazebo/plugins/build/libBetaflightPlugin.so`

细节（串行拉取固件子模块、`JOBS`、`CLEAN=1` 等）见 `sim/README.md`。

## 4. 一次性配置飞控

飞控配置声明在 `sim/config/betaflight_sim.txt`，由脚本经 TCP 5761 送入 CLI
后 `save` 持久化到 `sim/config/eeprom.bin`：

```bash
sim/scripts/configure_fc.sh
```

| 命令 | 作用 |
|---|---|
| `set msp_override_channels_mask = 15` | 允许伴飞覆盖通道 1–4（AETR） |
| `aux 0 0 0 1700 2100` | ARM 绑到 AUX1（通道 5），接收机驱动 |
| `aux 1 50 1 1700 2100` | MSP OVERRIDE（box 50）绑到 AUX2（通道 6），接收机驱动 |

`launch_sim.sh` 每次启动都会把 `eeprom.bin` 安装到 ELF 所在目录（SITL 以自身
工作目录解析它），所以 `make clean` 也不会丢配置。

## 5. 启动仿真

```bash
sim/scripts/launch_sim.sh                 # 无界面
SHOW_GAZEBO=1 sim/scripts/launch_sim.sh   # 带 Gazebo GUI
```

## 6. 自检（不依赖 ROS 2）

先确认"飞控 ↔ 仿真"这条链路本身没问题：

```bash
python3 sim/scripts/bringup_test.py --takeoff
```

脚本会依次验证：MSP 链路、MSP OVERRIDE 确实在接管 AETR、AUX1（ARM）仍由接收机
驱动、解锁、在油门下拉升（用 `gz model -m iris -p` 位姿判定）、以及上锁。

各脚本的作用：

| 脚本 | 作用 |
|---|---|
| `sim/scripts/flyrat_msp.py` | MSP v1 客户端（`set_raw_rc` + 遥测），TCP 5761 |
| `sim/scripts/rc_receiver.py` | 虚拟接收机，UDP 9004（AUX1=ARM，AUX2=MSP OVERRIDE） |
| `sim/scripts/bringup_test.py` | 端到端自检 |

## 7. 编译并运行 ROS 2 层

### 7.1 编译

```bash
# 纯算法库（无 ROS）
cmake -S autopilot -B autopilot/build -DCMAKE_BUILD_TYPE=Release
cmake --build autopilot/build
ctest --test-dir autopilot/build

# ROS 2 包
cd ros2_ws
colcon build --symlink-install
```

> 在 zsh 里请用 `bash -c 'source /opt/ros/jazzy/setup.bash && ...'` 包一层再
> source，直接 source 可能出错。

### 7.2 运行

```bash
# 终端 A：仿真
sim/scripts/launch_sim.sh
# 终端 B：控制
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch flyrat_control flyrat_sitl.launch.py auto_takeoff:=true
```

`auto_takeoff:=true` 会走一遍"解锁 → 缓慢加油门 → 悬停 → 自动上锁"的流程。

### 7.3 接口一览

| 名称 | 类型 | 说明 |
|---|---|---|
| `/flyrat/rc_override` | `std_msgs/Float64MultiArray`（订阅） | 4 个 µs 值，**接收机原始顺序** `[roll, pitch, throttle, yaw]` |
| `/flyrat/attitude` | `geometry_msgs/Vector3Stamped`（发布） | roll/pitch/yaw（rad） |
| `/flyrat/rc` | `std_msgs/UInt16MultiArray` | 飞控内部通道（原始接收机顺序） |
| `/flyrat/motor` | `std_msgs/UInt16MultiArray` | 电机输出 |
| `/flyrat/imu` | `sensor_msgs/Imu` | 陀螺 / 加速度 |
| `/flyrat/status` | `std_msgs/Int32MultiArray` | `[armed, msp_override, cycletime, sensors]` |
| `/flyrat/armed` | `std_msgs/Bool` | 解锁状态 |
| `/flyrat/altitude` | `std_msgs/Float32` | 高度（m，无效时为 NaN） |
| `/flyrat/battery` | `std_msgs/Float32` | 电压（V） |
| `/flyrat/set_arm` | `std_srvs/SetBool`（服务） | 拨动 ARM 开关（AUX1） |
| `/flyrat/set_msp_override` | `std_srvs/SetBool`（服务） | 拨动 MSP OVERRIDE 开关（AUX2） |

常用参数：`flyrat_msp` 的 `host` / `msp_port`(5761) / `rc_port`(9004) /
`rate`(50) / `telemetry_rate`(10)；`flyrat_control` 的 `control_rate`(50) /
`state_timeout`(2.0) / `takeoff_throttle`(0.7) / `flight_seconds`(5)。

## 8. 算法库（`autopilot/`）

与 ROS 解耦的纯 C++17 库，接口刻意保持极简：

- `flyrat::State`：姿态（rad）、角速度、高度、电压、是否解锁等估计量。
- `flyrat::Setpoint`：期望姿态 / 偏航角速度 / 油门。
- `flyrat::Command`：归一化指令 `roll/pitch/yaw ∈ [-1,1]`、`throttle ∈ [0,1]`。
- `flyrat::Autopilot`：`reset()` + `update(state, setpoint, dt) -> Command`。
- `flyrat::AttitudeHold`：示例实现（P 姿态控制 + 偏航角速度阻尼 + 输出限幅）。

`flyrat_control` 通过 `add_subdirectory(.../autopilot)` 直接链接该库，因此算法
可以脱离 ROS 单独编译和测试。

## 9. 端口速查

| 端口 | 方向 | 含义 |
|---|---|---|
| UDP 9002 | SITL → Gazebo | 电机指令 `float[4]`（0–1） |
| UDP 9003 | Gazebo → SITL | FDM 状态（陀螺 / 加速度 / 四元数 / 位置 / 气压） |
| UDP 9004 | 外部 → SITL | RC 帧 `{double 时间戳; uint16[16]}`（1000–2000） |
| TCP 5761 | 双向 | UART1，MSP / CLI（SITL 只允许**一个**客户端） |
| TCP 6761 | 双向 | websockify → 5761（网页配置器，默认禁用） |

## 10. 常见问题排查

**拉起后不起飞 / 油门没反应**

- 先用 `bringup_test.py --takeoff` 排除 ROS 2 层的因素。
- MSP OVERRIDE 需要 **fresh** 帧：`MSP_SET_RAW_RC` 必须持续推送（固件的新鲜度
  窗口是 300 ms；本项目按 50 Hz 推送 16 通道），一旦停发，飞控会退回接收机
  通道。所以"发送"必须和"读取遥测"分离到不同线程，读遥测不能把发送卡住。
- 解锁必须 **先低后高**：AUX1 若一开始就是高电平，飞控会因 `NOT_DISARMED`
  拒绝解锁。请先保持 AUX1 低，等链路稳定后再拉高。
- 油门低起飞前不要提前加油门，否则会触发 `RUNAWAY THROTTLE` 而拒绝解锁。

**电机输出正常但飞机不升（无升力）**

- 自检脚本里**不要**在油门关键循环里高频调用外部命令（例如每帧执行
  `gz model ... -p`）。每次调用都会 fork 一个进程，几百次/秒会把
  Gazebo/SITL 的实时同步拖垮，导致旋翼不产生升力。采样位姿要降频（如 0.5 s
  一次）。
- 旋翼必须真的转：升力由 `gz-sim` 的 LiftDrag 按 `ω²` 计算，转速过低无升力。

**MSP 遥测时不时超时**

- 飞机真正在飞时，SITL 的 MSP 往返偶尔会变慢；因此发送线程与遥测线程必须
  分开，且 `set_raw_rc` **不要**在每帧等待 ACK（会拖慢到 ~20 Hz）。
- SITL 的 5761 只允许一个 TCP 客户端。若出现 `ConnectionReset`/`Broken pipe`，
  多半是有残留的 `msp_node` 还占着连接——先杀掉旧节点。

**`state telemetry stale` 反复出现**

- 通常只是启动瞬间（还没收到第一帧姿态）。若持续出现，检查 `/flyrat/attitude`
  是否在发布（`ros2 topic hz /flyrat/attitude`）。
- 控制节点在遥测超时会**失效保护**：油门归零、姿态回中。

**日志里一直刷 `Arming disabled: RXLOSS`**

- SITL 的 stdout 经启动器管道缓冲，时间戳会滞后，且这行在能正常飞行的运行里
  也会出现，不能单凭它判断故障。请以 `bringup_test.py` / 位姿判定为准。

**`aux` 配置像是没保存**

- 用**不带参数**的 `aux` 查看全部映射；`aux <index>` 这类参数不足的写法会
  **重置**该条映射（`cliAux` 在参数个数不为 4/5/6 时 `memset` 清空）。

**清理进程**

- 结束仿真：找到 `betaloop/start.py` 的进程组并 `kill -TERM -<pgid>`。
- **不要**用 `pkill -f betaloop/start.py` 之类的写法：它的 SIGTERM 处理器会杀掉
  整个进程组，可能连带把当前 shell 一起干掉。同理，任何 `pkill`/`grep` 的关键字
  都不要和命令行的其它文本重复（包括 `echo` 里的字面量），否则会误杀自己。
