# flyrat SITL 仿真（Betaflight SITL × Gazebo Harmonic）

本目录把 **Betaflight SITL** 与 **Gazebo Harmonic** 组合成一个闭环四轴仿真，
用于在开发容器内验证我们自己的自动驾驶算法。

控制链路采用 **真·MSP OVERRIDE**：

- 仿真接收机是 Betaflight 的 `RX_UDP`（UDP 9004）。它负责 **ARM** 和
  **MSP OVERRIDE** 开关，属于安全关键通道，始终由接收机决定。
- 通道 1–4（roll/pitch/yaw/throttle，即 AETR）交给伴飞计算机（我们的
  算法），经 `MSP_SET_RAW_RC` 覆盖写入。
- 反馈只取 Betaflight 的估计状态（MSP 遥测），不引入 Gazebo 真值。

```
伴飞/算法 ──MSP_SET_RAW_RC(AETR)──► TCP 5761
          ◄──MSP 遥测(姿态/IMU/高度/GPS)──
虚拟接收机 ──RC 帧(ARM/MSP-OVERRIDE)──► UDP 9004
Betaflight SITL ──电机 0..1──► UDP 9002 ──► Gazebo 桥接插件
               ◄──FDM 状态──  UDP 9003 ◄──
```

## 目录

- `third_party/` —— git submodule
  - `betaflight`（master）：飞控固件，构建 SITL 目标。
  - `aeroloop_gazebo`（`gz` 分支）：机型 / 世界 / Gazebo 桥接插件。
  - `betaloop`（`gz` 分支）：一键启动器。
- `worlds/betaflight_sitl.sdf` —— 我们调好物理步长的世界（400 Hz）。
- `config/betaflight_sim.txt` —— 飞控 CLI 配置（MSP 覆盖掩码 + 模式映射）。
- `scripts/` —— 构建 / 配置 / 启动脚本。

## 前置依赖

Gazebo Harmonic（`gz sim --version` 应显示 `8.x`）、`cppzmq-dev`
（构建桥接插件）、`python3`。这些已经写进仓库根目录的 `Dockerfile`。

## 1. 构建

```bash
sim/scripts/build_all.sh
```

- `build_betaflight.sh`：先串行 `git submodule update --init`（飞控 Makefile
  会自行拉取固件子模块，并行时会争抢 git 锁），再 `make TARGET=SITL`。
  产物：`third_party/betaflight/obj/main/betaflight_SITL.elf`。
  可用环境变量：`BETAFLIGHT_TARGET`、`JOBS`、`CLEAN=1`。
- `build_plugin.sh`：构建 Gazebo 桥接插件。
  产物：`third_party/aeroloop_gazebo/plugins/build/libBetaflightPlugin.so`。

## 2. 配置飞控（一次性）

先单独启动 SITL，让它在 `third_party/betaflight/` 下生成/读取
`eeprom.bin`，然后写入配置：

```bash
# 终端 A：单独运行 SITL（用于配置）
(cd sim/third_party/betaflight && ./obj/main/betaflight_SITL.elf)

# 终端 B：应用配置
sim/scripts/config_betaflight.py
```

脚本把 `config/betaflight_sim.txt` 里的命令经 TCP 5761 送入 CLI 并执行
`save`。具体设置：

| 命令 | 作用 |
|---|---|
| `set msp_override_channels_mask = 15` | 允许伴飞覆盖通道 1–4（AETR） |
| `aux 0 0 0 1700 2100` | ARM 绑到 AUX1（通道 5），由 9004 接收机驱动 |
| `aux 1 50 1 1700 2100` | MSP OVERRIDE（box 50）绑到 AUX2（通道 6），由 9004 驱动 |

配置持久化在 `eeprom.bin` 中。注意：`save` 后 SITL 会重启，通常表现为
进程退出，需要重新启动。

## 3. 启动仿真

```bash
sim/scripts/launch_sim.sh            # 无 GUI（headless）
SHOW_GAZEBO=1 sim/scripts/launch_sim.sh   # 带 Gazebo GUI
VERBOSE=1 sim/scripts/launch_sim.sh       # 打开启动器详细日志
```

启动器（`betaloop/start.py`）会依次拉起 Gazebo 世界、SITL，并（默认
禁用）网页配置器用的 websockify。

## 端口速查

| 端口 | 方向 | 含义 |
|---|---|---|
| UDP 9002 | SITL → Gazebo | 电机指令 `float[4]`（0..1） |
| UDP 9003 | Gazebo → SITL | FDM 状态（陀螺/加速度/四元数/位置/气压） |
| UDP 9004 | 外部 → SITL | RC 帧 `{double 时间戳; uint16[16]}`（1000–2000） |
| TCP 5761 | 双向 | UART1 MSP / CLI |
| TCP 6761 | 双向 | websockify → 5761（网页配置器，默认禁用） |
