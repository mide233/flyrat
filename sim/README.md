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

飞控配置在 `config/betaflight_sim.txt` 中声明，由脚本经 TCP 5761 送入
CLI 并 `save` 持久化。SITL 以**自身工作目录**解析 `eeprom.bin`，而启动器
（betaloop）以 **ELF 所在目录**（`third_party/betaflight/obj/main`）为工作
目录启动 SITL，所以要用 `configure_fc.sh` 自动完成"起一个临时 SITL → 写入
配置 → 把生成的 `eeprom.bin` 存回 `config/eeprom.bin`"：

```bash
sim/scripts/configure_fc.sh
```

`launch_sim.sh` 每次启动都会把 `config/eeprom.bin` 安装到 ELF 目录，所以
即使 `make clean` 清掉 `obj/`，配置也不会丢。修改了 `betaflight_sim.txt`
后重新运行 `configure_fc.sh` 即可。具体设置：

| 命令 | 作用 |
|---|---|
| `set msp_override_channels_mask = 15` | 允许伴飞覆盖通道 1–4（AETR） |
| `aux 0 0 0 1700 2100` | ARM 绑到 AUX1（通道 5），由 9004 接收机驱动 |
| `aux 1 50 1 1700 2100` | MSP OVERRIDE（box 50）绑到 AUX2（通道 6），由 9004 驱动 |

配置持久化在 `config/eeprom.bin` 中。注意：`save` 后 SITL 会重启，通常表现
为进程退出——这就是 `configure_fc.sh` 里临时 SITL 结束的方式。

> **调试提示**：查看模式映射时请用**不带参数**的 `aux`（列出全部 20 条）。
> `aux <index>` 这类参数不足的写法会**重置**该条映射（`cliAux` 在参数
> 个数不为 4/5/6 时会 `memset` 清空），容易误判成"配置没保存成功"。

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
