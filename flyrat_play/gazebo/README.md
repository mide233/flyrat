# flyrat SITL 仿真（Betaflight SITL × Gazebo Harmonic）

本目录把 **Betaflight SITL** 与 **Gazebo Harmonic** 组合成一个闭环四轴仿真，
用于在开发容器内验证我们自己的自动驾驶算法。

控制链路采用 **真·MSP OVERRIDE**：

- 仿真接收机是 Betaflight 的 `RX_UDP`（UDP 9004）。它负责 **ARM** 和
  **MSP OVERRIDE** 开关，属于安全关键通道，始终由接收机决定。
- 通道 1–4（roll/pitch/yaw/throttle，即 AETR）交给伴飞计算机（我们的
  算法），经 `MSP_SET_RAW_RC` 覆盖写入。
- 反馈分两路：姿态/角速度取飞控的 MSP 遥测；**位置/速度默认取 Gazebo 里程计
  真值（60 Hz）**，由 `flyrat_msp` 转成 `/flyrat/position`、`/flyrat/velocity`。
  实机上把这一路换成 VSLAM 即可；`use_ground_truth:=false` 可退回飞控的 10 Hz
  GPS 通路（仅用于自检）。

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
- `worlds/betaflight_sitl.sdf` —— 我们调好物理步长的世界（400 Hz），含起飞坪、穿越门、
  集装箱、建筑、树木、矮墙和抬高的降落平台等静态场景。
- `models/betaflight_iris/` —— 本地机型覆盖：修正电机顺序，并在机身挂了一个前视相机
  （灰度 1280×800@30 Hz，由 `flyrat_sitl.launch.py` 默认桥接，或 `.script/camera.sh` 单独启动，转发到 ROS 2 `/flyrat/camera/*`）。
- `config/betaflight_sim.txt` —— 飞控 CLI 配置（MSP 覆盖掩码 + 模式映射）。
- `scripts/` —— 配置、启动与自检脚本；构建脚本在仓库根目录的 `.script/`。

## 前置依赖

Gazebo Harmonic（`gz sim --version` 应显示 `8.x`）、`cppzmq-dev`
（构建桥接插件）、`python3`。这些已经写进仓库根目录的 `Dockerfile`。

## 1. 构建

只构建仿真相关产物：

```bash
.script/build_betaflight.sh
.script/build_plugin.sh
```

构建仓库全部产物（算法库、实机程序、SITL、插件、ROS 2 包）：

```bash
.script/build_all.sh
```

- `build_betaflight.sh`：先串行 `git submodule update --init`（飞控 Makefile
  会自行拉取固件子模块，并行时会争抢 git 锁），再 `make TARGET=SITL`。
  产物：`third_party/betaflight/obj/main/betaflight_SITL.elf`。
  可用环境变量：`BETAFLIGHT_TARGET`、`JOBS`、`CLEAN=1`。
- `build_plugin.sh`：先对上游子模块打补丁（`flyrat_play/gazebo/patches/*.patch`，见下），
  再构建 Gazebo 桥接插件。
  产物：`third_party/aeroloop_gazebo/plugins/build/libBetaflightPlugin.so`。

清理构建产物（含 SITL ELF、插件、colcon 与 CMake 构建目录）：

```bash
.script/clean_build.sh --dry-run   # 先看会删什么
.script/clean_build.sh             # 全部清理
.script/clean_build.sh --keep-sim  # 保留 SITL ELF 与插件
```

> **关于 `patches/`**：上游插件在物理步进内用 `select` 等待电机指令包最多
> **1000 ms**（`BetaflightPlugin.cc` 的 `waitMs = 1000`）。一旦丢一个包，
> 整个仿真会卡在这一步，实时率（real_time_factor）从 1.0 掉到 ~0.5%，
> 表现为"飞机不升 / MSP 遥测超时 / 下一次运行连不上"。补丁把它降到 2 ms，
> 只影响等待上限，不改变控制逻辑。补丁由 `build_plugin.sh` 幂等应用。

## 2. 配置飞控（一次性）

飞控配置在 `config/betaflight_sim.txt` 中声明，由脚本经 TCP 5761 送入
CLI 并 `save` 持久化。SITL 以**自身工作目录**解析 `eeprom.bin`，而启动器
（betaloop）以 **ELF 所在目录**（`third_party/betaflight/obj/main`）为工作
目录启动 SITL，所以要用 `configure_fc.sh` 自动完成"起一个临时 SITL → 写入
配置 → 把生成的 `eeprom.bin` 存回 `config/eeprom.bin`"：

```bash
flyrat_play/gazebo/scripts/configure_fc.sh
```

`launch_sim.sh` 每次启动都会把 `config/eeprom.bin` 安装到 ELF 目录，所以
即使 `make clean` 清掉 `obj/`，配置也不会丢。修改了 `betaflight_sim.txt`
后重新运行 `configure_fc.sh` 即可。具体设置：

| 命令 | 作用 |
|---|---|
| `set msp_override_channels_mask = 15` | 允许伴飞覆盖通道 1–4（AETR） |
| `aux 0 0 0 1700 2100` | ARM 绑到 AUX1（通道 5），由 9004 接收机驱动 |
| `aux 1 50 1 1700 2100` | MSP OVERRIDE（box 50）绑到 AUX2（通道 6），由 9004 驱动 |
| `rates_type = BETAFLIGHT` + `*_rc_rate = 200` + `*_expo = 0` + `*_srate = 0` | 线性速率曲线 400 deg/s，控制器用一个比例系数就能把角速度换算成摇杆 |
| `yaw_motors_reversed = ON` | 本机型电机转向与默认混控相反，不反转 yaw 混控会正反馈到翻滚 |
| `debug_mode = ALTITUDE` | `MSP_DEBUG` 暴露高度估计通道，排查高度问题用 |

配置持久化在 `config/eeprom.bin` 中。注意：`save` 后 SITL 会重启，通常表现
为进程退出——这就是 `configure_fc.sh` 里临时 SITL 结束的方式。

> **调试提示**：查看模式映射时请用**不带参数**的 `aux`（列出全部 20 条）。
> `aux <index>` 这类参数不足的写法会**重置**该条映射（`cliAux` 在参数
> 个数不为 4/5/6 时会 `memset` 清空），容易误判成"配置没保存成功"。

## 3. 启动仿真

```bash
flyrat_play/gazebo/scripts/launch_sim.sh            # 带 Gazebo GUI（默认）
HIDE_GAZEBO=1 flyrat_play/gazebo/scripts/launch_sim.sh   # 无界面（headless）
VERBOSE=1 flyrat_play/gazebo/scripts/launch_sim.sh       # 打开启动器详细日志
```

启动器（`betaloop/start.py`）会依次拉起 Gazebo 世界、SITL，并（默认
禁用）网页配置器用的 websockify。

## 4. 自动驾驶联调（ROS 2 / 实机程序 + 算法库）

仿真本身跑通后，可以接上我们自己的算法做闭环。仓库分为三层：

- `libflyrat/`：与 ROS 解耦的纯 C++17 算法库 + Betaflight MSP/UDP 传输，仿真与实机共用。
  控制器包括姿态保持与 on-manifold MPC（`flyrat::Ommpc`）。
- `flyrat_play/ros2_ws/`：仿真用的 ROS 2 层
  - `flyrat_msp`：MSP 桥接与虚拟接收机（`msp_node`），另外把 Gazebo 里程计
    转成 `/flyrat/position`、`/flyrat/velocity`（60 Hz）。
  - `flyrat_control`：控制节点（`control_node`，`controller:=attitude_hold|ommpc`）。
- `flyrat_pilot/`：实机端程序骨架（无 ROS，暂未实现运行逻辑），预留串口/TCP MSP 与算法接入。

MPC 闭环（8 字轨迹）的启动方式：

```bash
ros2 launch flyrat_control flyrat_sitl.launch.py \
  controller:=ommpc auto_takeoff:=true flight_seconds:=120 msp_override:=false
```

不依赖 ROS 的链路自检：`bringup_test.py`（覆盖 / 解锁 / 拉升）。

完整流程、接口表、MPC 调参与排查清单见
[`docs/zh-cn/betaflight_sitl_gazebo.md`](../../../docs/zh-cn/betaflight_sitl_gazebo.md)。

可视化：另开一个终端运行 `.script/foxglove.sh`，在主机上用 Foxglove
Studio 连接 `ws://<主机地址>:8765`，即可看到 `/flyrat/*` 话题。

机身前视相机：`flyrat_sitl.launch.py` 默认就把桥接一起拉起来（`camera:=false` 可关，
`camera_prefix:=/cam` 可换命名空间），图像会出现在
`/flyrat/camera/image_raw`（`sensor_msgs/Image`，`mono8`，1280×800，≈30 Hz），
`/flyrat/camera/camera_info` 同时发布，`/clock` 也会被桥接（图像的
`header.stamp` 是仿真时间）。这几个话题都会由 Foxglove 一并转发。

定位：默认用 Gazebo 的里程计真值（`model.sdf` 的 `OdometryPublisher` →
`ros_gz_bridge` → `/flyrat/ground_truth/odometry` → `msp_node` 的 60 Hz 定时器 →
`/flyrat/position`、`/flyrat/velocity`，`frame_id=map`）。实机上换成 VSLAM，接口不变。
`use_ground_truth:=false` 退回飞控 GPS + `flyrat::LocalEstimator` 的 10 Hz 通路，
只用于自检。坐标契约、两条数据链和 IMU 比力补丁见
[`docs/zh-cn/betaflight_sitl_gazebo.md`](../../../docs/zh-cn/betaflight_sitl_gazebo.md) 7.6 节。

如果只想看图像、不启动控制节点，再单独跑 `.script/camera.sh`（不要和 launch 同时开）。

画面闪烁几乎都是**残留的旧仿真栈**导致的（两个 `gz sim` 同名话题，一个 bridge
在两份画面间切换）：`launch_sim.sh` 现在检测到已有栈会直接拒绝启动，先
`pgrep -af 'betaloop/start.py'` 确认只有一个，必要时 kill 掉再重来。

## 端口速查

| 端口 | 方向 | 含义 |
|---|---|---|
| UDP 9002 | SITL → Gazebo | 电机指令 `float[4]`（0..1） |
| UDP 9003 | Gazebo → SITL | FDM 状态（陀螺/加速度/四元数/位置/气压） |
| UDP 9004 | 外部 → SITL | RC 帧 `{double 时间戳; uint16[16]}`（1000–2000） |
| TCP 5761 | 双向 | UART1 MSP / CLI |
| TCP 6761 | 双向 | websockify → 5761（网页配置器，默认禁用） |
| TCP 8765 | 双向 | Foxglove bridge（`.script/foxglove.sh`） |
