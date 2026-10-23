# Betaflight SITL × Gazebo：仿真与自动驾驶联调

本文介绍如何在开发容器里把 **Betaflight SITL**、**Gazebo Harmonic** 和我们
自己的 **自动驾驶算法** 串成一个闭环，用于验证飞控算法。底层仿真工程的细节
见 [`flyrat_play/gazebo/README.md`](../../flyrat_play/gazebo/README.md)，本文侧重"从零到跑起来"的完整流程
以及踩过的坑。

## 1. 总体结构

```
┌──────────────── libflyrat/（纯 C++17，无 ROS 依赖）────────────────┐
│  算法：State / Setpoint / Command / Autopilot / AttitudeHold        │
│  传输：MspClient（TCP 或串口 MSP）+ RcReceiver（UDP 9004）          │
└───────────────┬───────────────────────────────────┬───────────────┘
                │                                   │
   ┌────────────▼──────────────┐      ┌─────────────▼──────────────┐
   │ flyrat_play/ros2_ws/      │      │ flyrat_pilot/              │
   │   flyrat_msp   (MSP 桥)   │      │   实机程序（无 ROS，占位）  │
   │   flyrat_control (控制)   │      │   预留串口 / TCP MSP 接入   │
   └────────────┬──────────────┘      └─────────────┬──────────────┘
                ▼                                   │ 实机
   ┌──────────────────────────────┐                 ▼
   │ Betaflight SITL × Gazebo     │        Betaflight 飞控（串口）
   └──────────────────────────────┘
```

仿真端与实机端共用同一份 `libflyrat`，区别只在传输后端：仿真走 TCP 5761 +
UDP 9004，实机走串口 MSP。

控制模型采用 **真·MSP OVERRIDE**：

- 虚拟接收机（UDP 9004）持有 **ARM** 与 **MSP OVERRIDE** 两个开关，属于安全
  关键通道，永远由接收机（我们的 9004 推送）决定，**绝不**放到 MSP 侧。
- 通道 1–4（roll/pitch/yaw/throttle）由伴飞计算机经 `MSP_SET_RAW_RC` 覆盖。
  飞控侧通过 `set msp_override_channels_mask = 15` 只开放这 4 个通道。
- 反馈只使用 Betaflight 自己的估计状态（MSP 遥测），不引入 Gazebo 真值。
  Gazebo 位姿只在 bring-up 自检里当作独立的地面真值参考。
- 相机是**独立的单向可视化通路**（Gazebo sensors → `ros_gz_bridge` → ROS 2
  `/flyrat/camera/*`），不参与控制回路，见 7.5。

## 2. 前置依赖

- Gazebo Harmonic（`gz sim --version` 显示 `8.x`）、`cppzmq-dev`（编译桥接
  插件）、`python3`、`websockify`；ROS 2 Jazzy。
- `libflyrat` 的控制器需要 **Eigen3** 和 **OSQP**（`libeigen3-dev`、
  `ros-$ROS_DISTRO-osqp-vendor`，后者提供 0.6.2 的 `libosqp.so` 和 CMake 配置）。
  缺 OSQP 时 CMake 会明确报错。
- 以上都已写入仓库根目录的 `Dockerfile`，构建镜像后即有。

## 3. 构建

一次构建全部产物（算法库、实机程序、SITL、桥接插件、ROS 2 包）：

```bash
.script/build_all.sh
```

也可以只构建其中一部分：

| 脚本 | 产物 |
|---|---|
| `.script/build_libflyrat.sh` | `libflyrat/build/libflyrat.a`（并运行单元测试） |
| `.script/build_flyrat_pilot.sh` | `flyrat_pilot/build/flyrat_pilot` |
| `.script/build_betaflight.sh` | `.../betaflight/obj/main/betaflight_SITL.elf` |
| `.script/build_plugin.sh` | `.../aeroloop_gazebo/plugins/build/libBetaflightPlugin.so` |
| `.script/build_ros2.sh` | `flyrat_play/ros2_ws/install/`（`flyrat_msp`、`flyrat_control`） |

常用参数：

- `--clean`：先删除对应构建目录再编译（`build_betaflight.sh` 用环境变量 `CLEAN=1`）。
- `--no-sim` / `--no-ros`（仅 `build_all.sh`）：跳过仿真或 ROS 2 部分。
- `--no-tests` / `--debug`（`build_libflyrat.sh`、`build_flyrat_pilot.sh`）：跳过单元测试 / 编译 Debug 版。
- `build_ros2.sh` 的额外参数会透传给 colcon，例如 `--packages-select flyrat_msp`。
- `JOBS`：并行编译数，默认 `nproc`。

仿真侧的完整产物：

- `flyrat_play/gazebo/third_party/betaflight/obj/main/betaflight_SITL.elf`
- `flyrat_play/gazebo/third_party/aeroloop_gazebo/plugins/build/libBetaflightPlugin.so`

细节（串行拉取固件子模块、`JOBS`、`CLEAN=1` 等）见 `flyrat_play/gazebo/README.md`。

清理构建产物：

```bash
.script/clean_build.sh --dry-run   # 只列出将删除的路径
.script/clean_build.sh             # 删除 SITL/插件/colcon/CMake 产物与缓存
.script/clean_build.sh --keep-sim  # 保留 SITL ELF 与桥接插件
```

> `build_plugin.sh` 会先对上游插件子模块应用 `flyrat_play/gazebo/patches/*.patch`。其中
> `aeroloop_gazebo-motor-recv-timeout.patch` 把物理步进内等待电机指令包的
> 上限从 1000 ms 降到 2 ms——这是**必需修复**，否则丢一个包就会让整个仿真
> 卡死（详见第 10 节）。

## 4. 一次性配置飞控

飞控配置声明在 `flyrat_play/gazebo/config/betaflight_sim.txt`，由脚本经 TCP 5761 送入 CLI
后 `save` 持久化到 `flyrat_play/gazebo/config/eeprom.bin`：

```bash
flyrat_play/gazebo/scripts/configure_fc.sh
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
flyrat_play/gazebo/scripts/launch_sim.sh                 # 带 Gazebo GUI（默认）
HIDE_GAZEBO=1 flyrat_play/gazebo/scripts/launch_sim.sh   # 无界面（headless）
```

世界（`worlds/betaflight_sitl.sdf`）除地面和太阳外还包含一组静态场景，方便验证
避障、低空飞行和定点降落，全部由基本几何体构成，不依赖外部 mesh：

- 起飞坪（视觉）、6 m 外高 3.2 m 的穿越门、两个集装箱、三根细杆；
- 6 栋 4–10 m 高的建筑、6 棵树、两道 1.5 m 高的矮墙；
- 一个抬高 0.6 m 的降落平台（`(6, 6)`），可作为精确降落目标。

原点附近约 3 m 内保持净空，起飞/悬停行为与最小世界一致。场景均为 `static`，
实测飞行时 `real_time_factor` 仍稳定在 ~1.0。

## 6. 自检（不依赖 ROS 2）

先确认"飞控 ↔ 仿真"这条链路本身没问题：

```bash
python3 flyrat_play/gazebo/scripts/bringup_test.py --takeoff
```

脚本会依次验证：MSP 链路、MSP OVERRIDE 确实在接管 AETR、AUX1（ARM）仍由接收机
驱动、解锁、在油门下拉升（用 `gz model -m iris -p` 位姿判定）、以及上锁。

各脚本的作用：

| 脚本 | 作用 |
|---|---|
| `flyrat_play/gazebo/scripts/flyrat_msp.py` | MSP v1 客户端（`set_raw_rc` + 遥测），TCP 5761 |
| `flyrat_play/gazebo/scripts/rc_receiver.py` | 虚拟接收机，UDP 9004（AUX1=ARM，AUX2=MSP OVERRIDE） |
| `flyrat_play/gazebo/scripts/bringup_test.py` | 端到端自检 |

## 7. 编译并运行 ROS 2 层

### 7.1 编译

```bash
# 纯算法库（无 ROS）+ 单元测试
.script/build_libflyrat.sh

# ROS 2 包（脚本内部会 source /opt/ros/jazzy/setup.bash）
.script/build_ros2.sh
```

两者都可以加 `--clean` 从零开始；`build_ros2.sh` 的额外参数会透传给 colcon。

> 手动执行时，在 zsh 里请用 `bash -c 'source /opt/ros/jazzy/setup.bash && ...'`
> 包一层再 source，直接 source 可能出错。

### 7.2 运行

```bash
# 终端 A：仿真
flyrat_play/gazebo/scripts/launch_sim.sh
# 终端 B：控制
source /opt/ros/jazzy/setup.bash
source flyrat_play/ros2_ws/install/setup.bash
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
| `/flyrat/altitude` | `std_msgs/Float32` | 气压/EKF 高度（m，无效时为 NaN） |
| `/flyrat/position` | `geometry_msgs/Vector3Stamped`（发布） | 本地 ENU 位置（m），`frame_id=map`，≈10 Hz |
| `/flyrat/velocity` | `geometry_msgs/Vector3Stamped`（发布） | 本地 ENU 速度（m/s），同上 |
| `/flyrat/battery` | `std_msgs/Float32` | 电压（V） |
| `/flyrat/set_arm` | `std_srvs/SetBool`（服务） | 拨动 ARM 开关（AUX1） |
| `/flyrat/set_msp_override` | `std_srvs/SetBool`（服务） | 拨动 MSP OVERRIDE 开关（AUX2） |
| `/flyrat/camera/image_raw` | `sensor_msgs/Image`（发布） | 机身前视相机，`mono8`，1280×800，≈30 Hz（`flyrat_sitl.launch.py` 默认桥接，或 `.script/camera.sh`） |
| `/flyrat/camera/camera_info` | `sensor_msgs/CameraInfo`（发布） | 相机内参（同上） |

常用参数：`flyrat_msp` 的 `host` / `msp_port`(5761) / `rc_port`(9004) /
`rate`(50) / `telemetry_rate`(10)；`flyrat_control` 的 `control_rate`(50) /
`state_timeout`(2.0) / `takeoff_throttle`(0.7) / `flight_seconds`(5)。

### 7.4 可视化（Foxglove）

`ros-jazzy-foxglove-bridge` 已随镜像安装。另开一个终端启动桥接：

```bash
.script/foxglove.sh                 # ws://<主机>:8765
.script/foxglove.sh --port 9000     # 换端口
```

然后在**主机**上打开 Foxglove Studio，`Open connection` → `WebSocket` →
`ws://<主机地址>:8765`。桥接默认监听 `0.0.0.0`，容器使用 host 网络，
通常 `ws://127.0.0.1:8765` 也能直接连。

可以订阅上面接口表里的全部 `/flyrat/*` 话题。这些话题使用系统时间戳，
不需要打开 `use_sim_time`。

### 7.5 相机（Gazebo → ROS 2）

`flyrat_play/gazebo/models/betaflight_iris/model.sdf` 在机身 `base_link` 上挂了一个
前视相机（灰度 `mono8`、1280×800、30 Hz、水平视场 60°）。Gazebo 的 sensors 系统发布在

```
/world/betaflight_sitl/model/iris/link/base_link/sensor/camera/image
/world/betaflight_sitl/model/iris/link/base_link/sensor/camera/camera_info
```

仿真启动后，桥接随 ROS 2 的 launch 一起起来（默认开启）：

```bash
ros2 launch flyrat_control flyrat_sitl.launch.py                # 默认带相机
ros2 launch flyrat_control flyrat_sitl.launch.py camera:=false  # 不要相机
ros2 launch flyrat_control flyrat_sitl.launch.py camera_prefix:=/cam
```

如果只想看图像、不启动控制节点，也可以用独立脚本：

```bash
.script/camera.sh                        # -> /flyrat/camera/{image_raw,camera_info}
.script/camera.sh --prefix /cam          # 换命名空间
```

注意两者**不要同时开**：会出现两个 `/flyrat/camera` 发布者和两个 `/clock` 发布者。

桥接由 `ros_gz_bridge` 完成（`ros-jazzy-ros-gz-bridge`，已随镜像安装），除了图像和
`camera_info`，还会把 Gazebo 的 `/clock` 桥接成 `rosgraph_msgs/msg/Clock`（400 Hz，
与物理步长一致），需要时查看端可以打开 `use_sim_time`。图像是
`sensor_msgs/msg/Image`（`mono8`），`header.frame_id` 为 `iris/base_link/camera`。

```bash
ros2 topic hz /flyrat/camera/image_raw   # ≈30 Hz
ros2 run rqt_image_view rqt_image_view   # 直接看图
```

也可以直接用 7.4 的 Foxglove 桥接查看，`/flyrat/camera/*` 会一并转发。

脚本同时把 Gazebo 的 `/clock` 桥到 ROS 2（`rosgraph_msgs/msg/Clock`，400 Hz），
所以图像的 `header.stamp` 是**仿真时间**而不是系统时间。查看端如果按时间戳
渲染，请让它使用仿真时钟：

```bash
ros2 param set /<查看节点名> use_sim_time true
```

**画面一闪一闪？**

本仓库侧的链路已实测干净：无界面、带 GUI、以及飞行过程中分别抓了 300–900 帧，
都是 30 Hz、最大间隔 < 50 ms、没有黑帧（逐帧均值完全一致）。所以闪烁基本只会
来自下面两类原因：

1. **重复的仿真栈（最常见）**：两个 `gz sim` 会发布同名的 Gazebo 话题，一个
   bridge 在两份画面之间来回切换，看起来就是闪烁。确认只有一个栈：

   ```bash
   pgrep -af 'betaloop/start.py'                 # 应该只有 1 条
   env -u GZ_CONFIG_PATH PATH="/usr/bin:$PATH" \
     gz topic -i -t /world/betaflight_sitl/model/iris/link/base_link/sensor/camera/image
   # Publisher count 应为 1
   ```

   `launch_sim.sh` 现在会在检测到已有栈（或 5761 已被占用）时直接拒绝启动。
   注意：在 source 过 ROS 2 的终端里，`gz` 会解析到 `gz_*_vendor` 的精简版
   命令行（没有 `sim` 命令），所以上面用了 `env -u GZ_CONFIG_PATH PATH=...`。

2. **查看端带宽**：1280×800 `mono8` @ 30 Hz 约 **31 MB/s**，浏览器里的 Foxglove
   或 rviz2 解码这么大的未压缩流时，弱机器/远程连接会掉帧。可先停掉
   `.script/camera.sh`，用 `ros2 topic hz` 确认发布端仍是 30 Hz 来区分；
   确属带宽问题就把 `models/betaflight_iris/model.sdf` 里的 `<update_rate>`
   降到 15、分辨率降到 320×240，然后重启仿真。

### 7.6 定位：GPS → 本地 ENU（实机换 VSLAM）

四旋翼的几何 MPC 需要位置和速度，而 `MSP_ATTITUDE` 只给姿态。仿真里用飞控自己的
GPS，实机上换成 VSLAM，**两者走同一个接口**。

数据链：

```
Gazebo 世界球坐标（spherical_coordinates）
  → 插件把局部位置转成 lat/lon/alt 并发给 SITL（virtual GPS）
  → 飞控 GPS + 气压计 → EKF
  → MSP_RAW_GPS / MSP_ALTITUDE
  → flyrat::LocalEstimator（libflyrat，无 ROS）
  → /flyrat/position, /flyrat/velocity（≈10 Hz）
```

**坐标契约**：本地切平面 ENU，`x=东`、`y=北`、`z=上`，原点在**第一次有效定位**
处锁定。实机的 VSLAM 只要发布同样语义的 `/flyrat/position` + `/flyrat/velocity`，
`libflyrat` 与 `flyrat_control` 都不需要改。

各通道的来源和取舍：

| 量 | 来源 | 说明 |
|---|---|---|
| 水平位置 | `MSP_RAW_GPS` 经纬度 | 以原点为参考做等距圆柱投影（`R=6378137`），10 Hz |
| 水平速度 | `MSP_RAW_GPS` 地速 + 航向 | 悬停时航向噪声大，加了 **0.25 m/s 死区** |
| 垂直位置/速度 | `MSP_ALTITUDE` | 用 EKF 高度和升降率；**不用 GPS 高度**（字段分辨率 1 m） |

注意事项：

- **z 原点是相对的**。原点取第一次定位时的气压高度，与 Gazebo 地面 `z=0.195` 之间
  有 0.1–0.35 m 的常值偏差。控制器只需要相对高度，但**评估时**要用
  `gz model -m iris -p` 对比，别直接和 0 比。
- 解锁瞬间飞控会重置估计器，高度会短暂掉到 0 再收敛；`has_position` 仍然为真。
- 位置话题是 **10 Hz**，对 50 Hz 的姿态环够用，但**做 100 Hz MPC 时位置需要预测/
  插值**，或者改用更高频的里程计（实机的 VSLAM 通常能给 30–60 Hz）。

**踩过的坑：IMU 比力**。上游 Gazebo Harmonic 插件把 Gazebo 的世界系运动学加速度
（不含重力）直接塞进 `imuLinearAccelerationXYZ`，飞控因此认为静止时是 0 g，位置估计器
一直积分 `-9.81 m/s²`，`MSP_ALTITUDE` 永远是 0。修复在
`flyrat_play/gazebo/patches/aeroloop_gazebo-imu-specific-force.patch`：
在世界系加回 `+9.81 ẑ`，再用 IMU 姿态旋到机体系（FLU）。验证方法是把 `debug_mode`
设为 `ALTITUDE`，看 `DEBUG_ALTITUDE` 第 6 项（垂直加速度），静止时应为 0 而不是 -981。

## 8. 算法库与实机程序

### 8.1 `libflyrat/`

与 ROS 解耦的纯 C++17 库，同时包含算法与 Betaflight 传输：

- `flyrat::State`：姿态（rad）、角速度、高度、电压、是否解锁等估计量。
- `flyrat::Setpoint`：期望姿态 / 偏航角速度 / 油门。
- `flyrat::Command`：归一化指令 `roll/pitch/yaw ∈ [-1,1]`、`throttle ∈ [0,1]`。
- `flyrat::Autopilot`：`reset()` + `update(state, setpoint, dt) -> Command`。
- `flyrat::AttitudeHold`：示例实现（P 姿态控制 + 偏航角速度阻尼 + 输出限幅）。
- `flyrat::MspClient`：MSP v1 客户端，支持 TCP（SITL）与串口（实机）；
  `raw_gps()` / `altitude()` / `debug()` 分别读 GPS、EKF 高度和调试通道。
- `flyrat::LocalEstimator`：把 `MSP_RAW_GPS` + `MSP_ALTITUDE` 转成本地 ENU
  位置/速度（原点锁定、0.25 m/s 速度死区、无 GPS 时保留原点）。实机换 VSLAM 时
  直接替换这一层的输入。
- `flyrat::RcReceiver`：SITL 虚拟接收机（UDP 9004）。
- `flyrat::so3`：`exp/log/hat/leftJacobian`，头文件实现，SO(3) 上做误差与线性化。
- `flyrat::Ommpc`：on-manifold MPC（Lu/Xu/Zhang, IEEE TIE 2023）。误差状态
  `(δp, δv, Log(RᵀR_d))` 共 9 维，输入为归一化推力 + 机体角速度；沿参考轨迹逐点
  一阶线性化得到 LTV 模型，用 OSQP 解 80 变量 QP。默认 N=20 / 10 ms，单元测试里
  2200 次求解平均 0.86 ms、最大 1.17 ms。
- `flyrat::ThrustModel`：`油门 → 加速度` 的递归最小二乘在线辨识（遗忘因子
  0.998，用约 40 ms 前的指令对齐执行器延迟）。

`flyrat_control` 与 `flyrat_msp` 都通过 `add_subdirectory(.../libflyrat)` 直接链接
该库，因此算法可以脱离 ROS 单独编译和测试。

### 8.2 `flyrat_pilot/`

实机端程序，**不依赖 ROS**，链接同一份 `libflyrat`。目前是**占位骨架**：
只保留可编译的 `main()` 和 CMake 配置，运行时逻辑尚未实现。

```bash
.script/build_flyrat_pilot.sh
./flyrat_pilot/build/flyrat_pilot
```

预留的接入点：`flyrat::MspClient`（TCP / 串口）、`flyrat::RcReceiver`（UDP 9004）、
`flyrat::Autopilot`（算法）。建议采用“固定频率发送线程 + 独立遥测/算法线程”的结构。
详见 `flyrat_pilot/README.md`。

## 9. 端口速查

| 端口 | 方向 | 含义 |
|---|---|---|
| UDP 9002 | SITL → Gazebo | 电机指令 `float[4]`（0–1） |
| UDP 9003 | Gazebo → SITL | FDM 状态（陀螺 / 加速度 / 四元数 / 位置 / 气压） |
| UDP 9004 | 外部 → SITL | RC 帧 `{double 时间戳; uint16[16]}`（1000–2000） |
| TCP 5761 | 双向 | UART1，MSP / CLI（SITL 只允许**一个**客户端） |
| TCP 6761 | 双向 | websockify → 5761（网页配置器，默认禁用） |
| TCP 8765 | 双向 | Foxglove bridge（`.script/foxglove.sh`） |

## 10. 常见问题排查

**拉起后不起飞 / 油门没反应**

- 先用 `bringup_test.py --takeoff` 排除 ROS 2 层的因素。
- MSP OVERRIDE 需要 **fresh** 帧：`MSP_SET_RAW_RC` 必须持续推送（固件的新鲜度
  窗口是 300 ms；本项目按 50 Hz 推送 16 通道），一旦停发，飞控会退回接收机
  通道。所以"发送"必须和"读取遥测"分离到不同线程，读遥测不能把发送卡住。
- 解锁必须 **先低后高**：AUX1 若一开始就是高电平，飞控会因 `NOT_DISARMED`
  拒绝解锁。请先保持 AUX1 低，等链路稳定后再拉高。
- 油门低起飞前不要提前加油门，否则会触发 `RUNAWAY THROTTLE` 而拒绝解锁。

**仿真时间几乎不前进（飞机"悬不住"、MSP 突然全超时）**

- 症状：起飞瞬间 Gazebo 的实时率从 ~1.0 掉到 0.00x，8 s 的起飞窗口里几乎没有
  仿真时间流过，飞机只能挪十几厘米；同时 MSP 遥测成片超时、有时"下一次运行
  连不上"。用
  `gz topic -e -t /world/betaflight_sitl/stats | grep real_time_factor`
  可以直接看到实时率。
- 原因：上游桥接插件在**物理步进内**用 `select` 等电机指令包，且"在线"后把等待
  上限设为 **1000 ms**（`BetaflightPlugin.cc` 的 `Recv(..., waitMs=1000)`）。
  只要丢一个包，物理线程就阻塞在这一步，之后 SITL 与插件的收发互相等待，
  正反馈式雪崩。
- 修复：`flyrat_play/gazebo/patches/aeroloop_gazebo-motor-recv-timeout.patch`（`waitMs=2`），
  由 `.script/build_plugin.sh` 自动、幂等地应用。改动只影响"等多久"，
  不影响控制律。若手工改过子模块导致补丁打不上，先
  `git -C flyrat_play/gazebo/third_party/aeroloop_gazebo checkout -- .`。

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

**相机画面一闪一闪**

- 先按 7.5 的清单区分"重复仿真栈"和"查看端带宽"：数据链路本身实测是 30 Hz、
  无丢帧、无黑帧，闪烁来自查看侧。
- 最常见的原因是残留的旧栈：两个 `gz sim` 同名话题会让一个 bridge 在两张画面
  之间切换。`launch_sim.sh` 已加检测，启动前会直接拒绝并提示先停掉旧栈。
- 从 source 过 ROS 2 的终端里 `gz` 会指向 `gz_*_vendor` 的精简版命令行（没有
  `sim` 命令），需要时用
  `env -u GZ_CONFIG_PATH PATH="/usr/bin:$PATH" gz ...`。

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
