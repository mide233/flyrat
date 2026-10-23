# Betaflight SITL × Gazebo：仿真与自动驾驶联调

本文介绍如何在开发容器里把 **Betaflight SITL**、**Gazebo Harmonic** 和我们
自己的 **自动驾驶算法** 串成一个闭环，用于验证飞控算法。底层仿真工程的细节
见 [`flyrat_play/gazebo/README.md`](../../flyrat_play/gazebo/README.md)，本文侧重"从零到跑起来"的完整流程
以及踩过的坑。

## 1. 总体结构

```
┌──────────────── libflyrat/（纯 C++17，无 ROS 依赖）────────────────┐
│  算法：State / Setpoint / Command / AttitudeHold / Ommpc / SO(3)   │
│  模型：ThrustModel（油门→加速度）、RcMapping（角速度→摇杆）        │
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
- 反馈分两路：

  | 量 | 来源 | 频率 |
  |---|---|---|
  | 姿态、机体角速度 | 飞控 MSP 遥测（`MSP_ATTITUDE` / `MSP_RAW_IMU`），`msp_node` 转发 | 50 Hz |
  | 位置、速度 | Gazebo 里程计 → `/flyrat/position`、`/flyrat/velocity` | 60 Hz |
  | 高度 | 飞控 `MSP_ALTITUDE`（EKF 气压），备用/对比 | 50 Hz |

  控制节点以 100 Hz 运行，用最新一帧状态做预测。

  仿真里没有 VSLAM，所以位置默认取 Gazebo 真值；**接口与实机一致**，实机上把
  这一路换成 VSLAM 的话题即可，见 7.6。`use_ground_truth:=false` 可退回飞控的
  10 Hz GPS 通路（精度与延迟都更差，仅用于自检）。
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

> 桥接插件需要的两处修复已经作为提交放在 `mide233/aeroloop_gazebo` 的 `gz` 分支上，
> 由子模块指针固定：
>
> - 物理步进内等待电机指令包的上限从 1000 ms 降到 **2 ms**，否则丢一个包就会让
>   整个仿真卡死（详见第 10 节）；
> - IMU 输出**比力**（世界系加回重力后旋到机体系），否则飞控的高度估计不可用。
>
> 所以 `.script/build_plugin.sh` **不再打补丁**：干净的 checkout 直接
> `git submodule update --init --recursive` 就能构建出修好的插件，脚本会检查
> `waitMs = 2` 以防子模块没更新。

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
| `rates_type = BETAFLIGHT` + `*_rc_rate = 200` + `*_expo = 0` + `*_srate = 0` | 线性速率曲线 `400 × 摇杆`（deg/s），控制器可以一个比例系数把角速度指令换成摇杆量 |
| `yaw_motors_reversed = ON` | 本机型 1/4 号电机与默认混控的转向相反，不反转 yaw 混控会让偏航正反馈直到翻滚 |
| `debug_mode = ALTITUDE` | `MSP_DEBUG` 暴露高度估计的垂直通道，排查高度/比力问题用 |

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

用 on-manifold MPC 飞 8 字轨迹（目前验证过的完整配置）：

```bash
ros2 launch flyrat_control flyrat_sitl.launch.py \
  controller:=ommpc auto_takeoff:=true \
  flight_seconds:=120 msp_override:=false
```

关于 `msp_override:=false`：飞控在 **MSP OVERRIDE 处于接管状态时会拒绝解锁**
（`ARMING_DISABLED_MSP`），本项目的 `msp_node` 已经做了门控——AUX2 在飞控报告
解锁之前保持低电平，解锁后才拉高。若仍遇到解锁被拒，用 `msp_override:=false`
最稳：此时 AETR 不经 MSP 覆盖，而是直接由 9004 接收机通道下发，`msp_node`
往里推的就是同一份控制量，闭环行为完全一致。

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
| `/flyrat/position` | `geometry_msgs/Vector3Stamped`（发布） | 本地 ENU 位置（m），`frame_id=map`；默认 **60 Hz**，来自 Gazebo 里程计；`use_ground_truth:=false` 时为 10 Hz 的 GPS 通路 |
| `/flyrat/velocity` | `geometry_msgs/Vector3Stamped`（发布） | 本地 ENU 速度（m/s），同上 |
| `/flyrat/ground_truth/odometry` | `nav_msgs/Odometry`（内部） | `ros_gz_bridge` 从 `/model/iris/odometry` 转来的 Gazebo 真值，`msp_node` 再转成上面两个话题 |
| `/flyrat/target` | `geometry_msgs/Vector3Stamped`（订阅） | 相对悬停点的位置偏置（m），给任务层用；轨迹模式的参考由控制节点自己生成 |
| `/flyrat/battery` | `std_msgs/Float32` | 电压（V） |
| `/flyrat/set_arm` | `std_srvs/SetBool`（服务） | 拨动 ARM 开关（AUX1） |
| `/flyrat/set_msp_override` | `std_srvs/SetBool`（服务） | 拨动 MSP OVERRIDE 开关（AUX2） |
| `/flyrat/camera/image_raw` | `sensor_msgs/Image`（发布） | 机身前视相机，`mono8`，1280×800，≈30 Hz（`flyrat_sitl.launch.py` 默认桥接，或 `.script/camera.sh`） |
| `/flyrat/camera/camera_info` | `sensor_msgs/CameraInfo`（发布） | 相机内参（同上） |

常用参数：

- `flyrat_msp`：`host` / `msp_port`(5761) / `rc_port`(9004) / `rate`(50，RC 发送) /
  `telemetry_rate`(50) / `rc_timeout`(0.25) / `msp_override`(true) /
  `use_ground_truth`（节点默认 false，launch 默认 true）。
- `flyrat_control`：`control_rate`(100) / `controller`(`attitude_hold` 或 `ommpc`) /
  `auto_takeoff`(false) / `state_timeout`(2.0) / `takeoff_throttle`(0.7) /
  `takeoff_height`(1.0) / `takeoff_seconds`(2.0) / `flight_seconds`(60) /
  `disarm_after_flight`(true)，以及 7.7 节列出的全部 `mpc_*` 与 `trajectory*`。

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

### 7.6 定位：Gazebo 真值 / GPS → 本地 ENU（实机换 VSLAM）

MPC 需要位置和速度，而 `MSP_ATTITUDE` 只给姿态。位置有两条通路，**接口相同**
（`/flyrat/position` + `/flyrat/velocity`，ENU，`frame_id=map`）：

**默认：Gazebo 里程计真值（60 Hz）**

```
model.sdf 的 OdometryPublisher（/model/iris/odometry）
  → ros_gz_bridge → /flyrat/ground_truth/odometry（nav_msgs/Odometry）
  → msp_node 的 60 Hz 定时器：位置直接取，机体系速度用姿态旋到世界系
  → /flyrat/position, /flyrat/velocity（60 Hz）
```

这是仿真里最"实机化"的替身：实机的 VSLAM 只要发布同样语义的 60 Hz 位置/速度，
`libflyrat` 和 `flyrat_control` 一行都不用改。真值只用于**位置/速度**，姿态仍来自
飞控，所以姿态环、角速度环、电机混控都还是真的在跑。

**备用：飞控 GPS 通路（10 Hz，`use_ground_truth:=false`）**

```
Gazebo 世界球坐标（spherical_coordinates）
  → 插件把局部位置转成 lat/lon/alt 并发给 SITL（virtual GPS）
  → 飞控 GPS + 气压计 → EKF
  → MSP_RAW_GPS / MSP_ALTITUDE
  → flyrat::LocalEstimator（libflyrat，无 ROS）
  → /flyrat/position, /flyrat/velocity（≈10 Hz）
```

这条链路用来验证"没有外部定位时"的退化行为，精度和延迟都不如真值，**不要**用它
评估 MPC 性能。

**坐标契约**：本地切平面 ENU，`x=东`、`y=北`、`z=上`，原点在**第一次有效定位**
处锁定。Gazebo 真值通路的原点就是世界原点（起飞点在地面上方 0.195 m）。

各通道的来源和取舍（GPS 通路）：

| 量 | 来源 | 说明 |
|---|---|---|
| 水平位置 | `MSP_RAW_GPS` 经纬度 | 以原点为参考做等距圆柱投影（`R=6378137`），10 Hz |
| 水平速度 | `MSP_RAW_GPS` 地速 + 航向 | 悬停时航向噪声大，加了 **0.25 m/s 死区** |
| 垂直位置/速度 | `MSP_ALTITUDE` | 用 EKF 高度和升降率；**不用 GPS 高度**（字段分辨率 1 m） |

注意事项：

- **气压高度的原点是相对的**。GPS 通路的原点取第一次定位时的气压高度，与 Gazebo
  地面之间通常有 **0.1–0.35 m 的常值偏差**（实测约 0.16 m）。所以日志里
  `err_z ≈ -0.16` 而飞机实际悬停高度是对的；评估位置误差请以 Gazebo 位姿或
  60 Hz 真值为准。
- 解锁瞬间飞控会重置估计器，GPS 通路的高度会短暂掉到 0 再收敛；`has_position`
  仍然为真。
- 60 Hz 真值通路的延迟约一个物理步长（2.5 ms）加桥接开销，100 Hz MPC 直接用，
  不需要插值；10 Hz 的 GPS 通路则必须做预测，否则相位滞后会把闭环推向振荡。

**踩过的坑：IMU 比力**。上游 Gazebo Harmonic 插件把 Gazebo 的世界系运动学加速度
（不含重力）直接塞进 `imuLinearAccelerationXYZ`，飞控因此认为静止时是 0 g，位置估计器
一直积分 `-9.81 m/s²`，`MSP_ALTITUDE` 永远是 0。修复已提交在
`mide233/aeroloop_gazebo`（`gz` 分支，`06f5236`）：
在世界系加回 `+9.81 ẑ`，再用 IMU 姿态旋到机体系（FLU）。验证方法是把 `debug_mode`
设为 `ALTITUDE`，看 `DEBUG_ALTITUDE` 第 6 项（垂直加速度），静止时应为 0 而不是 -981。

### 7.7 on-manifold MPC（`controller:=ommpc`）

`flyrat_control` 内置两种控制器：

- `controller:=attitude_hold`（默认）：姿态 P 控制 + 偏航阻尼，不带位置环。
- `controller:=ommpc`：把飞控当成"角速度 + 推力"的执行器，外层跑 OMMPC。

控制链（100 Hz）：

```
/flyrat/position, /flyrat/velocity (60 Hz) ┐
/flyrat/attitude, /flyrat/imu (50 Hz)      ├─► Ommpc::solve → (a_T, ω)
参考轨迹（定点 / circle / figure8）        ┘
   a_T → throttle = a_T / mpc_thrust_to_accel
   ω   → 摇杆（400 deg/s 线性曲线）+ 限幅（mpc_max_body_rate_*）+ 限速（mpc_max_rate_slew）
   → /flyrat/rc_override → msp_node → UDP 9004 / MSP_SET_RAW_RC
```

姿态参考由 MPC 的期望加速度反解（`attitude_from_acceleration`），偏航角保持
起飞时的朝向（`mpc_yaw_offset` 可加偏置）。

**轨迹**：`trajectory: none | circle | figure8`，圆心/8 字原点取进入 hold 相位时的
位置，半径 `trajectory_radius`（默认 1.5 m），周期 `trajectory_period`（圆 20 s，
8 字 10 s）。8 字是 Gerono 双纽线 `(a sin θ, b sin 2θ)`，其中 `a` 为半径、`b = a/2`；
圆则把圆心沿 x 平移一个半径，使 t=0 时参考点正好在起飞点上，不会出现参考位置跳变。

**调参**：全部在 `flyrat_control/config/mpc.yaml`，改完重启节点生效。

| 参数 | 含义 | 经验 |
|---|---|---|
| `mpc_horizon` / `mpc_step` | 前瞻步数 × 步长 | 40 × 10 ms，覆盖速率环延迟 |
| `mpc_q_position_xy` | 水平位置权重 | 8 字轨迹用 60；25 时误差 0.3–0.6 m |
| `mpc_q_position_z` | 高度权重 | 120 |
| `mpc_q_velocity` | 速度权重 | 2.0 |
| `mpc_q_attitude_*` | 姿态跟踪权重 | 40，太高会和内环打架 |
| `mpc_r_body_rate_xy` | 角速度代价 | 1.2 |
| `mpc_max_body_rate_xy` | 角速度限幅 | 1.5 rad/s；1.0 时 8 字跟不上 |
| `mpc_max_rate_slew` | 角速度变化率上限 | 8 rad/s²，抑制指令跳变 |
| `mpc_thrust_to_accel` | 油门→加速度标定 | 14.2（本机型实测） |

**实测性能**（Gazebo，100 Hz 求解）：

| 场景 | 结果 |
|---|---|
| 定高悬停 | 1.35 m，`att_err=0`，`thr=0.58` |
| 1 m 位置阶跃 | 上升 1.18 s，超调 13.2 %，稳定 4.04 s，稳态误差 0.000 m |
| 1.5 m 圆 / 20 s | 0.46 m/s，跟踪误差 ~2 cm |
| 1.5 m 8 字 / 10 s | ~1.0 m/s，跟踪误差 4–15 cm，`att_err` 1–2° |

稳态求解 4.7–5.4 ms（`it=25`），在 100 Hz 下大约占 50 % 的单核实时预算；
再提高控制频率或加大 horizon 之前，先确认这台机器的求解时间。

每个控制周期会打一行（`RCLCPP_INFO_THROTTLE`，10 Hz）：

```
mpc: cmd=(...) att_err=.. vel=(.. .. ..) att_thr=.. thr2acc=.. thr=.. err=(.. .. ..) solve=..ms it=..
```

`err` 是参考位置减当前位置，`att_err` 是当前姿态与期望姿态的 SO(3) 测地角，
排振荡时先看这两个量和 `solve`。

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
  实现分两层：`msp_client.cpp` 只做连接与收发，`msp_telemetry.cpp` 放各条
  遥测的解析，共享的线格式辅助在内网头 `msp_codec.hpp` 里。
- `flyrat::LocalEstimator`：把 `MSP_RAW_GPS` + `MSP_ALTITUDE` 转成本地 ENU
  位置/速度（原点锁定、0.25 m/s 速度死区、无 GPS 时保留原点）。实机换 VSLAM 时
  直接替换这一层的输入。
- `flyrat::RcReceiver`：SITL 虚拟接收机（UDP 9004）。
- `flyrat::so3`：`exp/log/hat/leftJacobian`，头文件实现，SO(3) 上做误差与线性化。
- `flyrat::Ommpc`：on-manifold MPC（Lu/Xu/Zhang, IEEE TIE 2023）。误差状态
  `(δp, δv, Log(RᵀR_d))` 共 9 维，输入为归一化推力 + 机体角速度；沿参考轨迹逐点
  一阶线性化得到 LTV 模型，用 OSQP 解 QP。库内默认 N=20 / 10 ms；仿真里用
  `mpc_horizon=40`（0.4 s 前瞻，因为 Betaflight 速率环本身有 ~40 ms 延迟）。
  单元测试在 2200 次求解里平均 0.86 ms、最大 1.17 ms。
- `flyrat::RcMapping`：把机体角速度（rad/s）换成摇杆量。仿真侧把速率曲线配置成
  线性的 400 deg/s（见第 4 节），于是只剩一个比例系数。
- `flyrat::ThrustModel`：`油门 → 加速度` 的递归最小二乘在线辨识（遗忘因子
  0.998，用约 40 ms 前的指令对齐执行器延迟）。仿真里默认用实测标定的固定系数
  `mpc_thrust_to_accel=14.2`，`mpc_adapt_thrust:=true` 才启用在线辨识。

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
- 修复：`Recv(..., waitMs=2)`，已作为 commit `06f5236` 放在
  `mide233/aeroloop_gazebo` 的 `gz` 分支，由子模块指针固定，
  `git submodule update --init --recursive` 即可获得。改动只影响"等多久"，
  不影响控制律。

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

**`arming blocked: flags=0x...` 每 2 秒刷一次**

- 这是 `msp_node` 在"未解锁"时打的诊断（位图是 Betaflight 的 arming disable
  flags），启动阶段刷几秒、随后解锁就消失，属正常。
- 若一直不解锁，按位图排查最常见的三类：
  - **MSP**：MSP OVERRIDE 处于接管状态时飞控拒绝解锁（`ARMING_DISABLED_MSP`）。
    本项目已把 AUX2 门控成"解锁后才拉高"，仍被拒时用 `msp_override:=false`
    最稳，此时 AETR 直接由 9004 接收机通道下发，闭环等价。
  - **ARM 开关**：AUX1 必须"先低后高"。`auto_takeoff:=true` 会等链路稳定后拉高；
    手动飞则调用 `/flyrat/set_arm`。
  - **THROTTLE**：解锁瞬间油门必须在最低，别提前给油（会触发
    `RUNAWAY THROTTLE`）。
- 无头批量起多个实例时特别容易命中：SITL 还没就绪就开始拉 ARM，之后不会重试。
  目前的行为是时序相关的，批量脚本需要给每个实例留足启动时间。

**一起飞就自旋 / 翻滚**

- 检查 `config/betaflight_sim.txt` 里的 `yaw_motors_reversed = ON`：本机型 1/4 号
  电机的转向与 Betaflight 默认混控相反，不反转 yaw 混控会让偏航环变成正反馈，
  直到整机翻滚。
- 同时确认速率曲线是线性的（`rates_type = BETAFLIGHT`、
  `*_rc_rate = 200`、`*_expo = 0`、`*_srate = 0`）。控制器的"角速度 → 摇杆"
  换算依赖这条 400 deg/s 的直线，改成曲线会让增益随杆量变化。

**8 字 / 圆轨迹越飞越偏**

- 先看 MPC 日志里的 `err` 和 `att_err`：
  - `att_err` 长期大于 5°：角速度指令超出速率环带宽，降低
    `mpc_max_body_rate_xy`（1.5 → 1.0）或 `mpc_q_position_xy`（60 → 25）；
  - `err` 有明显相位滞后而 `att_err` 很小：位置权重或前瞻不够，适当加大
    `mpc_q_position_xy` / `mpc_horizon`。
- 确认位置是 60 Hz 真值：`ros2 topic hz /flyrat/position` 应约 60 Hz；只有
  10 Hz 说明 `use_ground_truth:=false`，闭环会因滞后而振荡。

**`aux` 配置像是没保存**

- 用**不带参数**的 `aux` 查看全部映射；`aux <index>` 这类参数不足的写法会
  **重置**该条映射（`cliAux` 在参数个数不为 4/5/6 时 `memset` 清空）。

**清理进程**

- 结束仿真：找到 `betaloop/start.py` 的进程组并 `kill -TERM -<pgid>`。
- **不要**用 `pkill -f betaloop/start.py` 之类的写法：它的 SIGTERM 处理器会杀掉
  整个进程组，可能连带把当前 shell 一起干掉。同理，任何 `pkill`/`grep` 的关键字
  都不要和命令行的其它文本重复（包括 `echo` 里的字面量），否则会误杀自己。
