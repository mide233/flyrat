# 坐标系与变换说明

本文只讨论实机 ROS 2 链路中的坐标系：`sensor_receiver_node` 发布的 IMU/双目，
`flyrat_slam` 内部使用的 VINS-Fusion body 系，以及 `slam_map`、`imu_link`、
`camera_left_optical_frame` 之间的关系。标定流程和精度评估见
[`vins_fusion.md`](vins_fusion.md)。

## 1. 约定

统一使用右手系和“目标系 ← 源系”的矩阵记号：

- `T_a_b` 把**源系 b** 中的点变换到**目标系 a**，即 `p_a = T_a_b * p_b`。
- 平移部分是源系原点在目标系中的坐标。
- 旋转矩阵的**列**是源系三个轴在目标系中的方向。

ROS 机体系 `imu_link` 使用 FLU：

| 轴 | 方向 |
|---|---|
| x | 前 |
| y | 左 |
| z | 上 |

相机系 `camera_left_optical_frame` / `camera_right_optical_frame` 使用标准光学系：

| 轴 | 方向 |
|---|---|
| x | 图像右 |
| y | 图像下 |
| z | 光轴向前 |

## 2. 完整数据链

```text
MSP_RAW_IMU
  │  Betaflight 原始计数
  ▼
sensor_receiver_node
  │  尺度换算 + 轴向映射
  ▼
/flyrat/sensors/imu            frame_id = imu_link (FLU)
/flyrat/sensors/stereo/...     frame_id = camera_*_optical_frame
  │
  ▼
flyrat_slam / slam_node
  │  VINS-Fusion 直接输出 T_world_body
  ▼
/flyrat/slam/odometry          slam_map -> imu_link
TF  slam_map -> imu_link       （动态）
TF  imu_link -> camera_left_optical_frame（静态，来自 body_T_cam0）
TF  imu_link -> camera_right_optical_frame（静态，来自 body_T_cam1）
```

VINS-Fusion 的 body 系就是 IMU 系，要求 FLU。`slam_map` 的世界系由 VINS
初始化定义：重力对齐，但初始 yaw 由第一帧图像/IMU 窗口决定，不是固定的
FLU 朝向。静止时加速度计应输出 `z ≈ +9.81 m/s²`。

## 3. IMU 轴向映射

本机实测 `MSP_RAW_IMU` 的加速度和陀螺仪都已经是 ROS FLU，不需要翻转：

| 数据 | 原始约定 | 映射 | 参数 |
|---|---|---|---|
| 加速度 | 已是 FLU | `(x, y, z)` | `imu_accel_axis=none` |
| 陀螺仪 | 已是 FLU，左偏航为正 | `(x, y, z)` | `imu_gyro_axis=none` |

实测：静止水平 `linear_acceleration.z ≈ +9.81`；机头向左偏航时
`angular_velocity.z ≈ +0.43 rad/s`；机头向右偏航时为负。

`imu_gyro_axis` 的可选值：

| 值 | 变换 | 适用情况 |
|---|---|---|
| `none` | `(x, y, z)` | 原始数据已经是完整 FLU（当前实机） |
| `bf_to_flu` | `(x, y, -z)` | 某些飞控的偏航为顺时针正，只翻 z |
| `frd_to_flu` | `(x, -y, -z)` | 原始数据是 FRD |

`imu_accel_axis` 的可选值：

| 值 | 变换 | 适用情况 |
|---|---|---|
| `none` | `(x, y, z)` | 原始数据已经是 FLU（当前实机） |
| `frd_to_flu` | `(x, -y, -z)` | 原始数据是 FRD |

当前实机的判定依据：静止时 `linear_acceleration.z ≈ +9.81`，绕竖直轴向左
（俯视逆时针）转时 `angular_velocity.z` 为正。因此两个数据流都不翻。

### 3.1 IMU 尺度、轴失准与时间偏移

`kalibr_ws/calib-imu.yaml` 的 scale-misalignment 模型由
`flyrat_play/ros2_ws/src/flyrat_sensors/config/imu_intrinsics.yaml` 加载，
`sensor_receiver_node` 在发布前补偿：

```text
a_body = inv(M_acc) * a_raw
a_gyro = C_gyro_i * a_body
w_body = C_gyro_i^T * inv(M_gyr) * (w_raw - A * a_gyro)
```

零偏 `b_acc` / `b_gyr` 没有写入配置（Kalibr 把它建模成时变样条），由
VINS-Fusion 在线估计。`T_i_b` 为单位阵，即 IMU 芯片系与 `imu_link` 同向。

同一文件还设置 `imu_time_offset`：Kalibr 估计 `t_imu = t_cam + 0.003805 s`
（cam0）。接收节点从 IMU 时间戳中减去这个值，让 IMU 与相机对齐；
`uvc_640x480_imu.yaml` 因此使用 `estimate_td: 0`、`td: 0.0`。改标定结果时
同时更新该文件。

## 4. `body_T_cam0` / `body_T_cam1` 的方向

VINS 配置中的 `body_T_cam0` / `body_T_cam1` 是**相机 → body**：

```text
p_body = body_T_cam * p_camera
```

不是 body → camera。它同时就是相机在 body 中的位姿，所以 `slam_node` 在启动
时读出后：

- 静态 TF `imu_link -> camera_left_optical_frame` 直接使用 `body_T_cam0`
  （TF 存的是 child 在 parent 中的位姿）；
- 静态 TF `imu_link -> camera_right_optical_frame` 直接使用 `body_T_cam1`；
- 不再需要相机位姿到机体位姿的复合：VINS 的 `getPoseInWorldFrame()` 已经
  返回 `T_world_body`。

如果手上拿到的是 body → camera（例如某些 CAD 或标定工具的输出），写入
YAML 前必须求逆。

## 5. 当前标定外参

`libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml` 中的外参来自
`kalibr_ws/`（Aprilgrid 双目 + IMU-相机联合标定），不是占位值：

```yaml
body_T_cam0: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [0.007113, -0.020471, 0.999765, 0.040312,
         0.999929, -0.009399, -0.007306, -0.044603,
         0.009547, 0.999746, 0.020403, 0.008920,
         0.0, 0.0, 0.0, 1.0]

body_T_cam1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [-0.002976, -0.020406, 0.999787, 0.040461,
         0.999952, -0.009445, 0.002783, 0.020143,
         0.009386, 0.999747, 0.020433, 0.009549,
         0.0, 0.0, 0.0, 1.0]
```

`body_T_cam1 = body_T_cam0 * Stereo.T_c1_c2`，其中 `Stereo.T_c1_c2` 把右目
坐标变到左目（`p_left = T * p_right`），来自 `kalibr_ws/calib-camchain.yaml`：

```yaml
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [0.999949, -0.000045, 0.010089, 0.064749,
         0.000046, 0.999999, -0.000065, 0.000017,
         -0.010089, 0.000065, 0.999949, -0.000311,
         0.0, 0.0, 0.0, 1.0]
```

相机仍然是倒置安装，光轴大致朝 body 前方，但相对 body 有小角度偏差：

| 相机轴 | 在 body 中的方向 |
|---|---|
| 相机 x（图像右） | 约 body +y（左） |
| 相机 y（图像下） | 约 body +z（上） |
| 相机 z（光轴） | 约 body +x（前） |

左目平移是相机光心在 body 中的位置：前 +0.0403 m、左 -0.0446 m（即 IMU
右侧 0.0446 m）、上 +0.0089 m。双目基线 0.06475 m，右目相对左目绕 y 轴约
0.578°。

VINS 的 OpenCV 矩阵解析按配置的 `dt` 读取；当前配置使用 `dt: d`（double）。

## 6. 如何调整

### 6.1 只改 IMU 符号

启动时覆盖参数，不改代码：

```sh
ros2 launch flyrat_sensors sensors.launch.py \
  imu_accel_axis:=none \
  imu_gyro_axis:=none
```

### 6.2 改相机外参

直接编辑 `libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml` 的
`body_T_cam0` 和 `body_T_cam1`。为避免每次 `colcon build` 后安装目录不同步，
运行时把 `vins_config` 指向源码树中的文件：

```sh
ros2 launch flyrat_slam slam.launch.py \
  vins_config:=/workspaces/flyrat/libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml
```

`slam_node` 启动时会检查矩阵是否为有限值、旋转是否正交、行列式是否为 1；
不满足会直接报错退出，不会带着错误矩阵运行。

### 6.3 检查 TF 链

```sh
ros2 run tf2_ros tf2_echo slam_map imu_link
ros2 run tf2_ros tf2_echo imu_link camera_left_optical_frame
ros2 run tf2_ros tf2_echo imu_link camera_right_optical_frame
ros2 run tf2_ros tf2_echo camera_left_optical_frame camera_right_optical_frame
```

第二条应输出 `body_T_cam0` 本身（`tf2_echo` 打印的是把 target 的数据变换到
source 的矩阵）。若结果是它的逆，说明静态 TF 方向写反。第三条应输出
`body_T_cam1`。第四条由 TF 树组合得到，平移约 `(+0.06475, +0.0000175,
-0.00031)`、绕 y 轴约 0.578°。

## 7. 验证动作

先启动 `sensor_receiver_node`，不启动 SLAM，单独检查 IMU：

```sh
ros2 topic echo /flyrat/sensors/imu --field linear_acceleration
ros2 topic echo /flyrat/sensors/imu --field angular_velocity
```

| 动作 | 期望 |
|---|---|
| 静止水平放置 | `linear_acceleration.z ≈ +9.8`，x/y 接近 0 |
| 绕竖直轴向左（俯视逆时针）偏航 | `angular_velocity.z > 0` |
| 抬头 | `angular_velocity.y < 0` |
| 右侧下沉（向右滚转） | `angular_velocity.x > 0` |

如果左偏航时 `angular_velocity.z` 为负，`imu_gyro_axis` 改用 `bf_to_flu`
（只翻 z）；如果 y 也相反，用 `frd_to_flu`（翻 y、z）。本机实测 `none`
正确，不需要翻转。

然后启动 SLAM，在 Foxglove 中：

1. 固定坐标系选 `slam_map`；
2. 打开 `imu_link` 和两个相机光学系的 TF；
3. 手持设备缓慢平移、左右偏航、抬头低头；
4. 观察 `imu_link` 的朝向是否与真实机体一致。

不要在固定坐标系为 `imu_link` 时判断朝向；那样 `imu_link` 永远显示为单位姿态。
静止桌面场景通常无法完成 VINS 初始化，日志停留在 `NOT_INITIALIZED` 是预期
现象，需要平移或明显晃动。

## 8. 常见现象

- **左偏航时轨迹向右**：优先检查 `angular_velocity.z` 符号和
  `imu_gyro_axis`。
- **机体滚转在相机系里像偏航**：相机光轴与 body x 平行，光学系的 z 就是
  光轴；这是坐标分解，不是 SLAM 算错。看 `imu_link`，不要看相机系。
- **`imu_link` 绝对姿态在启动时漂移**：IMU 尚未完成重力对齐，世界系还没有
  定义；等初始化完成后再判断。
- **相机外参改了但运行没变化**：`vins_config` 指向的是安装目录中的旧
  YAML；改指向源码树，或重新 `colcon build`。
- **一直 `NOT_INITIALIZED`**：VINS 需要足够的运动激励和纹理，静止桌面不够；
  同时确认 IMU 时间戳单调且覆盖图像时刻。

## 9. 迁移记录

SLAM 迁移到 VINS-Fusion 后，输出直接是 `T_world_body`；静态外参改为
`body_T_cam0` / `body_T_cam1`。IMU 轴向映射和接收节点的
scale/misalignment、时间偏移补偿保持不变。
