# 坐标系与变换说明

本文说明实机 ROS 2 链路中的坐标系：`sensor_receiver_node` 发布的 IMU/双目，
`odometry_node` 使用的 body 系，以及 `odometry_map`、`imu_link`、
`camera_left_optical_frame` 之间的关系。里程计接口和参数见
[`overview.md`](overview.md)。

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

相机系 `camera_left_optical_frame` 使用标准光学系：

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
odometry_node
  │  后端输出 T_world_camera
  │  T_world_body = T_world_camera * T_camera_body
  ▼
/flyrat/odometry/odometry      odometry_map -> imu_link
TF  odometry_map -> imu_link   （动态）
TF  imu_link -> camera_left_optical_frame（静态，来自 imu.T_body_camera）
TF  camera_left_optical_frame -> camera_right_optical_frame
                                           （静态，来自 camera.T_left_right）
```

里程计后端约定 body 系就是 IMU 系，使用 FLU，重力方向 `(0, 0, -9.81)`。
静止时加速度计应输出 `z ≈ +9.81 m/s²`。

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

零偏 `b_acc` / `b_gyr` 没有写入配置（Kalibr 把它建模成时变样条），应由里程计
后端在线估计。`T_i_b` 为单位阵，即 IMU 芯片系与 `imu_link` 同向。

同一文件还设置 `imu_time_offset`：Kalibr 估计 `t_imu = t_cam + 0.003805 s`
（cam0）。接收节点从 IMU 时间戳中减去这个值，让 IMU 与相机对齐。改标定结果时
同时更新该文件。

## 4. `imu.T_body_camera` 的方向

配置中的 `imu.T_body_camera` 是**相机 → body**：

```text
p_body = T_body_camera * p_camera
```

不是 body → camera。它同时就是相机在 body 中的位姿 `T_body_camera`，所以
`odometry_node` 在启动时读出后：

- 静态 TF `imu_link -> camera_left_optical_frame` 直接使用
  `T_body_camera`（TF 存的是 child 在 parent 中的位姿）；
- 静态 TF `camera_left_optical_frame -> camera_right_optical_frame` 直接使用
  `camera.T_left_right`（它把右目坐标变到左目：`p_left = T * p_right`）；
- 求逆得到 `T_camera_body`，用于
  `T_world_body = T_world_camera * T_camera_body`。

如果手上拿到的是 body → camera（例如某些 CAD 或标定工具的输出），写入
YAML 前必须求逆。

## 5. 当前标定外参

`libflyrat/configs/camera/uvc_640x480.yaml` 中的外参来自 `kalibr_ws/`
（Aprilgrid 双目 + IMU-相机联合标定），不是占位值：

```yaml
imu:
  T_body_camera: !!opencv-matrix
    rows: 4
    cols: 4
    dt: d
    data: [0.007113, -0.020471, 0.999765, 0.040312,
           0.999929, -0.009399, -0.007306, -0.044603,
           0.009547, 0.999746, 0.020403, 0.008920,
           0.0, 0.0, 0.0, 1.0]
```

相机仍然是倒置安装，光轴大致朝 body 前方，但相对 body 有小角度偏差：

| 相机轴 | 在 body 中的方向 |
|---|---|
| 相机 x（图像右） | 约 body +y（左） |
| 相机 y（图像下） | 约 body +z（上） |
| 相机 z（光轴） | 约 body +x（前） |

平移是相机光心在 body 中的位置：前 +0.0403 m、左 -0.0446 m（即 IMU
右侧 0.0446 m）、上 +0.0089 m。

`camera.T_left_right` 同样来自 `kalibr_ws/calib-camchain.yaml`，是 Kalibr
`cam1.T_cn_cnm1` 的逆（左目 ← 右目）：

```yaml
camera:
  T_left_right: !!opencv-matrix
    rows: 4
    cols: 4
    dt: d
    data: [0.999949, -0.000045, 0.010089, 0.064749,
           0.000046, 0.999999, -0.000065, 0.000017,
           -0.010089, 0.000065, 0.999949, -0.000311,
           0.0, 0.0, 0.0, 1.0]
```

基线 0.06475 m，右目相对左目绕 y 轴约 0.578°。

## 6. 如何调整

### 6.1 只改 IMU 符号

启动时覆盖参数，不改代码：

```sh
ros2 launch flyrat_sensors sensors.launch.py \
  imu_accel_axis:=none \
  imu_gyro_axis:=none
```

### 6.2 改相机外参

直接编辑 `libflyrat/configs/camera/uvc_640x480.yaml`。为避免每次
`colcon build` 后安装目录不同步，运行时把 `camera_config_path` 指向源码树
中的文件：

```sh
ros2 launch flyrat_odometry odometry.launch.py \
  camera_config_path:=/workspaces/flyrat/libflyrat/configs/camera/uvc_640x480.yaml
```

`bringup.launch.py` 默认优先使用源码树中的配置。`odometry_node` 启动时会检查
矩阵是否为有限值、旋转是否正交、行列式是否为 1；不满足会直接报错退出，不会
带着错误矩阵运行。

### 6.3 检查 TF 链

```sh
ros2 run tf2_ros tf2_echo odometry_map imu_link
ros2 run tf2_ros tf2_echo imu_link camera_left_optical_frame
ros2 run tf2_ros tf2_echo camera_left_optical_frame camera_right_optical_frame
```

第二条应输出 `T_body_camera`，即 `imu.T_body_camera` 本身（`tf2_echo` 打印的
是把 target 的数据变换到 source 的矩阵）。若结果是它的逆，说明静态 TF 方向
写反。第三条应输出 `camera.T_left_right`，标定值为平移约
`(+0.06475, +0.0000175, -0.00031)`、绕 y 轴约 0.578°。

## 7. 验证动作

先启动 `sensor_receiver_node`，不启动里程计，单独检查 IMU：

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

后端实现后，在 Foxglove 中：

1. 固定坐标系选 `odometry_map`；
2. 打开 `imu_link` 和 `camera_left_optical_frame` 的 TF；
3. 手持设备缓慢平移、左右偏航、抬头低头；
4. 观察 `imu_link` 的朝向是否与真实机体一致。

不要在固定坐标系为 `imu_link` 时判断朝向；那样 `imu_link` 永远显示为单位姿态。

## 8. 常见现象

- **左偏航时轨迹向右**：优先检查 `angular_velocity.z` 符号和
  `imu_gyro_axis`。
- **机体滚转在相机系里像偏航**：相机光轴与 body x 平行，光学系的 z 就是
  光轴；这是坐标分解，不是里程计算错。看 `imu_link`，不要看相机系。
- **`imu_link` 绝对姿态在启动时漂移**：后端尚未完成重力对齐，世界系还没有
  定义；等初始化完成后再判断。
- **相机外参改了但运行没变化**：`camera_config_path` 指向的是安装目录中的旧
  YAML；改指向源码树，或重新 `colcon build`。

## 9. 相关提交

| 提交 | 内容 | 对坐标的影响 |
|---|---|---|
| `22cc062` | 拆分加速度/陀螺仪轴向参数 | 引入 `bf_to_flu`，只翻陀螺仪 z |
| `4d0005c` | 把陀螺仪默认值改成 `none` | **回归**：静态检查只看了加速度 z，没有覆盖偏航符号，左偏航仍为负 |
| `0675026` | 发布机体位姿、对齐传感器时间戳 | `T_world_body = T_world_camera * T_camera_body`，数值链路正确 |
| `2e9e00b` | 占位外参改为倒置正视前方 | 相机 z 与 body x 平行，滚转在相机系里表现为绕光轴旋转 |

本轮用实机逐项复测后确认：本机陀螺仪原始数据已经是 FLU，默认应为
`imu_gyro_axis=none`。左偏航实测 `angular_velocity.z ≈ +0.43 rad/s`，
右偏航为负；此前默认 `bf_to_flu` 会把偏航方向翻反。如果后续再改轴向，
请按第 7 节的偏航、俯仰、滚转三个动作逐项验证，不要只用静止加速度判断。
