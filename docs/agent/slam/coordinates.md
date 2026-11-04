# 坐标系与变换说明

本文只讨论实机 ROS 2 链路中的坐标系：`sensor_receiver_node` 发布的 IMU/双目，
`flyrat_slam` 内部使用的 ORB-SLAM3 body 系，以及 `slam_map`、`imu_link`、
`camera_left_optical_frame` 之间的关系。标定流程和精度评估见
[`orb_slam3.md`](orb_slam3.md)。

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
flyrat_slam / slam_node
  │  ORB-SLAM3 输出 T_world_camera
  │  T_world_body = T_world_camera * T_camera_body
  ▼
/flyrat/slam/odometry          slam_map -> imu_link
TF  slam_map -> imu_link       （动态）
TF  imu_link -> camera_left_optical_frame（静态）
```

ORB-SLAM3 的 body 系就是 IMU 系，要求 FLU、重力方向 `(0, 0, -9.81)`。
静止时加速度计应输出 `z ≈ +9.81 m/s²`。

## 3. IMU 轴向映射

`MSP_RAW_IMU` 同时包含加速度和陀螺仪，但两者在本机上的约定不同：

| 数据 | 原始约定 | 映射 | 参数 |
|---|---|---|---|
| 加速度 | 已是 FLU | `(x, y, z)` | `imu_accel_axis=none` |
| 陀螺仪 | x/y 为 FLU，偏航为顺时针正 | `(x, y, -z)` | `imu_gyro_axis=bf_to_flu` |

`imu_gyro_axis` 的可选值：

| 值 | 变换 | 适用情况 |
|---|---|---|
| `none` | `(x, y, z)` | 原始数据已经是完整 FLU |
| `bf_to_flu` | `(x, y, -z)` | Betaflight 陀螺仪，偏航符号相反 |
| `frd_to_flu` | `(x, -y, -z)` | 原始数据是 FRD |

`imu_accel_axis` 的可选值：

| 值 | 变换 | 适用情况 |
|---|---|---|
| `none` | `(x, y, z)` | 原始数据已经是 FLU（当前实机） |
| `frd_to_flu` | `(x, -y, -z)` | 原始数据是 FRD |

当前实机的判定依据：静止时 `linear_acceleration.z ≈ +9.83`，绕竖直轴向左
（俯视逆时针）转时原始 `angular_velocity.z` 为负。因此陀螺仪只翻 z。

## 4. `IMU.T_b_c1` 的方向

ORB-SLAM3 配置中的 `IMU.T_b_c1` 是**相机 → body**：

```text
p_body = T_b_c1 * p_camera
```

不是 body → camera。`slam_node` 在启动时读取它，并求逆得到
`T_camera_body`，用于：

- `T_world_body = T_world_camera * T_camera_body`；
- 静态 TF `imu_link -> camera_left_optical_frame`。

如果手上拿到的是 body → camera（例如某些 CAD 或标定工具的输出），写入
YAML 前必须求逆。

## 5. 当前占位外参

`libflyrat/configs/orb_slam3/uvc_640x480_imu.yaml`：

```yaml
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.0, 0.0, 1.0, 0.0,
         1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0]
```

它表示“相机倒置、光轴正对 body 前方、平移为 0”：

| 相机轴 | 在 body 中的方向 |
|---|---|
| 相机 x（图像右） | body +y（左） |
| 相机 y（图像下） | body +z（上） |
| 相机 z（光轴） | body +x（前） |

矩阵的列就是上表。平移为 0 只是占位，真实相机和 IMU 不共点；需要实测后填入
`data` 的第 4 列。

如果实际安装是“倒置、光轴朝 body 前方左侧 45°”，把绕 body z 的 45° 旋转
乘到当前矩阵左侧（body 系中的旋转）：

```text
T_b_c1(45°) = R_z(+45°) * T_b_c1(0°)
```

展开后的矩阵是：

```yaml
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [-0.707107, 0.0, 0.707107, 0.0,
          0.707107, 0.0, 0.707107, 0.0,
          0.0,      1.0, 0.0,      0.0,
          0.0,      0.0, 0.0,      1.0]
```

注意旋转方向要和“左侧 45°”一致；改完后用第 7 节的动作验证。

## 6. 如何调整

### 6.1 只改 IMU 符号

启动时覆盖参数，不改代码：

```sh
ros2 launch flyrat_sensors sensors.launch.py \
  imu_accel_axis:=none \
  imu_gyro_axis:=bf_to_flu
```

### 6.2 改相机外参

直接编辑 YAML 的 `IMU.T_b_c1` 矩阵。为避免每次 `colcon build` 后安装目录
不同步，运行时把 `settings_path` 指向源码树中的文件：

```sh
ros2 launch flyrat_slam slam.launch.py \
  vocabulary_path:=/workspaces/flyrat/playground/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  settings_path:=/workspaces/flyrat/libflyrat/configs/orb_slam3/uvc_640x480_imu.yaml
```

`slam_node` 启动时会检查矩阵是否为有限值、旋转是否正交、行列式是否为 1；
不满足会直接报错退出，不会带着错误矩阵运行。

### 6.3 检查 TF 链

```sh
ros2 run tf2_ros tf2_echo slam_map imu_link
ros2 run tf2_ros tf2_echo imu_link camera_left_optical_frame
```

第二条应输出 `T_camera_body`，即 `IMU.T_b_c1` 的逆。若方向相反，说明矩阵写反。

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

如果偏航符号相反，`imu_gyro_axis` 用 `bf_to_flu`；如果 y 也相反，用
`frd_to_flu`；如果全部相反，用 `none` 并检查加速度计是否也需要翻转。

然后启动 SLAM，在 Foxglove 中：

1. 固定坐标系选 `slam_map`；
2. 打开 `imu_link` 和 `camera_left_optical_frame` 的 TF；
3. 手持设备缓慢平移、左右偏航、抬头低头；
4. 观察 `imu_link` 的朝向是否与真实机体一致。

不要在固定坐标系为 `imu_link` 时判断朝向；那样 `imu_link` 永远显示为单位姿态。
静止桌面场景通常无法完成 IMU 初始化，日志出现
`Not enough motion for initializing` 是预期现象，需要平移或明显晃动。

## 8. 常见现象

- **左偏航时轨迹向右**：优先检查 `angular_velocity.z` 符号和
  `imu_gyro_axis`。
- **机体滚转在相机系里像偏航**：相机光轴与 body x 平行，光学系的 z 就是
  光轴；这是坐标分解，不是 SLAM 算错。看 `imu_link`，不要看相机系。
- **`imu_link` 绝对姿态在启动时漂移**：IMU 尚未完成重力对齐，世界系还没有
  定义；等初始化完成后再判断。
- **相机外参改了但运行没变化**：`settings_path` 指向的是安装目录中的旧
  YAML；改指向源码树，或重新 `colcon build`。

## 9. 相关提交

| 提交 | 内容 | 对坐标的影响 |
|---|---|---|
| `22cc062` | 拆分加速度/陀螺仪轴向参数 | 引入 `bf_to_flu`，只翻陀螺仪 z |
| `4d0005c` | 把陀螺仪默认值改成 `none` | **回归**：静态检查只看了加速度 z，没有覆盖偏航符号，左偏航仍为负 |
| `0675026` | 发布机体位姿、对齐传感器时间戳 | `T_world_body = T_world_camera * T_camera_body`，数值链路正确 |
| `2e9e00b` | 占位外参改为倒置正视前方 | 相机 z 与 body x 平行，滚转在相机系里表现为绕光轴旋转 |

`4d0005c` 的回归已在本轮恢复为 `imu_gyro_axis=bf_to_flu`。如果后续再改轴向，
请按第 7 节的偏航、俯仰、滚转三个动作逐项验证，不要只用静止加速度判断。
