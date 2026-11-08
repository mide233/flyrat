# 里程计模块

`flyrat_odometry` 包包含两条路线：

- **默认：TIO-Former ToF-惯性后端** `tio_odometry_node`。它使用前/左/右/下
  4 个 8×8 ToF 和 200 Hz IMU，通过 libtorch 运行 TorchScript 模型。实机链路
  已进入 `TRACKING`，但模型只在仿真数据上训练，2026-11-08 旧布局下静态
  零漂约 0.98 m / 20 s，暂不能用于飞行闭环。详见
  [`tio_former.md`](tio_former.md) 和 [`handoff.md`](handoff.md)。
- **legacy：立体/立体惯性骨架** `odometry_node`。本文其余部分说明这条旧路线。
  **算法后端尚未实现**：节点会接收、同步和缓存数据并发布状态与 TF，但位姿
  始终无效，状态始终为 `NOT_INITIALIZED`。

保留旧骨架是为了固定传感器链路、时间同步、坐标系和 ROS 接口；当前实机已
移除双目，改为单个黑白单目相机，因此旧路线不再是默认路径。

## 1. 组成

| 位置 | 作用 |
|---|---|
| `libflyrat/include/flyrat/odometry/odometry.hpp` | ROS 无关的里程计接口 |
| `libflyrat/src/odometry/odometry.cpp` | 当前为桩实现，后端接入点 |
| `libflyrat/configs/camera/uvc_640x480.yaml` | 旧双目标定；当前单目不使用 |
| `libflyrat/tests/test_odometry_api.cpp` | 接口和桩行为测试 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/` | ROS 2 节点、参数、launch |
| `docs/agent/odometry/coordinates.md` | 坐标系、TF 链和标定矩阵说明 |
| `docs/agent/odometry/tio_former.md` | TIO-Former TorchScript 推理、C++/ROS 接入与限制 |
| `docs/agent/odometry/handoff.md` | 当前状态、代码地图、实机结果和开发优先级 |

## 2. C++ 接口

命名空间 `flyrat::odometry`：

```cpp
struct Options {
    Mode mode = Mode::kImuStereo;   // kStereo 或 kImuStereo
    std::string camera_config_path;
};

class Odometry {
public:
    explicit Odometry(Options options);
    Result track(const ImageView& left, const ImageView& right, double timestamp);
    Result track(const ImageView& left, const ImageView& right, double timestamp,
                 const std::vector<ImuSample>& imu);
    void reset();
    void shutdown();
    void saveTrajectory(const std::string& path) const;
};
```

- `ImageView` 是非持有型 8 位灰度图（width/height/stride/data），不依赖 OpenCV。
- `ImuSample` 包含时间戳、加速度和角速度，单位分别为 m/s²、rad/s。
- `Result::pose` 是 `T_world_camera`；`State` 为
  `kNotInitialized / kTracking / kRecentlyLost / kLost`。
- 构造时检查 `camera_config_path` 是否存在；`track()` 检查图像非空且左右尺寸
  一致；`shutdown()` 后继续 `track()` 会抛 `OdometryError`。
- 当前实现中 `track()` 返回 `kNotInitialized` 和无效位姿，
  `saveTrajectory()` 抛 `OdometryError`。

### 2.1 接入新算法

1. 在 `libflyrat/src/odometry/` 增加实现文件，或在 `odometry.cpp` 中替换
   `track()` 的函数体。
2. 从 `configs/camera/uvc_640x480.yaml` 读取标定。当前节点只解析两个 4x4
   外参；内参和 IMU 噪声留给后端。
3. 维护自己的状态机并填 `Result`。`Pose` 必须是左目相机在世界系中的位姿。
4. 需要输出轨迹时实现 `saveTrajectory()`（TUM 格式，每行
   `timestamp tx ty tz qx qy qz qw`）。
5. 重新构建 libflyrat 和 ROS 包；节点和话题不需要改动。

## 3. 配置

`libflyrat/configs/camera/uvc_640x480.yaml` 使用中性 schema：

```yaml
camera:
  width: 640
  height: 480
  fps: 60
  left:  {fx, fy, cx, cy, k1, k2, p1, p2}
  right: {fx, fy, cx, cy, k1, k2, p1, p2}
  T_left_right: ...   # p_left = T_left_right * p_right
imu:
  frequency, noise_gyro, noise_acc, gyro_walk, acc_walk
  T_body_camera: ...  # p_body = T_body_camera * p_camera
```

数值来自 DECXIN 双目 + Betaflight IMU 的 Kalibr 标定。改标定后可直接编辑
源码树文件，`bringup.launch.py` 默认优先使用源码树版本。

## 4. ROS 2 节点

`odometry_node` 的职责：

- 订阅左右目图像，使用 `message_filters::ApproximateTime` 同步。
- 订阅 IMU，保留约 1 秒历史，按帧时间戳切分后传给后端。
- 只有最新一帧待处理，后端忙时丢旧帧。
- 读取 `camera.T_left_right` 和 `imu.T_body_camera`，发布静态 TF。
- 把 `T_world_camera` 转成 `T_world_body`；纯双目模式下把世界重定基到第一帧
  机体位姿（FLU）。
- 发布动态 TF、里程计、轨迹、诊断和重置服务。

`mode:=stereo` 时不订阅 IMU；默认 `mode:=imu_stereo`。

### 4.1 话题与服务

| 名称 | 类型 | 说明 |
|---|---|---|
| `/flyrat/odometry/odometry` | `nav_msgs/Odometry` | `odometry_map -> imu_link` |
| `/flyrat/odometry/path` | `nav_msgs/Path` | 机体轨迹，transient local |
| `/flyrat/odometry/status` | `diagnostic_msgs/DiagnosticStatus` | 1 Hz，含 FPS/丢帧/IMU 年龄 |
| `/flyrat/odometry/reset` | `std_srvs/Trigger` | 清空后端状态、轨迹和缓存 |
| `/tf` | TF | 动态 `odometry_map -> imu_link` |
| `/tf_static` | TF | `imu_link -> camera_left_optical_frame`、左目 -> 右目 |

状态消息的 `hardware_id` 为 `flyrat_odometry`，`name` 为 `flyrat_odometry`。

### 4.2 参数

`config/odometry.yaml`：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `left_topic` / `right_topic` | `/flyrat/sensors/stereo/*/image_raw` | 双目输入 |
| `imu_topic` | `/flyrat/sensors/imu` | IMU 输入 |
| `mode` | `imu_stereo` | `imu_stereo` 或 `stereo` |
| `world_frame` | `odometry_map` | 世界系 |
| `body_frame` | `imu_link` | 机体/IMU 系 |
| `camera_frame` / `right_camera_frame` | 光学系 | 相机 TF |
| `camera_config_path` | 由 launch 注入 | 标定 YAML |
| `sync_slop` | `0.010` | 双目同步容差，秒 |
| `imu_buffer_seconds` | `1.0` | IMU 缓存长度 |
| `path_max_poses` | `1000` | 轨迹最大长度 |
| `path_publish_period` | `0.2` | 轨迹发布周期，秒 |
| `status_period` | `1.0` | 诊断发布周期，秒 |
| `trajectory_path` | 空 | 退出时写 TUM 轨迹（后端实现后） |

## 5. 构建与运行

```sh
.script/build_libflyrat.sh
.script/build_ros2.sh
```

以上命令默认只构建节点骨架；TIO-Former 节点需要 libtorch，用
`FLYRAT_BUILD_TIO=ON` 单独构建，步骤见
[`tio_former.md`](tio_former.md#3-构建)。

单独运行节点：

```sh
ros2 launch flyrat_odometry odometry.launch.py
```

实机一键启动（默认 TIO）：

```sh
.script/bringup_real.sh
```

旧立体骨架需要显式选择：

```sh
.script/bringup_real.sh odometry_backend:=stereo
```

不启动里程计、只看传感器：

```sh
.script/bringup_real.sh start_odometry:=false
```

`bringup.launch.py` 会依次启动 adb forward、设备端 forwarder、接收节点和
里程计节点，可选 Foxglove，并在退出时清理。

## 6. 测试

```sh
ctest --test-dir libflyrat/build --output-on-failure -R odometry
```

测试覆盖：标定路径校验、空图像/尺寸不一致拒绝、桩返回
`NOT_INITIALIZED`、`reset/shutdown` 幂等、关闭后 `track()` 拒绝。

## 7. 当前限制

- **以下限制仅针对 legacy 立体骨架**：没有任何位姿输出；
  `/flyrat/odometry/odometry` 不会被发布，`/tf` 只有静态外参。
- 状态固定为 `NOT_INITIALIZED`，诊断级别为 WARN。
- `saveTrajectory()` 未实现，设置 `trajectory_path` 只会在退出时报错。
- 旧双目与 IMU 没有硬件同步，时间对齐仍依赖接收节点的时间戳修正。
- 当前实机已无双目，单目相机还没有标定和视觉后端。
