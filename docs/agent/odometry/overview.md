# 里程计模块

`flyrat_odometry` 是实机双目惯性里程计。默认后端为 **SchurVINS**（SVO 2.0
前端 + Schur complement 滤波后端，CVPR 2024），通过 `libflyrat` 内的适配层
接入，ROS 节点、话题和 TF 接口保持不变。

关闭 `FLYRAT_WITH_SCHURVINS` 时退化为桩实现：节点仍能接收、同步和缓存数据并
发布状态与静态 TF，但位姿无效、状态为 `NOT_INITIALIZED`。

> **许可证**：SchurVINS 及其内置的 SVO/vikit/rpg_common/minkindr/fast 代码为
> **GPL-3.0**，源码位于 `libflyrat/third_party/schurvins/`。链接
> `flyrat_schurvins` 的 `flyrat_odometry` 组合二进制按 GPL-3.0 分发；
> libflyrat 自身源文件仍为 MIT。对外分发前请确认合规范围。

## 1. 组成

| 位置 | 作用 |
|---|---|
| `libflyrat/include/flyrat/odometry/odometry.hpp` | ROS 无关的里程计接口 |
| `libflyrat/src/odometry/odometry.cpp` | 接口门面、模式校验、后端转发 |
| `libflyrat/src/odometry/schurvins_backend.{hpp,cpp}` | 标定解析与 SchurVINS 适配 |
| `libflyrat/third_party/schurvins/` | 内置的 GPL-3.0 SVO/SchurVINS 源码 |
| `libflyrat/configs/camera/uvc_640x480.yaml` | 相机内参、双目外参、IMU 噪声和外参 |
| `libflyrat/tests/test_odometry_api.cpp` | 接口、参数校验和桩/后端行为测试 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/` | ROS 2 节点、参数、launch |
| `docs/agent/odometry/coordinates.md` | 坐标系、TF 链和标定矩阵说明 |

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
- 构造时检查 `camera_config_path` 是否存在；`track()` 检查图像非空、左右尺寸
  一致，并拒绝 `shutdown()` 后的调用。
- 启用 SchurVINS 时只支持 `kImuStereo`，构造 `kStereo` 会抛
  `OdometryError("SchurVINS requires IMU-stereo mode")`。
- `track()` 输出 `kTracking` 和有效的 `T_world_camera` 后，节点再转换为
  `T_world_body` 发布。
- 未启用 SchurVINS 时 `track()` 返回 `kNotInitialized` 和无效位姿，
  `saveTrajectory()` 抛 `OdometryError`。

### 2.1 SchurVINS 后端

- 前端：`svo::FrameHandlerStereo`，双目 `kStereo` 初始化，启用 `USE_SCHUR_VINS`。
- IMU：`svo::ImuHandler` 接收 IMU 样本；噪声密度、随机游走和采样率来自标定
  YAML 的 `imu.*` 字段。
- 相机：`vikit` 针孔 + radial-tangential 畸变模型；`T_C_B` 由
  `imu.T_body_camera` 和 `camera.T_left_right` 推导。
- 位姿：SVO 输出 `T_world_body`，适配层左乘 `imu.T_body_camera` 得到
  `T_world_camera`，符合 `Pose` 的接口约定。
- 轨迹：每个有效位姿追加到内存，`saveTrajectory()` 写 TUM 格式
  （`timestamp tx ty tz qx qy qz qw`）。
- `reset()` 重建整个前端和 IMU 处理器，因为只调用 SVO 的 `reset()` 会留下
  SchurVINS 的 EKF 状态。

**深度与初始化范围（实机）**：适配层按 0.3 m 桌面升降场景调整了 SVO 的默认
参数。双目有效深度为 0.25–20 m（均值 1.5 m），初始化要求 40 个特征、40 个
跟踪点、30 个内点、视差 10 px。上游默认值面向室外（最近 1 m、最远 50 m、
初始化 60/80/70/30），在 0.4–1 m 的桌面场景会长时间初始化失败；这段时间
没有视觉更新，SchurVINS 只靠 IMU 积分，实测 6 s 后初值已漂移数米。

2026-11-10 的实机 bag（28 s，静止 → 升高约 35 cm → 回到原位）回放结果：
首帧即开始跟踪，z 从 0.008 m 升到 0.364 m 再回到 0.016 m；相对起点的最大
水平偏移约 2.5 cm，结束时水平漂移约 1.4 cm；输出约 39–45 Hz。

**日志说明**：`updated seeds are unconverged` 表示深度 seed 尚未收敛，近距离
低视差时常见，不是崩溃；内置代码已把它节流为每 100 次输出一条。
`Downsizing storage not implemented` 是上游 `Frame::resizeFeatureStorage()`
只扩容不缩容的缺陷，内置代码已实现缩容，正常运行时不再出现。

**IMU 时序要求（关键）**：SchurVINS 在每帧时间戳两侧插值 IMU，并在
`schurvinsBackward()` 中对缺失数据执行 `CHECK(0)`。因此后端只有在
`first_imu <= frame.timestamp < last_imu` 时才把帧送入前端；ROS 节点通过
`imu_lookahead` 等待帧后样本，并保留帧前历史。IMU 断流时帧会被跳过并返回
`kRecentlyLost`，不会让进程 abort。

### 2.2 接入新算法

1. 在 `libflyrat/src/odometry/` 增加实现文件，并在 `odometry.cpp` 的门面中
   选择后端。
2. 从 `configs/camera/uvc_640x480.yaml` 读取标定，保持 `T_world_camera` 的
   输出约定。
3. 维护自己的状态机并填 `Result`。
4. 需要输出轨迹时实现 `saveTrajectory()`（TUM 格式）。
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
- 订阅 IMU，保留约 1 秒历史；每帧取 `(上一帧时间, 帧时间 + imu_lookahead]`
  的样本，首帧额外带上约 100 ms 历史。`imu_lookahead` 默认 10 ms。
- 只有最新一帧待处理，后端忙时丢旧帧。
- 读取 `camera.T_left_right` 和 `imu.T_body_camera`，发布静态 TF。
- 把 `T_world_camera` 转成 `T_world_body`；纯双目模式下把世界重定基到第一帧
  机体位姿（FLU）。
- 发布动态 TF、里程计、轨迹、诊断和重置服务。
- 退出时若设置了 `trajectory_path`，写 TUM 轨迹。

SchurVINS 需要 IMU，因此 `mode:=stereo` 只对桩后端有意义；启用 SchurVINS 时
启动会因模式校验失败而退出。默认 `mode:=imu_stereo`。

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
| `mode` | `imu_stereo` | `imu_stereo`（SchurVINS 构建下 `stereo` 不可用） |
| `world_frame` | `odometry_map` | 世界系 |
| `body_frame` | `imu_link` | 机体/IMU 系 |
| `camera_frame` / `right_camera_frame` | 光学系 | 相机 TF |
| `camera_config_path` | 由 launch 注入 | 标定 YAML |
| `sync_slop` | `0.010` | 双目同步容差，秒 |
| `imu_buffer_seconds` | `1.0` | IMU 缓存长度 |
| `imu_lookahead` | `0.010` | 帧后 IMU 前视窗口，秒；必须为正 |
| `imu_initial_history` | `0.1` | 首帧携带的帧前 IMU 历史，秒 |
| `path_max_poses` | `1000` | 轨迹最大长度 |
| `path_publish_period` | `0.2` | 轨迹发布周期，秒 |
| `status_period` | `1.0` | 诊断发布周期，秒 |
| `trajectory_path` | 空 | 退出时写 TUM 轨迹 |

## 5. 构建与运行

```sh
.script/build_libflyrat.sh
.script/build_ros2.sh
```

`FLYRAT_WITH_SCHURVINS` 默认 `ON`，需要 OpenCV、yaml-cpp 和 glog。
只使用接口或在不具备这些依赖的平台上构建：

```sh
cmake -S libflyrat -B libflyrat/build -DFLYRAT_WITH_SCHURVINS=OFF
```

SVO 默认关闭逐帧调试输出；需要排查前端行为时加
`-DFLYRAT_SCHURVINS_VERBOSE=ON`。

单独运行节点：

```sh
ros2 launch flyrat_odometry odometry.launch.py
```

实机一键启动：

```sh
.script/bringup_real.sh
```

不启动里程计、只看传感器：

```sh
.script/bringup_real.sh start_odometry:=false
```

记录轨迹：

```sh
.script/bringup_real.sh trajectory_path:=/tmp/flyrat.tum
```

`bringup.launch.py` 会依次启动 adb forward、设备端 forwarder、接收节点和
里程计节点，可选 Foxglove，并在退出时清理。

### 5.1 机载模式

机载 MJPEG 硬件解码的调研和实测结论见
[`mjpeg_decode_research.md`](mjpeg_decode_research.md)。硬件路径直接调用
MPP 异步接口，`decode_bench` 在 25 fps 下连续 60 s 无停流；`flyrat_pilot`
可通过 `--decode mpp` 启用，默认仍使用 CPU 解码。

开发机模式把双目和 IMU 经 adb 传到主机，再由 `odometry_node` 计算。机载模式
把 SchurVINS 放到 RK3566 上，主机只接收位姿：

```sh
.script/build_pilot_arm.sh
.script/bringup_pilot_slam.sh start_foxglove:=true
```

设备端 `flyrat_pilot` 直接读取 V4L2 MJPEG 和 MSP IMU，在本地完成标定修正与
跟踪，通过 TCP 5001 发送 64 字节 `T_world_body` 位姿。主机
`flyrat_pilot_bridge` 发布与开发机模式同名的话题和 TF，因此 Foxglove 配置可以
复用。IMU 的前视窗口和首帧历史与 ROS 节点一致（默认 10 ms / 100 ms）。
`pilot_bridge` 每收到一帧位姿立即发布；`stamp_source:=host` 用主机接收时间，
`stamp_source:=device` 使用设备 epoch 时间，可用 `time_offset` 补偿时钟偏差。

机载构建使用最小静态 OpenCV，目标端不需要安装 OpenCV；交叉编译和部署细节见
[`build_and_run.md`](../build_and_run.md) 的“机载 SLAM”。

## 6. 测试

```sh
ctest --test-dir libflyrat/build --output-on-failure -R odometry
```

测试覆盖：标定路径校验、空图像/尺寸不一致拒绝、无 IMU 时返回
`NOT_INITIALIZED`、`reset/shutdown` 幂等、关闭后 `track()` 拒绝，以及启用
SchurVINS 时 TUM 轨迹文件可写。

## 7. 当前限制

- 双目与 IMU 没有硬件同步；`imu_lookahead` 只能补偿软件时间戳，不能消除
  曝光/传输抖动。
- 大段 IMU 断流或长时间跟踪失败后，SVO 可能停在 `kPaused`；此时需要调用
  `/flyrat/odometry/reset` 重新初始化，尚未实现自动重初始化。
- 已在单个实机桌面 bag 上验证短时相对尺度和漂移（0.35 m 升降、水平漂移约
  1 cm），尚未做飞行、长距离和长期稳定性验证；`kfselect_*`、`max_n_kfs`
  等参数目前直接写入适配层，尚未暴露为 YAML 参数。
- 实机左目约 55.7 Hz（右目 60 Hz），存在约 8% 丢帧且画面比右目更模糊、
  更亮；当前 bag 上仍可跟踪，但会影响高速运动时的鲁棒性。
- 组合二进制受 GPL-3.0 约束，详见文首。
- 机载模式已在 RK3566 静态桌面场景验证：默认 25 fps，进程 RSS 约 27–30 MB；
  完全静止时跟踪会间歇进入 `RECENTLY_LOST`，移动后恢复，长时间失败需发送
  `SIGUSR1` 重建前端。尚未做飞行和长期漂移验证。
