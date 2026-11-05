# ORB-SLAM3 双目模块

`libflyrat/third_party/orb_slam3/` 是裁剪后的 ORB-SLAM3 核心，
`flyrat::slam::StereoSlam` 是对外的 C++ 门面。当前只支持：

- `Mode::kStereo`：双目视觉 SLAM；
- `Mode::kImuStereo`：双目 + IMU，IMU 由调用方提供。

单目和 RGB-D 的公开 API 没有暴露，核心源码仍保留完整 Tracking 路径。

## 1. 许可证

ORB-SLAM3 采用 **GPLv3**。`third_party/orb_slam3/LICENSE` 保留原许可证；
`include/flyrat/slam/stereo_slam.hpp` 和 `src/slam/stereo_slam.cpp` 使用
`GPL-3.0-or-later`。

链接 `flyrat_slam` 的程序整体应按 GPLv3 分发。不启用 `FLYRAT_BUILD_SLAM`
时，`libflyrat` 其余部分仍按原许可证使用。

## 2. 构建

SLAM 默认关闭，因为它依赖 OpenCV、Boost.Serialization、OpenSSL，并需要约
139 MB 的 ORB 词袋文件。配置选项、依赖安装和构建命令见
[`build_and_run.md`](../build_and_run.md) 的“ORB-SLAM3”一节。

构建产物：

- `libflyrat_orb_dbow2.a`
- `libflyrat_orb_g2o.a`
- `libflyrat_orb_core.a`
- `libflyrat_slam.a`
- `slam_bench`
- `test_slam_api`

`FLYRAT_BUILD_SLAM=OFF` 时不会引入 OpenCV、Boost.Serialization 或 OpenSSL。

## 3. 词袋

词袋文件不入库。ORB-SLAM3 只支持它的文本格式，当前版本没有二进制加载接口。
可以从 ORB-SLAM3 的 `Vocabulary/ORBvoc.txt` 复制，并通过
`-DFLYRAT_ORB_VOCAB=/path/to/ORBvoc.txt` 传给构建，或运行时用
`Options::vocabulary_path` / `slam_bench --vocab` 指定。

`FLYRAT_ORB_VOCAB` 只用于给测试和本地构建提供路径，运行时仍由
`Options::vocabulary_path` 决定。

## 4. 配置

模板：

- `libflyrat/configs/orb_slam3/uvc_640x480.yaml`：双目；
- `libflyrat/configs/orb_slam3/uvc_640x480_imu.yaml`：双目 + IMU。

里面的内参、基线、`IMU.T_b_c1`、噪声参数都是**占位值**。直接用可以跑通，
但尺度、深度和轨迹精度不可信。正式使用前必须标定：

- 左右目内参和畸变；
- `Stereo.T_c1_c2`（右目在左目坐标系）；
- `IMU.T_b_c1`（左目在 IMU/body 坐标系）；
- IMU 噪声密度和随机游走。

注意 `Stereo.T_c1_c2` 的方向与 `cv::stereoCalibrate` 返回的
`X2 = R * X1 + T` 相反，写入前要取逆。

当前 ROS 2 节点使用的 `IMU.T_b_c1` 是占位值，按“相机倒置、光轴正对 IMU 前方、
平移为 0”构造，body 轴为 FLU：

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

它表示 `T_b_c1`：把左目相机光学坐标变换到 IMU body 坐标（相机 → body）。
相机绕光轴倒置，光轴与 body 的 x 轴（前方）平行。平移和安装角都没有实测标定，
不能用于精度评估。矩阵方向、列的含义和调整方法见
[`coordinates.md`](coordinates.md)。

## 5. C++ API

```cpp
#include "flyrat/slam/stereo_slam.hpp"

flyrat::slam::Options options;
options.mode = flyrat::slam::Mode::kStereo;
options.vocabulary_path = "/path/to/ORBvoc.txt";
options.settings_path = "libflyrat/configs/orb_slam3/uvc_640x480.yaml";

flyrat::slam::StereoSlam slam(options);

// left/right 是已经拆开的单眼图像，CV_8UC1 或 CV_8UC3(BGR)。
// timestamp 单位为秒，必须单调递增。
auto result = slam.track(left, right, timestamp_s);

if (result.pose.valid) {
    const Eigen::Vector3d& p = result.pose.position;       // T_world_camera
    const Eigen::Quaterniond& q = result.pose.orientation;
}
```

约定：

- 世界系是 ORB-SLAM3 第一次成功初始化时的左目相机位姿；
- `position` / `orientation` 是 `T_world_camera`，不是 ORB-SLAM3 内部的
  `Tcw`；门面已经取逆；
- `track()` 只能单线程调用；
- `reset()` 丢弃当前地图，下一帧重新初始化；
- `shutdown()` 幂等，之后不能继续 `track()`；
- `saveTrajectory()` 必须在 `shutdown()` 之后调用，输出 TUM 格式。

双目惯性模式使用重载：

```cpp
std::vector<flyrat::slam::ImuSample> imu;
// timestamp: 秒；accel: m/s^2；gyro: rad/s
auto result = slam.track(left, right, timestamp_s, imu);
```

IMU 样本应覆盖上一帧到当前帧的时间段，按时间升序。调用方负责把
Betaflight 的原始计数换算成 SI 单位，并处理轴向。

## 6. ROS 2 双目/双目惯性节点

ROS 2 包 `flyrat_play/ros2_ws/src/flyrat_slam/` 把实机转发的双目和 IMU 直接接入
`flyrat::slam::StereoSlam`。参数 `mode` 选择模式：

- `imu_stereo`（默认）：`Mode::kImuStereo`，订阅双目和 IMU；
- `stereo`：`Mode::kStereo`，只订阅双目，不订阅 IMU。

```sh
ros2 launch flyrat_slam slam.launch.py mode:=stereo \
  vocabulary_path:=/path/to/ORBvoc.txt
```

纯双目模式仍需要 `IMU.T_b_c1` 来生成 `imu_link -> camera_left_optical_frame`
静态 TF 和机体位姿；默认的 `uvc_640x480_imu.yaml` 可以直接用，ORB-SLAM3 在
`Stereo` 模式下会忽略其中的 IMU 噪声参数。

订阅：

- `/flyrat/sensors/stereo/left/image_raw`
- `/flyrat/sensors/stereo/right/image_raw`
- `/flyrat/sensors/imu`

双目用 `message_filters::ApproximateTime` 同步（默认 `sync_slop=0.010` s），
IMU 缓存默认 1 s。图像回调只保留最新待处理帧，跟踪线程忙时丢旧帧，避免延迟
累积。跟踪线程用左目时间戳，把上一帧到当前帧之间的 IMU 样本传给
`StereoSlam::track()`。

发布：

- `/flyrat/slam/odometry`：`nav_msgs/msg/Odometry`，`slam_map -> imu_link`，
  位姿是 `T_world_body`（门面输出的 `T_world_camera` 再乘 `T_camera_body`）。
- `/flyrat/slam/path`：`nav_msgs/msg/Path`，限频发布，保留最近
  `path_max_poses` 个位姿，同样是机体轨迹。
- `/flyrat/slam/status`：`diagnostic_msgs/msg/DiagnosticStatus`，1 Hz，包含状态、
  输入/处理帧率、丢帧、失败帧、IMU 缓冲和 age。
- `/flyrat/slam/reset`：`std_srvs/srv/Trigger`，清空地图、轨迹和 IMU 缓冲。
- TF：动态 `slam_map -> imu_link`，静态 `imu_link -> camera_left_optical_frame`
  （由 `IMU.T_b_c1` 求逆得到）。

`camera_left_optical_frame` 是标准光学系（x 右、y 下、z 前）。当前占位外参是
“相机倒置、光轴正对机体前方”，所以相机光学系的 z 轴与机体 x 轴平行；机体绕 x
轴滚转时，在光学系里就是绕光轴 z 的偏航。这是坐标系分解，不是 SLAM 把滚转算成
了偏航。Foxglove 里应把固定坐标系设为 `imu_link` 或 `slam_map`，观察机体轨迹，
而不是直接看 `camera_left_optical_frame` 的轴。

参数在 `config/slam.yaml`，启动时可用 `-p` 覆盖。`vocabulary_path` 必填；
`settings_path` 默认使用安装后的 `uvc_640x480_imu.yaml`；`body_frame` 默认
`imu_link`；`trajectory_path` 非空时在节点退出时保存 TUM 轨迹。

实机一键启动（adb forward + 设备 forwarder + 接收节点 + SLAM）：

```sh
.script/bringup_real.sh
.script/bringup_real.sh start_foxglove:=true
```

`bringup.launch.py` 会自动查找词袋和源码树中的 settings，`Ctrl-C` 时清理
adb forward 和设备 forwarder。参数和单独启动方式见
[`../build_and_run.md`](../build_and_run.md) 的“一键 bringup”。

`uvc_640x480_imu.yaml` 默认设置 `IMU.fastInit: 1`，用于跳过 ORB-SLAM3 的
逐帧加速度变化门限，方便桌面/手持调试；它不能替代真正的 IMU 初始化。
静止或只有缓慢滚转时，日志仍会出现 `Not enough motion for initializing`，
需要平移、加速或明显晃动才能完成重力/尺度对齐。正式飞行前可改回 `0`。

节点要求 IMU 已按 ROS FLU 发布。当前实机 `sensor_receiver_node` 默认
`imu_accel_axis=none`、`imu_gyro_axis=bf_to_flu`：加速度原始数据已是 FLU，
陀螺仪只有 z 需要取反。若原始约定不同，按
[`sensor_forwarding.md`](../hardware/sensor_forwarding.md) 调整，否则
`IMU.T_b_c1`、重力方向和 IMU 预积分都会不一致。

坐标链、`T_b_c1` 的方向、当前占位矩阵和逐步调试方法见
[`coordinates.md`](coordinates.md)。

构建和启动命令见 [`build_and_run.md`](../build_and_run.md) 的“ROS 2 SLAM 节点”
一节。Foxglove 只作为可视化工具，不提交布局文件。

## 7. 命令行

`slam_bench` 的通用参数和启动示例见
[`build_and_run.md`](../build_and_run.md)。它支持三种输入：

| 输入 | 参数 | 时间戳 |
|---|---|---|
| 左右图像序列 | `--left DIR --right DIR` | 按 `--fps` 生成 |
| 左右拼接视频 | `--video FILE` | 按 `--fps` 生成 |
| 实时 V4L2 相机 | `--sbs /dev/video0` | 单调时钟 |

加 `--mode imu-stereo` 使用双目惯性模式，`--save-traj FILE` 保存 TUM 轨迹。
运行结束会打印跟踪帧数、平均/最大耗时和峰值 RSS。

## 8. 实测

开发机（x86_64，OpenCV 4.6，Release，占位内参）：

```text
90 帧真实双目序列（1280x480 MJPEG，拆成 2×640x480）
tracked=90/90
avg=11.53–12.67 ms/frame (79–87 fps)
max=17.87 ms
peak RSS=547 MB
```

同一序列在未启用 SLAM 的构建上仍保持原有 `ctest` 8/8；启用 SLAM 后为
`ctest` 9/9。配置 `FLYRAT_ORB_VOCAB` 时，`test_slam_api` 会额外加载真实
词袋并构造/销毁 `System`（约 3.5 秒）。

ROS 2 节点用实机转发数据实测（RK3566 采集、adb 转发、开发机 tracking）：

```text
IMU 输入             200.3–200.8 Hz
双目输入             60.0 Hz（左右各 640x480 mono8）
SLAM 输入/处理       60.0 Hz，启动后 dropped=1
状态发布             1 Hz
slam_node RSS        约 557 MB（含词袋和 ORB 地图）
```

当前桌面场景近似静止、视场内容少。启用 `IMU.fastInit: 1` 后，节点可以进入
`TRACKING` 并发布 `slam_map -> imu_link`，输入/处理约 60 Hz，`dropped_frames`
约 1–2；但日志仍会因缺少平移/加速度激励反复出现
`Not enough motion for initializing`，世界系尚未完成重力对齐。因此这些位姿
只能用于验证链路、TF 和 Foxglove 预览，不能作为可重复的轨迹精度数据。
`/flyrat/slam/reset` 服务已实测返回成功。

## 9. 无界面改造

上游 ORB-SLAM3 默认链接 Pangolin 并启动 Viewer 线程。本模块做了以下裁剪：

- 删除 `Viewer`、`MapDrawer` 和 Pangolin 依赖；
- `System` 保持 `bUseViewer` 参数兼容，但门面固定传 `false`；
- 保留 `FrameDrawer`（只依赖 OpenCV），用于生成跟踪图像；
- `Map` 的缩略图类型从 `GLubyte*` 改为 `unsigned char*`；
- 上游的 `exit(-1)` 配置错误改为抛出异常，由门面转换成 `SlamError`。

## 10. 已知限制

- 当前只在开发机验证，没有在泰山派 aarch64 上构建；泰山派没有 OpenCV，
  且内存只有 2 GB。
- 内参、外参和 IMU 参数是占位值；需要完整标定。
- 实机节点已经直接订阅 `/flyrat/sensors/*`，但桌面静止场景无法稳定初始化；
  需要真实运动、有纹理的视场验证 `TRACKING`。
- 双目和 IMU 没有硬件同步；`sensor_receiver_node` 只用 1 Hz 的 `TIME` 帧估计
  设备到主机的偏移，避免把 JPEG 解码时间算进图像时间戳。
- 图像同步窗口默认 10 ms；左右目时间戳差距更大时会丢帧。
- 未完成 IMU 初始化前，ORB-SLAM3 的世界系还没有对齐重力，此时 `imu_link`
  的绝对姿态不能当作真实机体姿态使用。
- 词袋 139 MB，加载约数秒；启动延迟明显。
- `reset()` 是异步生效，下一帧才会真正重建地图。
- GPLv3 会传播到链接本模块的程序。
