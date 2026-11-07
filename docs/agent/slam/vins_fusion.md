# VINS-Fusion

`libflyrat` 的 SLAM 模块使用 VINS-Fusion 双目惯性后端。本文记录 vendored
版本、C++ API、配置、构建/运行方式和已知限制。

## 1. 上游与许可证

- 上游：<https://github.com/HKUST-Aerial-Robotics/VINS-Fusion>
- commit：`be55a937a57436548ddfb1bd324bc1e9a9e828e0`
- 许可证：GNU GPL v3.0，全文见
  `libflyrat/third_party/vins_fusion/LICENSE`
- 代码位置：`libflyrat/third_party/vins_fusion/`

只 vendor 运行所需的 estimator、feature tracker、factor、initial 和
`camera_models`。ROS 1 节点、可视化、标定工具和测试图片不在仓库内。

## 2. 构建

```sh
sudo apt install libopencv-dev libeigen3-dev libboost-dev libceres-dev

cmake -S libflyrat -B libflyrat/build-slam \
  -DFLYRAT_BUILD_SLAM=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build libflyrat/build-slam -j
```

生成：

- `flyrat_vins_core`：vendored VINS 静态库
- `flyrat_slam`：`flyrat::slam` facade
- `slam_bench`
- `test_slam_api`

`FLYRAT_BUILD_SLAM` 默认 OFF，因为它引入 OpenCV、Ceres、Eigen 和 Boost。
ROS 2 的 `flyrat_slam` 包在配置时强制打开。

## 3. C++ API

头文件：`libflyrat/include/flyrat/slam/stereo_slam.hpp`

```cpp
flyrat::slam::Options options;
options.config_path = "libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml";

flyrat::slam::StereoSlam slam(options);

flyrat::slam::ImuSample imu;
imu.timestamp = 1.0;
imu.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
imu.gyro = Eigen::Vector3d::Zero();

auto result = slam.track(left, right, 1.0, {imu});
if (result.pose.valid) {
    // result.pose 是 T_world_body
}
```

接口只支持双目 + IMU。`track()` 的 `imu` 必须包含截至图像时间戳的 IMU 样本；
图像可以是 mono8/BGR8/BGRA8，内部转灰度；ROS 节点会按 encoding 先把 RGB/RGBA 转成灰度。

状态：

- `kNotInitialized`：尚未完成 VINS 初始化。
- `kTracking`：`solver_flag == NON_LINEAR` 且位姿有限。
- `kLost`：曾经进入 tracking，但 VINS 回退到 `INITIAL`。

`Pose` 是 `T_world_body`，body 为 IMU 系。VINS 的 world 系在初始化时定义：
重力对齐，初始 yaw 由第一帧窗口决定。

`reset()` 清空状态和轨迹；`shutdown()` 释放 estimator 但保留已累积轨迹；
`saveTrajectory(path)` 写 TUM 格式。

### 3.1 单实例限制

VINS 的参数（`RIC`、`TIC`、`CAM_NAMES`、`NUM_OF_CAM`、`USE_IMU` 等）是进程级
全局变量。一个进程只能有一个 `StereoSlam` 实例；adapter 用静态标志强制检查，
第二个实例会抛 `SlamError`。`shutdown()` 后可以创建新实例。

### 3.2 线程模型

adapter 强制 `multiple_thread=0`，在调用 `track()` 的线程内同步完成特征跟踪和
滑窗优化。调用方仍需保证 `track()`、`reset()`、`shutdown()` 不并发进入同一
实例；facade 内部有互斥锁。

## 4. 配置

主配置：`libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml`
相机内参：`cam0_pinhole.yaml`、`cam1_pinhole.yaml`

关键字段：

| 字段 | 说明 |
|---|---|
| `imu` | 必须为 1 |
| `num_of_cam` | 必须为 2 |
| `cam0_calib` / `cam1_calib` | 相对主配置解析 |
| `image_width` / `image_height` | 必须与输入图像一致 |
| `body_T_cam0` / `body_T_cam1` | 相机 → body，4x4 |
| `multiple_thread` | adapter 强制为 0 |
| `max_cnt` / `min_dist` | 特征数量和最小间距 |
| `max_solver_time` / `max_num_iterations` | 滑窗优化预算 |
| `acc_n` / `gyr_n` / `acc_w` / `gyr_w` | IMU 噪声和随机游走 |
| `g_norm` | 重力模长，默认 9.81 |
| `estimate_td` / `td` | 时间偏移；当前接收节点已补偿，设为 0 |

`body_T_cam1 = body_T_cam0 * Stereo.T_c1_c2`。外参和标定细节见
[`coordinates.md`](coordinates.md)。

## 5. slam_bench

IMU CSV 无表头：

```text
timestamp,ax,ay,az,gx,gy,gz
```

时间戳单位秒，加速度 m/s²，角速度 rad/s。图像帧时间戳从第一帧 IMU 时间戳开始
按 `--fps` 递增。

```sh
./libflyrat/build-slam/slam_bench \
  --config libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml \
  --imu imu.csv \
  --left /path/to/left --right /path/to/right \
  --fps 30 --save-traj /tmp/traj.txt
```

左右拼接视频用 `--video FILE`。`--realtime` 按 fps 限速，`--max-frames` /
`--seconds` 限制长度。实时 UVC 采集由 ROS 2 节点负责。

## 6. ROS 2 节点

`flyrat_play/ros2_ws/src/flyrat_slam` 把 `/flyrat/sensors/imu` 和双目话题接入
`StereoSlam`。

主要参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `vins_config` | 包内 `config/uvc_640x480_imu.yaml` | VINS 配置 |
| `left_topic` / `right_topic` | `/flyrat/sensors/stereo/...` | 双目话题 |
| `imu_topic` | `/flyrat/sensors/imu` | IMU 话题 |
| `world_frame` | `slam_map` | 世界系 |
| `body_frame` | `imu_link` | 机体/IMU 系 |
| `camera_frame` / `right_camera_frame` | 光学系 | 静态 TF 子帧 |
| `sync_slop` | 0.010 | 双目同步容差 |
| `imu_buffer_seconds` | 1.0 | IMU 历史长度 |
| `trajectory_path` | 空 | 退出时写 TUM 轨迹 |

输出：

```text
/flyrat/slam/odometry   nav_msgs/Odometry
/flyrat/slam/path       nav_msgs/Path
/flyrat/slam/status     diagnostic_msgs/DiagnosticStatus
/flyrat/slam/reset      std_srvs/Trigger
```

静态 TF 直接来自 `body_T_cam0` / `body_T_cam1`；动态 TF 为
`slam_map -> imu_link`。状态 `hardware_id` 为 `vins_fusion`。

节点每帧发送当前 IMU 缓冲区内的样本，可能略新于图像时间戳；VINS 内部会按
图像时刻截取区间，把剩余样本留在自己的缓冲区。没有 IMU 时会等待，不推进帧
时间戳。

## 7. 已知限制

- 没有回环检测和位姿图，长距离会漂移。
- VINS 初始化需要约 10 帧窗口和足够的运动/IMU 激励；静止桌面通常停在
  `NOT_INITIALIZED`。
- 上游 `failureDetection()` 在当前版本中被禁用；adapter 通过 `solver_flag`
  从 `NON_LINEAR` 回退到 `INITIAL` 判断丢失。丢失后需要调用 `/flyrat/slam/reset`
  或重启节点。
- VINS 参数是全局变量，一个进程只能有一个实例。
- 当前 `MULTIPLE_THREAD=0` 优先确定性；目标机帧率需要实测。

## 8. 验证

```sh
ctest --test-dir libflyrat/build-slam --output-on-failure
ros2 launch flyrat_slam slam.launch.py --show-args
ros2 launch flyrat_slam bringup.launch.py --show-args
```

实机验证时在 Foxglove 中观察 `/flyrat/slam/odometry`、`/flyrat/slam/path` 和
`/flyrat/slam/status`。初始化完成后，手持设备缓慢平移和偏航，确认
`imu_link` 的运动方向与真实机体一致。
