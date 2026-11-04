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

SLAM 默认关闭，因为它依赖 OpenCV、Boost.Serialization、OpenSSL，并需要一个
约 139 MB 的 ORB 词袋文件。

```sh
cmake -S libflyrat -B build-slam \
  -DFLYRAT_BUILD_SLAM=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-slam -j
```

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
可以从 ORB-SLAM3 的 `Vocabulary/ORBvoc.txt` 复制，或在配置时指定：

```sh
cmake -S libflyrat -B build-slam \
  -DFLYRAT_BUILD_SLAM=ON \
  -DFLYRAT_ORB_VOCAB=/path/to/ORBvoc.txt
```

`FLYRAT_ORB_VOCAB` 只用于给测试和本地构建提供路径，运行时仍由
`Options::vocabulary_path` 指定。

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

## 6. 命令行

左右目图像序列：

```sh
./build-slam/slam_bench \
  --vocab /path/to/ORBvoc.txt \
  --settings libflyrat/configs/orb_slam3/uvc_640x480.yaml \
  --left /tmp/uvc_stereo/left \
  --right /tmp/uvc_stereo/right \
  --fps 30 \
  --save-traj /tmp/traj.txt
```

左右拼接视频：

```sh
./build-slam/slam_bench --vocab ... --settings ... \
  --video recording.avi --fps 60
```

实时 UVC（开发机直接接相机时）：

```sh
./build-slam/slam_bench --vocab ... --settings ... \
  --sbs /dev/video0 --width 1280 --height 480 --fps 60 --seconds 20
```

`slam_bench` 会打印每帧状态、`T_world_camera`、单帧耗时，最后输出跟踪帧数、
平均耗时、最大耗时和峰值 RSS。实时模式的时间戳来自单调时钟；图像序列模式
按 `--fps` 生成。

## 7. 实测

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

## 8. 无界面改造

上游 ORB-SLAM3 默认链接 Pangolin 并启动 Viewer 线程。本模块做了以下裁剪：

- 删除 `Viewer`、`MapDrawer` 和 Pangolin 依赖；
- `System` 保持 `bUseViewer` 参数兼容，但门面固定传 `false`；
- 保留 `FrameDrawer`（只依赖 OpenCV），用于生成跟踪图像；
- `Map` 的缩略图类型从 `GLubyte*` 改为 `unsigned char*`；
- 上游的 `exit(-1)` 配置错误改为抛出异常，由门面转换成 `SlamError`。

## 9. 已知限制

- 当前只在开发机验证，没有在泰山派 aarch64 上构建；泰山派没有 OpenCV，
  且内存只有 2 GB。
- 内参、外参和 IMU 参数是占位值；需要完整标定。
- 开发机上的双目图像来自设备录制后回放，尚未把 `/flyrat/sensors/*` 直接
  喂给 SLAM。
- 双目和 IMU 没有硬件同步；IMU 模式需要调用方自行做时间对齐。
- 词袋 139 MB，加载约数秒；启动延迟明显。
- `reset()` 是异步生效，下一帧才会真正重建地图。
- GPLv3 会传播到链接本模块的程序。
