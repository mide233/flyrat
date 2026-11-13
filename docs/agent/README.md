# flyrat 文档

本目录保存项目全部文档，由 agent 维护。内容按主题组织；第一次接触项目时，
建议先读项目状态，再按需要进入具体模块。

用户维护的飞控资料位于 `docs/drone/`，不属于本目录，也不由 agent 修改。

## 推荐阅读顺序

1. [`status/project_status.md`](status/project_status.md)：项目目标、已完成模块、实测指标、限制和下一步。
2. [`build_and_run.md`](build_and_run.md)：所有组件的编译、测试与运行命令。
3. 按任务选择主题文档：
   - 实机传感器 → [`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
   - 双目/IMU 重新标定 → [`calibration/raw_recording.md`](calibration/raw_recording.md)
   - 仿真 → [`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
   - 里程计 → [`odometry/overview.md`](odometry/overview.md)
   - 坐标系与变换 → [`odometry/coordinates.md`](odometry/coordinates.md)
   - 性能、分阶段计时、MJPEG 解码与优化 → [`odometry/performance.md`](odometry/performance.md)

## 权威文档映射

同一事实只在一处维护，其他文档链接过去：

| 主题 | 权威文档 |
|---|---|
| 构建、运行、ROS 2、SITL、实机启动 | [`build_and_run.md`](build_and_run.md) |
| 传感器转发、录制、回放、splitter | [`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md) |
| 双目/IMU 重新标定的原始录制与导出 | [`calibration/raw_recording.md`](calibration/raw_recording.md) |
| 性能、分阶段计时、MJPEG 解码、优化记录 | [`odometry/performance.md`](odometry/performance.md) |
| 项目状态与实测指标 | [`status/project_status.md`](status/project_status.md) |
| 坐标与标定 | [`odometry/coordinates.md`](odometry/coordinates.md) |
| Betaflight SITL / Gazebo | [`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md) |
| 脚本入口与分派 | [`.script/README.md`](../../.script/README.md) |

## 脚本入口

日常操作优先使用统一入口，参数原样传给具体脚本：

```sh
.script/bringup.sh sensors    # 实机传感器 + 里程计 + 可选 Foxglove
.script/bringup.sh slam       # 机载 SchurVINS + 主机 bridge
.script/bringup.sh raw        # 原始 IMU/双目录制与标定导出
.script/bringup.sh build      # 构建
.script/bringup.sh clean      # 清理构建产物
.script/bringup.sh foxglove   # 只启动 Foxglove bridge
```

完整映射见 [`.script/README.md`](../../.script/README.md)。

## 目录

### 构建与运行

- [`build_and_run.md`](build_and_run.md)
  环境依赖、一键构建脚本、核心库与工具、实机程序、交叉编译、机载 SLAM、
  ROS 2、SITL、里程计、Docker/CI、清理和常见问题。

### 状态

- [`status/project_status.md`](status/project_status.md)
  当前进度、模块清单、实测指标、已知限制和后续计划。

### 硬件

- [`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
  泰山派上的 IMU、4 颗 VL53L5CX ToF、双目相机如何采集、打包并通过 adb
  转发到开发机 ROS 2。

### 标定

- [`calibration/raw_recording.md`](calibration/raw_recording.md)
  原始 IMU 计数和原生 MJPEG 的录制、导出、Kalibr folder 生成，以及重新
  标定后旧数据能否复用。

### 仿真

- [`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
  Betaflight SITL、Gazebo、ROS 2 控制、Foxglove、相机桥和定位链路。

### 里程计

- [`odometry/overview.md`](odometry/overview.md)
  里程计 C++ 接口、SchurVINS 后端、ROS 2 节点与机载模式、话题、参数、
  构建运行方式和后端接入点。
- [`odometry/coordinates.md`](odometry/coordinates.md)
  IMU/相机坐标系、`imu.T_body_camera` 方向、TF 链、当前标定外参和逐步调试方法。

### 报告

- [`reports/arming_and_msp_link.md`](reports/arming_and_msp_link.md)
  关于解锁流程与 MSP 链路的专题报告，保留原始分析。

### 开发环境

- [`development/docker_image.md`](development/docker_image.md)
  开发镜像、本地构建、代理、GitHub Actions 和镜像使用方式。

### 用户资料

- `docs/drone/` 为用户维护的飞控资料，不属于项目文档体系，仓库当前未跟踪。

## 约定

- 文档正文使用中文，文件名和目录名使用英文。
- 构建和运行命令集中在 `build_and_run.md`，主题文档只保留该模块特有的接口、
  设计、数据和限制，避免重复。
- 实测数据应注明设备、配置和测量方式；占位参数必须明确标注。
- 所有项目文档、报告和 agent 生成的中间材料都放在 `docs/agent/`，除非用户明确指定其他位置。
- 新增文档按主题放入对应子目录；一次性材料可以放在本目录或单独子目录，并在文末
  说明生成依据和清理方式。
- 长期维护的内容应整理为正式主题文档，放入 `hardware/`、`simulation/`、
  `odometry/` 等目录。
- `docs/drone/` 是用户资料，不移动、不修改、不提交。
