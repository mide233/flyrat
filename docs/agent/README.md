# flyrat 文档

本目录保存项目全部文档，由 agent 维护。内容按主题组织；第一次接触项目时，
建议先读项目状态，再按需要进入具体模块。

用户维护的飞控资料位于 `docs/drone/`，不属于本目录，也不由 agent 修改。

## 推荐阅读顺序

1. [`status/project_status.md`](status/project_status.md)：项目目标、已完成模块、实测指标、限制和下一步。
2. [`build_and_run.md`](build_and_run.md)：所有组件的编译、测试与运行命令。
3. 按任务选择主题文档：
   - 实机传感器 → [`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
   - 仿真 → [`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
   - SLAM → [`slam/orb_slam3.md`](slam/orb_slam3.md)
   - 坐标系与变换 → [`slam/coordinates.md`](slam/coordinates.md)

## 目录

### 构建与运行

- [`build_and_run.md`](build_and_run.md)
  环境依赖、一键构建脚本、核心库与工具、实机程序、交叉编译、ROS 2、SITL、
  ORB-SLAM3、Docker/CI、清理和常见问题。

### 状态

- [`status/project_status.md`](status/project_status.md)
  当前进度、模块清单、实测指标、已知限制和后续计划。

### 硬件

- [`hardware/sensor_forwarding.md`](hardware/sensor_forwarding.md)
  泰山派上的 IMU、4 颗 VL53L5CX ToF、双目相机如何采集、打包并通过 adb
  转发到开发机 ROS 2。

### 仿真

- [`simulation/betaflight_sitl_gazebo.md`](simulation/betaflight_sitl_gazebo.md)
  Betaflight SITL、Gazebo、ROS 2 控制、Foxglove、相机桥和定位链路。

### SLAM

- [`slam/orb_slam3.md`](slam/orb_slam3.md)
  vendored ORB-SLAM3 的许可证、无界面改造、双目/双目惯性 C++ API、配置、
  `slam_bench`、ROS 2 IMU-stereo 节点和实测结果。
- [`slam/coordinates.md`](slam/coordinates.md)
  IMU/相机坐标系、`IMU.T_b_c1` 方向、TF 链、当前占位外参和逐步调试方法。

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
  `slam/` 等目录。
- `docs/drone/` 是用户资料，不移动、不修改、不提交。
