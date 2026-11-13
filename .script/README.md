# 脚本入口

日常操作优先使用统一入口：

```sh
.script/bringup.sh sensors    # 实机传感器 + 里程计 + 可选 Foxglove
.script/bringup.sh slam       # 机载 SchurVINS + 主机 bridge
.script/bringup.sh raw        # 原始 IMU/双目录制与标定导出
.script/bringup.sh build      # 构建 host / arm / ROS 2 / 仿真
.script/bringup.sh clean      # 清理构建产物
.script/bringup.sh foxglove   # 只启动 Foxglove bridge
```

`bringup.sh` 内置 sensors/slam/raw 三条实机流程，参数原样传入；
build/clean/foxglove 分派给对应的专用脚本。

## 具体脚本

| 脚本 | 用途 | 权威文档 |
|---|---|---|
| `bringup.sh` | 统一入口，内置三种实机流程 | 本文件 |
| `bringup.sh sensors` | 实机 forwarder + ROS 2 接收/里程计 | `docs/agent/build_and_run.md` |
| `bringup.sh slam` | 机载 SLAM、录制/回放、trace | `docs/agent/build_and_run.md` |
| `bringup.sh raw` | 原始 IMU/相机录制、预览、导出 | `docs/agent/calibration/raw_recording.md` |
| `build_all.sh` | host、arm、ROS 2、SITL 统一构建 | `docs/agent/build_and_run.md` |
| `build_pilot_arm.sh` | 只交叉编译 RK3566 程序 | `docs/agent/build_and_run.md` |
| `build_ros2.sh` | 只构建 ROS 2 workspace | `docs/agent/build_and_run.md` |
| `build_betaflight.sh` | 构建 Betaflight SITL | `docs/agent/build_and_run.md` |
| `build_plugin.sh` | 构建 Gazebo System plugin | `docs/agent/build_and_run.md` |
| `clean_build.sh` | 清理构建产物 | `docs/agent/build_and_run.md` |
| `foxglove.sh` | Foxglove bridge | `docs/agent/build_and_run.md` |
| `camera.sh` | Gazebo 相机桥 | `docs/agent/simulation/betaflight_sitl_gazebo.md` |
| `export_compile_commands.sh` | 合并 clangd 数据库 | `docs/agent/build_and_run.md` |

## 约定

- 所有实机脚本都支持 `ADB_SERIAL`；多设备时必须设置。
- `ROS_DISTRO` 默认 `jazzy`，`FLYRAT_ROS_WS` 默认 `<repo>/flyrat_play/ros2_ws`。
- 设备端二进制默认放在 `/tmp`，由脚本推送；不要手工覆盖正在运行的副本。
- 实机脚本退出时会清理自己启动的进程和 `adb forward`；录制目录不会自动删除，
  确认已 pull 后再手动清理。
