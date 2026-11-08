# TIO-Former ToF-惯性里程计

本页记录研究模型 TIO-Former 在 `libflyrat` / ROS 2 中的实验性接入。它是
`research/tio_former/` 的推理后端，**不替代**双目/双目惯性骨架
[`overview.md`](overview.md)。当前模型完全在 Gazebo 仿真数据上训练，尚未在
实机飞行数据上验证。

## 1. 组成

| 位置 | 作用 |
|---|---|
| `research/tio_former/` | 数据集生成、PyTorch 模型、训练与评估 |
| `research/tio_former/scripts/export_tio.py` | 将 checkpoint 导出为 TorchScript |
| `libflyrat/models/tio_former_window64.pt` | 导出的 TorchScript 模型（4.6 MB） |
| `libflyrat/models/tio_former_window64.json` | 模型元数据（窗口、视角、参数量） |
| `libflyrat/include/flyrat/tio/tio_odometry.hpp` | C++ 后端接口 |
| `libflyrat/src/tio/tio_odometry.cpp` | 滑动窗口、IMU 插值、推理和增量积分 |
| `libflyrat/tools/tio_bench.cpp` | 合成输入基准程序 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/src/tio_odometry_node.cpp` | ROS 2 节点 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/launch/tio_odometry.launch.py` | 单节点启动 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/config/tio_odometry.yaml` | 参数 |

模型输入为 `[1, 64, 4, 8, 8]` 的 ToF 距离、`[1, 64, 4, 8, 8]` 的可靠性权重和
`[1, 64, 16, 6]` 的 IMU 窗口；输出为 64 个逐步的机体坐标系
`delta_p` / `delta_phi`。C++ 后端只取最后一帧的增量，并按
`p_t = p_{t-1} + R_{t-1} delta_p`、`R_t = R_{t-1} exp(delta_phi)` 积分。

视角顺序固定为 **front / left / right / down**，与训练数据一致。

## 2. C++ 接口

```cpp
#include "flyrat/tio/tio_odometry.hpp"

flyrat::tio::Options options;
options.model_path = "libflyrat/models/tio_former_window64.pt";
options.warmup_frames = 64;  // 约 4.3 s @ 15 Hz

flyrat::tio::TioOdometry odometry(options);
odometry.addImu(imu_sample);
auto result = odometry.track(
    {front_grid, left_grid, right_grid, down_grid}, timestamp);
```

- `TofGrid::range` 单位为米，`weight` 为 `[0, 1]`。无效 zone 的距离应填 0、
  权重填 0。
- `Result::pose` 是 **`T_world_body`**，不是双目接口的 `T_world_camera`。
- `warmup_frames` 必须不大于 `window_frames`（64）。窗口不足时状态为
  `NOT_INITIALIZED`；窗口填满后第一帧位姿为单位位姿，之后的预测才被积分。
  不要把 `warmup_frames` 降到很小的值：窗口前部会用零填充，与训练分布不一致。
- `addImu()` 与 `track()` 线程安全；`reset()` / `shutdown()` 不应与
  `track()` 并发调用。
- `saveTrajectory()` 写 TUM 格式：`timestamp tx ty tz qx qy qz qw`。

## 3. 构建

TIO 后端依赖 libtorch，因此默认关闭。普通 `libflyrat` 构建不需要 Torch。

### 3.1 CPU libtorch

CUDA 版 Python wheel 不包含 nvcc/CUDA toolkit，不能直接用于 C++ CMake。开发机
使用 CPU wheel：

```sh
cd research/tio_former
python3 -m venv .venv-cpu
.venv-cpu/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
cd ../..

TORCH_CMAKE="$PWD/research/tio_former/.venv-cpu/lib/python3.12/site-packages/torch/share/cmake"
```

### 3.2 libflyrat

```sh
cmake -S libflyrat -B libflyrat/build-tio \
  -DFLYRAT_BUILD_TIO=ON \
  -DFLYRAT_BUILD_TESTS=OFF \
  -DFLYRAT_BUILD_TOOLS=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$TORCH_CMAKE"
cmake --build libflyrat/build-tio -j"$(nproc)"
ctest --test-dir libflyrat/build-tio --output-on-failure -R tio
libflyrat/build-tio/tio_bench libflyrat/models/tio_former_window64.pt 40
```

基准程序使用合成距离和零 IMU，只验证模型加载与推理链路，不代表定位精度。

### 3.3 ROS 2

默认 `colcon build` 不构建 TIO 节点。需要显式打开：

```sh
bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd flyrat_play/ros2_ws
  colcon build --symlink-install \
    --packages-select flyrat_sensors_interfaces flyrat_sensors flyrat_odometry \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DFLYRAT_BUILD_TIO=ON \
      -DCMAKE_PREFIX_PATH='"$TORCH_CMAKE"'
'
```

包会把 `tio_former_window64.pt` 和 `.json` 安装到
`share/flyrat_odometry/models/`。

## 4. ROS 2 节点

```sh
ros2 launch flyrat_odometry tio_odometry.launch.py
```

节点订阅：

```text
/flyrat/sensors/tof_0 .. tof_3   flyrat_sensors_interfaces/TofFrame
/flyrat/sensors/imu              sensor_msgs/Imu
```

`TofFrame` 的 8×8 或 4×4 分辨率都可接受；4×4 会最近邻上采样到 8×8。4×4
只是兼容路径，模型训练使用 8×8。`flyrat_sensor_forwarder` 默认已改为
**8×8 @ 15 Hz**；节点还会按 `tof_rate_hz` 对过快到达的同步帧做节流，避免
4×4@60 Hz 时把“每 66.7 ms 的增量”按 60 Hz 积分。
zone 的 `valid` 为真且距离在 `[0.015, 4.0] m` 时权重为 1，否则距离和权重都为 0。

发布与双目节点相同的接口：

```text
/flyrat/odometry/odometry   nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path       nav_msgs/Path，transient local
/flyrat/odometry/status     diagnostic_msgs/DiagnosticStatus，1 Hz
/flyrat/odometry/reset      std_srvs/Trigger
/tf                         odometry_map -> imu_link
```

状态消息的 `name` 为 `flyrat_tio_odometry`，`hardware_id` 为 `tio_former`，
并报告输入/处理/位姿发布/丢弃/失败帧数和 IMU 样本数。

### 4.1 实机 ToF 方向映射

RK3566 的 GPIO 编号为 `bank*32 + group*8 + pin`。当前实机 INT/LPn 接线：

| 传感器 ID | 方向 | INT | LPn | Linux GPIO |
|---:|---|---|---|---|
| 0 | 下 | 3B3 | 3B4 | 107 / 108 |
| 1 | 后 | 3B0 | 3C2 | 104 / 114 |
| 2 | 左 | 3A4 | 3A5 | 100 / 101 |
| 3 | 右 | 3A1 | 3A2 | 97 / 98 |

`TofArray` 的默认 GPIO 数组与上表一致，`/flyrat/sensors/tof_0..3` 按传感器
ID 发布。模型训练视角为 front/left/right/down，节点用
`sensor_order: [1, 2, 3, 0]` 映射为：

```text
模型 front <- 传感器 1（实机后向）
模型 left  <- 传感器 2（实机左向）
模型 right <- 传感器 3（实机右向）
模型 down  <- 传感器 0（实机下向）
```

**当前机架没有前向 ToF**，所以模型 front 槽位暂时喂后向数据。这会破坏模型
学到的“前向”视角语义；用于实机精度验证前，应加装前向传感器，或重新生成
rear/left/right/down 布局的数据并重新训练/导出模型。节点启动时会打印映射，
并在 front 被喂后向时给出警告。

主要参数（`config/tio_odometry.yaml`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model_path` | launch 注入 | TorchScript 模型 |
| `world_frame` / `body_frame` | `odometry_map` / `imu_link` | TF 坐标系 |
| `warmup_frames` | `64` | 首帧位姿前的 ToF 帧数 |
| `tof_rate_hz` / `imu_buffer_seconds` | `15` / `2.0` | 时间基准 |
| `sensor_order` | `[1,2,3,0]` | 视角到传感器编号 |
| `path_max_poses` / `path_publish_period` | `1000` / `0.2` | 轨迹发布 |
| `trajectory_path` | 空 | 退出时写 TUM 轨迹 |

### 4.2 一键 bringup

```sh
.script/bringup_real.sh odometry_backend:=tio
.script/bringup_real.sh odometry_backend:=tio start_foxglove:=true
```

`bringup_real.sh` 会把参数原样传给 `bringup.launch.py`。默认
`odometry_backend:=stereo` 使用双目骨架；设为 `tio` 后启动
`tio_odometry_node`，不再需要 `camera_config_path`。

## 5. 仿真回放

`research/tio_former/scripts/publish_processed_ros.py` 可以把一条处理后的 NPZ
序列发布成 ROS 话题，用于无 Gazebo 的端到端检查。使用实机默认映射时，把
仿真 front/left/right/down 分别发到 `tof_1/tof_2/tof_3/tof_0`：

```sh
cd research/tio_former
.venv/bin/python scripts/publish_processed_ros.py \
  data/processed/test_corridor_0000.npz --rate 15 --sensor-order 1 2 3 0
```

另一个终端启动节点后检查：

```sh
ros2 topic echo /flyrat/odometry/status --once
ros2 topic echo /flyrat/odometry/odometry --once
ros2 run tf2_ros tf2_echo odometry_map imu_link
```

开发机实测（开发机 CPU TorchScript 推理，仿真序列）：节点进入
`TRACKING`，输入/处理/丢弃/失败计数正常，并成功发布 `nav_msgs/Odometry`
与 `odometry_map -> imu_link`。这只是链路验证，不是精度评估；精度以
`research/tio_former/report/` 中的仿真测试为准。

## 6. 限制

- 模型只在 4 向 VL53L5CX 仿真数据上训练，没有实机 ToF 噪声、反射率、
  多径和安装误差验证。
- 训练数据使用前/左/右/下四向布局；当前实机是后/左/右/下，没有前向
  ToF，`sensor_order=[1,2,3,0]` 只是把后向数据填入 front 槽位，不能替代
  真正的前向视角。用于精度验证前应加装前向传感器，或重新训练
  rear/left/right/down 模型。
- `warmup_frames=64` 意味着约 4.3 s 后才输出第一个有效位姿；这是保证输入
  分布与训练一致的必要代价。
- 需要 libtorch；RK3566/Buildroot 上尚未做交叉编译和性能评估，不能把开发机
  latency 当作机载指标。
- 没有回环检测和全局重定位，长距离漂移仍会累积。
- TorchScript 模型由 `checkpoints/main_imu/seed0/best.pt` 导出；重新训练后需
  重新运行 `scripts/export_tio.py` 并重新构建 ROS 包。
