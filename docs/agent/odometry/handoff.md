# 里程计开发交接

最后更新：2026-11-08  
分支：`dev/odom-tio`  
基线提交：`4593a72 feat(sensors): configure real ToF 8x8@15Hz and direction mapping`  
（本文档随之后的清理/交接提交一起更新，最新提交以 `git log -1` 为准。）

本文档给准备接手里程计/ToF 开发的同事使用：先说明“现在到底有什么、验证到
什么程度”，再给出代码地图、接口契约、测试命令和下一步优先级。

## 1. 当前结论

| 部分 | 状态 | 说明 |
|---|---|---|
| 实机 4×VL53L5CX 8×8@15 Hz | **已验证** | 设备端 10 s 实测 14.8–15.2 Hz，0 读取错误 |
| 实机 IMU + ToF 转发/ROS 接收 | **已验证** | 4 路 ToF、200 Hz IMU 可稳定进入 ROS 2 |
| TIO-Former C++/ROS 推理链路 | **已验证能跑** | 进入 `TRACKING`，发布 odometry/path/status/TF |
| TIO-Former 实机静态零漂 | **未通过（旧布局）** | 2026-11-08 旧映射下静止放桌面 20 s 漂移约 0.98 m |
| TIO-Former v3.1 仿真 | ATE 0.0879 ± 0.0025 m | 3 seeds，141 条 test，四个 Gazebo 场景 |
| TIO-Former v3.1 实机部署 | **模型已加载并出首帧位姿** | 2026-11-09，seed1 TorchScript；桌面漂移仍需净空复测 |
| 单目黑白相机转发 | **已验证** | `/dev/video10`，640x480 MJPEG，ROS 发布 mono8，主机约 30 Hz |
| 旧立体 `flyrat::odometry` 后端 | **legacy 骨架** | 只输出 `NOT_INITIALIZED`，无有效位姿 |
| 机载 RK3566 部署 | **未验证** | 目前只在开发机用 CPU libtorch 跑通 |

**结论：传感器链路可以继续开发；当前 TIO-Former 模型不能直接用于飞行闭环。**
2026-11-08 的旧测试使用后/左/右/下布局，且下视传感器距桌面只有 0–5 mm。
机头重定义后实机已是前/左/右/下，映射修正为 `[1,3,2,0]`，但还没有用新布局
做净空静止或飞行验证。

## 2. 代码地图

### 2.1 实机传感器

| 路径 | 作用 |
|---|---|
| `libflyrat/include/flyrat/hardware/tof.hpp` | 4×VL53L5CX 配置、GPIO、`TofFrame`/`TofZone` |
| `libflyrat/src/hardware/tof.cpp` | I2C 初始化、地址恢复、8×8/4×4 采集线程 |
| `flyrat_pilot/src/sensors/sensor_forwarder*.{hpp,cpp}` | 设备端 IMU/ToF/单目 TCP 转发；ToF 默认 8×8@15 Hz |
| `libflyrat/include/flyrat/hardware/mono_jpeg_source.hpp` | `/dev/video10` MJPEG 单目采集 |
| `flyrat_play/ros2_ws/src/flyrat_sensors/` | 主机端 TCP 接收、协议解析、ROS 发布 |
| `flyrat_play/ros2_ws/src/flyrat_sensors_interfaces/` | `TofFrame`/`TofZone` 消息 |

### 2.2 TIO-Former 后端

| 路径 | 作用 |
|---|---|
| `research/tio_former/` | 数据生成、PyTorch 模型、训练/评估、研究报告 |
| `research/tio_former/scripts/export_tio.py` | checkpoint → TorchScript |
| `libflyrat/models/tio_former_window64.pt` | 导出的固定 64 帧模型（4.6 MB） |
| `libflyrat/include/flyrat/tio/tio_odometry.hpp` | `flyrat::tio::TioOdometry` 接口 |
| `libflyrat/src/tio/tio_odometry.cpp` | 滑动窗口、IMU 插值、libtorch 推理、增量积分 |
| `libflyrat/tools/tio_bench.cpp` | 合成输入推理基准 |
| `libflyrat/tests/test_tio_api.cpp` | 接口、warm-up、reset、推理冒烟 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/src/tio_odometry_node.cpp` | ROS 2 节点 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/config/tio_odometry.yaml` | 节点参数和实机方向映射 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/launch/tio_odometry.launch.py` | 单节点启动 |
| `docs/agent/odometry/tio_former.md` | 详细接入、构建、话题和限制 |

### 2.3 旧立体骨架（legacy）

| 路径 | 作用 |
|---|---|
| `libflyrat/include/flyrat/odometry/odometry.hpp` | 旧立体/立体惯性接口 |
| `libflyrat/src/odometry/odometry.cpp` | 桩实现，`TODO(odometry)` 标出接入点 |
| `flyrat_play/ros2_ws/src/flyrat_odometry/src/odometry_node.cpp` | 旧立体 ROS 节点、同步、TF、状态 |
| `libflyrat/configs/camera/uvc_640x480.yaml` | 实机 Kalibr 标定（不要随意改） |

## 3. 接口契约

### 3.1 TIO-Former 模型

固定输入：

```text
tof              [1, 64, 4, 8, 8]  float32，单位 m
imu              [1, 64, 16, 6]    float32，加速度 + 角速度
weight           [1, 64, 4, 8, 8]  float32，0..1
previous_tof     同 tof，前一帧
previous_weight  同 weight，前一帧
```

固定输出：

```text
delta_p    [1, 64, 3]  机体坐标系位移增量
delta_phi  [1, 64, 3]  机体坐标系旋转增量（轴角）
```

C++ 后端只取第 64 帧的增量，并按：

```text
p_t = p_{t-1} + R_{t-1} * delta_p
R_t = R_{t-1} * exp(delta_phi)
```

积分。`Result::pose` 是 **`T_world_body`**，不是旧立体接口的
`T_world_camera`。

视角顺序固定为 **front / left / right / down**。当前机头在原来后向 ToF
的一端，实机 `sensor_order` 为 `[1, 3, 2, 0]`：front←1、left←3、right←2、
down←0。

### 3.2 ROS 话题与 TF

`tio_odometry_node` 与旧立体节点使用同一套接口：

```text
/flyrat/sensors/tof_0 .. tof_3     flyrat_sensors_interfaces/TofFrame
/flyrat/sensors/imu                sensor_msgs/Imu
/flyrat/odometry/odometry          nav_msgs/Odometry，odometry_map -> imu_link
/flyrat/odometry/path              nav_msgs/Path，transient local
/flyrat/odometry/status            diagnostic_msgs/DiagnosticStatus，1 Hz
/flyrat/odometry/reset             std_srvs/Trigger
/tf                                odometry_map -> imu_link
```

状态消息包含 `state`、输入/处理/位姿发布/丢弃/失败帧数、IMU 样本数、
`imu_age_s`、`tof_age_s`、`tof_rate_hz` 和模型路径。状态 `name` 为
`flyrat_tio_odometry`，`hardware_id` 为 `tio_former`。

### 3.3 实机 ToF 方向

RK3566 Linux GPIO = `bank*32 + group*8 + pin`（A=0，B=1，C=2，D=3）。

| Sensor ID | 方向 | INT | LPn | Linux GPIO |
|---:|---|---|---|---|
| 0 | 下 | 3B3 | 3B4 | 107 / 108 |
| 1 | 前 | 3B0 | 3C2 | 104 / 114 |
| 2 | 右 | 3A4 | 3A5 | 100 / 101 |
| 3 | 左 | 3A1 | 3A2 | 97 / 98 |

映射来自用户提供的 INT/LPn 接线，尚未通过逐个遮挡传感器做物理复核。

## 4. 构建、运行、测试

### 4.1 默认构建（不依赖 libtorch）

```sh
.script/build_libflyrat.sh --clean
.script/build_flyrat_pilot.sh --clean
ctest --test-dir libflyrat/build --output-on-failure
```

当前结果：9 个 C++ 测试全部通过。

### 4.2 TIO 构建

```sh
cd research/tio_former
python3 -m venv .venv-cpu
.venv-cpu/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
cd ../..

TORCH_CMAKE="$PWD/research/tio_former/.venv-cpu/lib/python3.12/site-packages/torch/share/cmake"

cmake -S libflyrat -B libflyrat/build-tio \
  -DFLYRAT_BUILD_TIO=ON -DFLYRAT_BUILD_TESTS=ON -DFLYRAT_BUILD_TOOLS=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$TORCH_CMAKE"
cmake --build libflyrat/build-tio -j"$(nproc)"
ctest --test-dir libflyrat/build-tio --output-on-failure -R tio
```

`tio_bench` 只验证模型加载/推理链路，不代表精度。

### 4.3 ROS 2 TIO 构建

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

### 4.4 实机一键启动

```sh
.script/bringup_real.sh start_foxglove:=true odometry_backend:=tio
```

注意：

- 参数名是 `start_odometry`，不是旧的 `start_slam`；`start_odometry` 默认
  为 `true`，通常不用写。
- 默认后端已经是 `tio`；`odometry_backend:=stereo` 仍可选择旧立体骨架。
- 单目相机默认 `/dev/video10`、640x480@60；可用 `camera_device`、
  `camera_width`、`camera_height`、`camera_fps` 覆盖。
- `start_foxglove:=true` 启动 Foxglove Bridge，默认 `ws://<host>:8765`。

第一次有效位姿需要 64 帧，约 4.3 s @15 Hz。

### 4.5 无 Gazebo 回放

```sh
cd research/tio_former
.venv/bin/python scripts/publish_processed_ros.py \
  data/processed/test_corridor_0000.npz --rate 15 --sensor-order 1 3 2 0
```

另开终端启动节点后检查：

```sh
ros2 topic echo /flyrat/odometry/status --once
ros2 topic echo /flyrat/odometry/odometry --once
ros2 run tf2_ros tf2_echo odometry_map imu_link
```

### 4.6 Python 研究测试

```sh
cd research/tio_former
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

当前 50 项通过。

## 5. 真机验证记录（2026-11-08）

设备：RK3566，四颗 VL53L5CX（0x60/0x62/0x64/0x66），设备端 `tof_bench`
10 s 结果：

| Sensor | 帧数 | 实测频率 | 读取错误 | 平均读取 |
|---:|---:|---:|---:|---:|
| 0（下） | 151 | 15.1 Hz | 0 | 4.66 ms |
| 1（旧后，新前） | 152 | 15.2 Hz | 0 | 4.68 ms |
| 2（旧左，新右） | 150 | 15.0 Hz | 0 | 4.69 ms |
| 3（旧右，新左） | 148 | 14.8 Hz | 0 | 4.68 ms |

单目相机：`/dev/video10`，ROS 图像 `640x480`、`mono8`，主机实测约 30 Hz。

新机头布局（`sensor_order=[1,3,2,0]`）下 TIO 节点实时状态（约 10 s）：

```text
state: TRACKING
input_frames: 174
processed_frames: 172
pose_frames: 109
dropped_frames: 2
failed_frames: 0
imu_samples: 2264
```

旧映射下静止放桌面 20 s：末端约 `(0.9806, -0.0694, 0.0278) m`，最大位移
约 `0.9835 m`。**这是旧布局的负结果，不能代表新映射。**

原因按优先级：

1. 下视传感器距桌面 0–5 mm，远超训练分布；
2. 当时映射为 `[1,2,3,0]`，模型 front 槽位喂的是后向数据；
3. 模型只在 Gazebo 数据上训练，sim-to-real 差异明显；
4. 真机 IMU 偏置/振动/安装误差与仿真不同。

## 6. 下一步优先级

### P0：先得到可信的实机基线

1. 把无人机抬到离地约 1 m、四周无近距遮挡处，静止 30–60 s 重测零漂；
2. 逐个遮挡/靠近传感器，观察 `/flyrat/sensors/tof_N`，确认 0/1/2/3 与
   下/前/右/左的物理对应；
3. 记录下视在空中的有效距离和 status 分布；
4. 保存一份真实 ToF+IMU+GT（或至少静止/已知运动）数据，作为后续微调集。

### P1：验证新布局并做域适配

1. 用前/左/右/下新布局在净空静止和已知运动下重测，区分映射错误与域差异；
2. 记录真实 ToF 噪声、失效比例、下视地面距离和 IMU 偏置；
3. 用实机数据做微调或重训，再重新导出模型；
4. 重新导出后必须重跑 `test_tio_api`、ROS 回放和实机链路测试；
5. 详情和现有 v2 负结果见 `report/model_v2_results.md`。

### P2：工程化

1. 优化 C++ 推理：当前每帧重算 64 帧窗口，开发机 CPU 约 9 ms；可改为增量
   streaming cache 或降低重算范围；
2. 评估 libtorch 在 RK3566/Buildroot 上的交叉编译、内存和延迟；
3. 统一 ToF/IMU 时间基准，记录真实时间偏移；
4. 把里程计接入 `flyrat_pilot` 时增加失效保护：状态非 `TRACKING`、IMU 超时、
   ToF 丢帧过多时退出闭环。

### P3：旧立体骨架（legacy）

旧立体后端仍是桩实现，和 TIO 是两条独立路线。若继续立体路线，按
`odometry/overview.md` 的接入点实现，不要破坏现有 ROS 契约。

## 7. 调试清单

```sh
# 设备是否在线
adb devices

# 设备端 ToF 频率（先停 forwarder，避免 I2C 争用）
# 需要交叉编译的 tof_bench；参数见 docs/agent/hardware/sensor_forwarding.md

# ROS 输入
ros2 topic hz /flyrat/sensors/imu
ros2 topic hz /flyrat/sensors/tof_0
ros2 topic echo --once /flyrat/sensors/tof_0

# TIO 状态
ros2 topic echo /flyrat/odometry/status --once
ros2 topic echo /flyrat/odometry/odometry --once
ros2 run tf2_ros tf2_echo odometry_map imu_link
```

常见现象：

- `NOT_INITIALIZED` 持续：没有收齐 4 路 ToF 或 IMU；看 status 的
  `input_frames`/`imu_samples`。
- `dropped_frames` 增长快：4×4@60 输入被限频，或 ApproximateTime 同步失败。
- `failed_frames` 增长：模型推理异常或输入形状不对。
- 状态 `TRACKING` 但位置漂移：先检查下视距离、前/右/左映射和 IMU。

## 8. 不要做的事

- 不要把 `warmup_frames` 降到很小来“加快出位姿”；窗口前部会零填充，模型
  输入分布不匹配。
- 不要把模型四元数顺序改成 `qx qy qz qw` 以外的格式；trajectory 输出是 TUM
  顺序。
- 不要修改 `libflyrat/configs/camera/uvc_640x480.yaml` 的数值来“试漂移”；
  它属于旧的立体路线，当前单目相机没有可用标定。
- 不要沿用旧映射 `[1,2,3,0]`；机头重定义后正确值是 `[1,3,2,0]`。
- 不要修改 `docs/drone/` 或 `playground/`。
