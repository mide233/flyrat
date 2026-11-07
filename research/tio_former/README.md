# TIO-Former 仿真复现

Clean-room reproduction of **TIO-Former** (arXiv:2609.17198) for FlyRat.
The method uses 4× VL53L5CX 8×8 ToF arrays (front / left / right / down) and a
200 Hz IMU to estimate 6-DoF motion.

完整计划见 [`docs/agent/research/tio_former.md`](../../docs/agent/research/tio_former.md)。

## 状态

- **M0 完成**：目录、配置、SE(3)/ToF/轨迹/IMU 工具、数据容器，24 项测试通过。
- **M1 完成**：四个 primitive 场景、4 向 ToF rig、Gazebo 轨迹播放插件、
  单序列 headless 录制与 8×8 后处理。已跑通 `corridor_demo`：182 帧 /
  12.07 s / 1.80 m，4×8×8 @ 15 Hz，GT 对齐，有效率约 89%。
- M2–M5（全量数据、模型、训练、报告）尚未开始。

## 依赖

基础（当前开发镜像已具备）：

```text
python3 >= 3.10, numpy, PyYAML, pytest
Gazebo Harmonic 8.x (gz sim, gz-transport13 python bindings)
```

训练与完整数据（多卡集群安装）：

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
pip install -r requirements-train.txt
```

Gazebo 环境如果被 ROS vendor 覆盖，使用系统工具链：

```bash
export PATH=/usr/bin:$PATH
unset GZ_CONFIG_PATH
```

## 快速开始（M1 单序列）

以下命令均在 `research/tio_former/` 下执行。

构建 Gazebo 轨迹插件：

```bash
bash scripts/build_gz_plugin.sh
```

生成四个场景和示例轨迹：

```bash
python3 scripts/generate_worlds.py
python3 scripts/generate_trajectories.py \
  --kind demo --scene corridor --duration 12 --name corridor_demo
```

录制 12 秒走廊序列（headless Gazebo）：

```bash
python3 scripts/record_sequence.py \
  --world sim/worlds/generated/corridor.sdf \
  --trajectory data/trajectories/corridor_demo.csv \
  --out data/raw/corridor_demo.npz \
  --duration 12
```

后处理为 8×8 ToF、GT 和解析 IMU：

```bash
python3 scripts/postprocess_sequence.py \
  --raw data/raw/corridor_demo.npz \
  --out data/processed/corridor_demo.npz \
  --scene corridor --kind demo
```

检查：

```bash
python3 scripts/inspect_sequence.py data/processed/corridor_demo.npz
```

## 测试

```bash
cd research/tio_former
python3 -m pytest -q
```

## 目录

```text
configs/     传感器、场景、数据、模型和训练配置
sim/         世界生成器输出、ToF rig、Gazebo 插件
scripts/     世界/轨迹生成、录制、后处理、训练和评估入口
tio_former/  Python 包：SE(3)、轨迹、ToF、IMU、数据和模型
tests/       单元测试和资产检查
report/      中文技术报告（后续）
```

## 约定

- 世界坐标：ENU / Gazebo world，z 向上。
- 机体坐标：FLU（+x 前，+y 左，+z 上）。
- `T_a_b` 表示把 b 坐标系中的点变换到 a 坐标系：`p_a = T_a_b * p_b`。
- ToF 数据单位米；无效值为 0，有效性由 `tof_status` 表示。
- 不提交 `data/`、`assets/`、`runs/`、`checkpoints/`。
