# TIO-Former 仿真复现

Clean-room reproduction of **TIO-Former** (arXiv:2609.17198) for FlyRat.
The method uses 4× VL53L5CX 8×8 ToF arrays (front / left / right / down) and a
200 Hz IMU to estimate 6-DoF motion.

完整计划见 [`docs/agent/research/tio_former.md`](../../docs/agent/research/tio_former.md)。

## 状态

- **M0 完成**：目录、配置、SE(3)/ToF/轨迹/IMU 工具、数据容器，26 项测试通过。
- **M1 完成**：四个场景、4 向 ToF rig、Gazebo 轨迹播放插件、
  单序列 headless 录制与 8×8 后处理。已跑通 `corridor_demo`：182 帧 /
  12.07 s / 1.80 m，4×8×8 @ 15 Hz，GT 对齐，有效率约 89%。
- **M2 完成**：Gazebo Fuel 家具 mesh + 程序化几何的四个场景、障碍物感知轨迹、
  全量数据生成与索引。实际 **324 序列 / 9,728.9 s / 2,081.6 m /
  146,265 帧**；四场景有效 zone 比例 75.2%–97.0%（总体 85.7%）；
  随机划分 train/val/test = 165/56/103 序列。
- **M3 完成**：PyTorch 模型（ToF 编码、4 视角 Transformer、IMU 引导
  cross-attention、Local KV + Chunk-FIFO 流式 Transformer、位姿头）、论文全部
  损失项、流式/全序列因果一致性测试、单序列过拟合。模型 1,129,542 参数；
  过拟合 300 步损失 0.2658 → 0.0045（−98.3%）。
- **M4 完成**：主训练、测试集评估、核心消融、鲁棒性和 PyTorch 实时性测试完成。
  主模型 **103 条测试序列 ATE 0.4316 m / RPEp 0.0186 m /
  RPER 0.257° / EDR 9.49%**（仿真 4 向 ToF，不能与论文 6 向数值直接对比）。
  完整模型优于 IMU-only 和 memoryless，但 `naive_concat` 与
  `kv_only_no_fifo` 仍更好，已在 `report/m4_results.md` 中作为负消融结果记录。
  逐帧流式推理 RTX 3060 上均值 1.95 ms、P95 2.31 ms，0/200 次超过 15 Hz 预算。
- **M5 完成**：3 seeds 主结果、LOSO、鲁棒性、实时性、中文技术报告与
  paper-style 表格/图。3 seeds 平均 **ATE 0.4073 ± 0.0218 m /
  RPEp 0.0179 ± 0.0005 m / RPER 0.315 ± 0.061° / EDR 8.98 ± 0.43%**；
  LOSO 平均 ATE **0.4652 m**。完整报告见 `report/m5_report.md`，
  M4 结果表见 `report/m4_results.md`。最佳 seed 的轨迹/精度可视化和
  失效分析见 `report/best_model/analysis.md`。
- **v3.1 数据与训练**：重做速度/步长分布，统一 train/val/test 的 5 个速度 bin
  并按目标速度重定时；共 **894 序列 / 22,927.5 s / 6,229.5 m / 344,829 帧**，
  split **618/135/141**，平均速度 0.315/0.316/0.308 m/s。关闭合成 status=10，
  加入 3% dropout + 2% reflectivity dropout，增加 0.6–2.2 m 垂直运动。
  审计除 val–test KS=0.173 外均通过。旧 checkpoint 选择规则曾误选 epoch 7
  （ATE 0.5184 m）；修复为只惩罚 `[0.83,1.2]` 之外的尺度误差后，3 seeds
  取得测试 **ATE 0.0879 ± 0.0025 m / RPEp 0.00363 m / RPER 0.175° /
  EDR 2.52%**，路径比约 1.00，三个 seed 的 v3 gate 全部通过。
  结果与图见 `report/v3_model/analysis.md`。
- **P1 前向单目相机**：仿真 `tof_rig` 增加 `320×240@15` L8 相机；数据集记录
  `camera_jpeg/camera_offset`（160×120 JPEG q85）；模型增加 `CameraEncoder`
  和 IMU>camera>ToF 的软先验可靠性 gate（`--camera` 开启）。完整数据集
  891 序列 / 23,361 s，3 seeds 测试 ATE 0.0868 ± 0.0047 m；相机置零
  0.0945 m，全部 ToF 丢弃时 0.174 m（无相机模型 0.205 m）。
  `libflyrat/models/tio_former_window64_camera.pt` 已导出并成为
  `bringup_real.sh` 的默认 TIO 模型，`tio_camera:=false` 可回退。
  真实相机仍未标定，实机精度尚未复测。
- **模型改进筛选（v2）**：新增 `hybrid`、`gated_fifo`、`hybrid_gated`、
  `hybrid_split`。15-epoch 筛选中 `hybrid_gated` ATE 0.4208 m；40-epoch 复核
  0.4122 m，优于原 full 的 0.4316 m，但仍未超过 `naive_concat`（0.4013 m）和
  `kv_only_no_fifo`（0.3925 m），RPER 0.757° 也更差。位置/旋转双分支
  `hybrid_split` 为 0.4350 m，没有改善。结论为负结果，保留原 full 模型；
  详见 `report/model_v2_results.md`。
- **C++/ROS 接入完成（实验）**：主模型导出为
  `libflyrat/models/tio_former_window64.pt`，新增 `flyrat::tio::TioOdometry`
  和 `tio_odometry_node`，用 libtorch 在 C++ 中推理并发布
  `/flyrat/odometry/*` 与 `odometry_map -> imu_link`。开发机已用仿真序列
  跑通端到端链路；尚未做实机精度验证或 RK3566 部署。详见
  [`docs/agent/odometry/tio_former.md`](../../docs/agent/odometry/tio_former.md)。
  当前 Python 测试 **50 项通过**，C++ 默认构建 9 项、TIO 构建另加
  `test_tio_api`。
- **真机验证（2026-11-08，旧布局）**：4 颗 VL53L5CX 设备端实测 14.8–15.2 Hz，
  0 读取错误；TIO 节点能进入 `TRACKING` 并发布 odometry/TF。但静止放桌面
  20 s 漂移约 **0.98 m**，当前不能用于飞行。主要原因是下视距桌面 0–5 mm、
  旧映射 front 槽位接后向和仿真域差异。机头重定义后新映射为
  `[1,3,2,0]`，需在净空条件下复测。交接记录见
  [`docs/agent/odometry/handoff.md`](../../docs/agent/odometry/handoff.md)。

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

## 全量数据生成（M2）

Gazebo Fuel 家具 mesh 是可选的：不下载时世界仍使用程序化几何，轨迹与录制流程不变。
下载后把 `assets/models/` 加入 Gazebo 资源路径（录制脚本已自动加入）。

```bash
# 下载 8 个 CC-BY/CC0 家具模型到 assets/models/，并写入 assets/LICENSES.md
bash scripts/download_assets.sh

# 重新生成包含 mesh 的四个世界
python3 scripts/generate_worlds.py

# 试跑 4 个短序列，检查录制/后处理链路
python3 scripts/generate_dataset.py --smoke --overwrite

# 生成完整数据集（目标时长由 generation 配置的 target_seconds 决定；可中断续跑）
# workers: 0 表示按 CPU 自动选择（上限 6；再高会因并发 EGL 上下文失败）
python3 scripts/generate_dataset.py

# 指定并发（本机 12 核实测：6 workers 约 12x 实时，3 workers 约 5.5x）
python3 scripts/generate_dataset.py --workers 6

# 建立索引、随机划分与留一场景划分
python3 scripts/build_dataset_index.py
```

### 前向单目相机（P1）

`sim/models/tof_rig/model.sdf` 现在包含一个前向 `320×240 @ 15 Hz` 单目相机
（`/tio/camera_front`，L8）。录制时按 15 Hz 与 ToF/GT 对齐，后处理把每帧
缩放到 `160×120` 并以 JPEG q85 存入 NPZ：

- `camera_jpeg`：所有帧的 JPEG 字节拼接（uint8）
- `camera_offset`：`T+1` 个偏移，用于切片
- `camera_frame_valid`：该 ToF 帧是否有对齐的相机帧

这些字段是可选的，不进入 `REQUIRED_ARRAYS`，旧数据集仍可直接加载。

实机 rosbag 可离线检查与回放：

```bash
# 解包 mcap -> bag.npz/bag.json（需要 ROS jazzy 的 rosbag2_py 和 cv2）
bash -lc 'source /opt/ros/jazzy/setup.bash && python3 scripts/inspect_rosbag.py \
  --bag ../../issue_6_logbag_1_1 --out /tmp/bag_issue6'

# 用指定 checkpoint 回放并输出轨迹
.venv/bin/python scripts/replay_rosbag.py --extracted /tmp/bag_issue6/bag.npz \
  --checkpoint checkpoints/seed0.pt --out runs/bag_issue6 --plot --device cpu
```
训练时加 `--camera` 会启用 `CameraEncoder` 和可靠性 gate：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/train.py \
  --processed data_v3/processed --out checkpoints_cam --run-name seed0 \
  --camera --epochs 60 --workers 8
```

gate 使用 IMU > camera > ToF 的软先验，但权重可学习，相机可以在静态漂移时
压过 IMU/ToF。训练日志每个 epoch 会输出 `gate imu/camera/tof` 平均权重，
`metrics.jsonl` 中同时记录 `train_gate`/`val_gate`；`gate_regularization`
中的 `tof_floor` 通过 `relu(tof_floor - w_tof)` 防止 ToF 分支塌缩。

ToF 大面积缺失（视觉可用）的训练样本由 `augmentation.tof_degraded_*`
控制；评估时可用：

```bash
# 正常输入
.venv/bin/python scripts/evaluate.py --checkpoint checkpoints_cam/seed0/best.pt \
  --processed data_cam_full/processed --split test --out runs_cam/eval
# 退化 ToF：丢 2 路视角并随机丢弃 60% 有效区
.venv/bin/python scripts/evaluate.py --checkpoint checkpoints_cam/seed0/best.pt \
  --processed data_cam_full/processed --split test \
  --drop-views front left --status-dropout 0.6 --out runs_cam/eval_degraded
```

`summary.json` 额外写入 `gate_usage`，便于比较有/无相机时的分支使用率。

完整相机数据集配置为 `configs/data/generation_camera_full.yaml`
（目标 20,000 s，输出到 `data_cam_full/`）。

### 集群训练

`scripts/cluster/` 提供两个 Slurm 脚本，默认布局为
`$HOME/flyrat_tio/p1_camera/`（代码、`data_cam_full/processed`、
`checkpoints_cam_full`、`logs`），可用 `FLYRAT_P1_ROOT` 覆盖：

```bash
# 3 个相机 seed + 1 个无相机基线
sbatch scripts/cluster/train_seed.sbatch 0 seed0 1
sbatch scripts/cluster/train_seed.sbatch 1 seed1 1
sbatch scripts/cluster/train_seed.sbatch 2 seed2 1
sbatch scripts/cluster/train_seed.sbatch 0 seed0_nocam 0

# 每个 run 评估：正常 test、退化 ToF、v3.1 test（相机补零）
sbatch scripts/cluster/eval_run.sbatch seed0
```

脚本假定 Python 环境已装 torch/Pillow（例如 micromamba `fdrone`），
`--processed` 必须是绝对路径；否则 `train.py` 会相对项目根解析。

结果写入 `data/processed/`、`data/index.csv`、`data/splits.json`、
`data/stats.json`、`data/report.md`。数据目录不提交。

数据集容器当前为 NPZ；HDF5 因训练环境未安装 h5py 而推迟到后续阶段。

## 模型与过拟合检查（M3）

```bash
# 训练依赖（多卡环境按需安装）
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt -r requirements-train.txt

# 单序列过拟合，验证模型/损失/数据管线
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/overfit_sequence.py \
  --sequence data/processed/train_corridor_0000.npz --steps 300
```

检查点写入 `checkpoints/`（不提交）。M3 实测：1,129,542 参数；300 步损失
0.2658 → 0.0045（−98.3%）。

## 训练与评估（M4）

```bash
# 若修改了 IMU 噪声/零偏配置，从已处理序列重建 IMU 窗口
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/regenerate_imu.py

# 主模型训练（单卡或单进程；40 epochs）
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/train.py \
  --workers 8 --epochs 40 --out checkpoints/main_imu --run-name seed0

# 测试集评估
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/evaluate.py \
  --checkpoint checkpoints/main_imu/seed0/best.pt --split test \
  --out runs/eval_main_imu

# 核心消融
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/run_ablations.py \
  --variants full imu_only tof_only naive_concat memoryless kv_only_no_fifo \
  --epochs 15 --workers 8 --main-checkpoint checkpoints/main_imu/seed0/best.pt \
  --checkpoint-root checkpoints/ablations_imu --eval-root runs/ablations_imu

# 鲁棒性
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/run_robustness.py \
  --checkpoint checkpoints/main_imu/seed0/best.pt --limit 0 --out runs/robustness_imu

# 逐帧实时性
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/benchmark_latency.py \
  --checkpoint checkpoints/main_imu/seed0/best.pt --out runs/latency_imu.json

# 汇总报告
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/make_report.py \
  --main runs/eval_main_imu/summary.json \
  --ablations runs/ablations_imu/summary.json \
  --train-metrics checkpoints/main_imu/seed0/metrics.jsonl \
  --robustness runs/robustness_imu/summary.json --out report
```

结果见 `report/m4_results.md`。主模型 103 条测试序列 ATE 0.4316 m；
完整模型优于 IMU-only，但 `naive_concat`/`kv_only_no_fifo` 更优，
该负消融结果已如实记录。训练数据、检查点和运行输出均不提交。

## 3 seeds 与 LOSO（M5）

```bash
# 3 seeds 聚合
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/aggregate_seeds.py \
  --runs runs/eval_main_imu runs/eval_main_imu_seed1 runs/eval_main_imu_seed2

# 留一场景
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/run_loso.py --epochs 15
```

完整中文技术报告见 `report/m5_report.md`。

最佳 seed 的轨迹与失效分析：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/analyze_best_model.py \
  --eval runs/eval_main_imu_seed1 \
  --split-eval train=runs/eval_best_train \
  --split-eval val=runs/eval_best_val \
  --out report/best_model
```

结果见 `report/best_model/analysis.md`，重点是速度相关的平移尺度塌缩。

## C++ / ROS 2 接入（实验）

导出 TorchScript 模型（默认读取 `checkpoints/main_imu/seed0/best.pt`）：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/export_tio.py \
  --checkpoint checkpoints/main_imu/seed0/best.pt \
  --out ../libflyrat/models/tio_former_window64.pt
```

C++ 后端需要 CPU libtorch：

```bash
python3 -m venv .venv-cpu
.venv-cpu/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
TORCH_CMAKE="$PWD/.venv-cpu/lib/python3.12/site-packages/torch/share/cmake"
cd ..
cmake -S libflyrat -B libflyrat/build-tio \
  -DFLYRAT_BUILD_TIO=ON -DFLYRAT_BUILD_TESTS=ON -DFLYRAT_BUILD_TOOLS=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$TORCH_CMAKE"
cmake --build libflyrat/build-tio -j"$(nproc)"
ctest --test-dir libflyrat/build-tio --output-on-failure -R tio
```

ROS 包默认不编译 TIO 节点，需要 `-DFLYRAT_BUILD_TIO=ON`。节点、话题、回放和
实机限制见 [`docs/agent/odometry/tio_former.md`](../../docs/agent/odometry/tio_former.md)。

## 测试

```bash
cd research/tio_former
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 用于避开 ROS Jazzy 注入的 pytest11 插件。

## 目录

```text
configs/     传感器、场景、数据、模型和训练配置
sim/         世界生成器输出、ToF rig、Gazebo 插件
scripts/     世界/轨迹生成、录制、后处理、训练和评估入口
tio_former/  Python 包：SE(3)、轨迹、ToF、IMU、数据和模型
tests/       单元测试和资产检查
report/      M4/M5 结果表、v2 负结果与技术报告
```

## 约定

- 世界坐标：ENU / Gazebo world，z 向上。
- 机体坐标：FLU（+x 前，+y 左，+z 上）。
- `T_a_b` 表示把 b 坐标系中的点变换到 a 坐标系：`p_a = T_a_b * p_b`。
- ToF 数据单位米；无效值为 0，有效性由 `tof_status` 表示。
- 不提交 `data/`、`assets/`、`runs/`、`checkpoints/`。
