# TIO-Former 仿真复现计划

- 论文：TIO-Former: Ultra-Lightweight 6-Directional ToF-Inertial Odometry for
  Nano-UAVs via a Streaming Causal Transformer（arXiv:2609.17198）
- 代码目录：`research/tio_former/`
- 状态：M0–M5 完成
- 最后更新：2026-11-08

## 1. 目标与边界

### 1.1 目标

在仿真中 clean-room 复现 TIO-Former 的核心方法，验证 4 向 ToF + 200 Hz IMU
的 6-DoF 里程计性能趋势、消融关系和实时性。

### 1.2 不做的事情

- 不追求复现论文 6 向 TOFSense-M 的绝对数值。
- 不实现 TLIO、AirIO、Crazyflie Flow 作为基线。
- 不在本阶段做边缘部署；先完成 PyTorch 原型，再把 TorchScript 模型接到
  `libflyrat`/ROS 2 做开发机验证。RK3566 交叉编译与机载性能评估仍后置。
- 不改动飞控代码；研究代码放在 `research/tio_former/`，接入只新增
  `flyrat::tio` 和 `tio_odometry_node`；旧立体骨架保留为 legacy。

### 1.3 与论文的关键差异

| 项目 | 论文 | 本复现 |
|---|---|---|
| ToF 数量与布局 | 6 向正交 TOFSense-M | 4 向 VL53L5CX：前/左/右/下 |
| ToF 分辨率/频率 | 8×8 @ 15 Hz | 8×8 @ 15 Hz（高分辨率渲染后聚合） |
| IMU | 200 Hz | 200 Hz，仿真解析生成 |
| 数据来源 | 实飞 + 动捕 | Gazebo 场景 ray-cast + 完美 GT |
| 平台 | Crazyflie + SG2002 | 离线 Python 原型 |
| 主表对比 | 6 向结果 | 仅作趋势参考，标注布局差异 |

论文 Table III 中“four lateral views”为 0.129 m，可作为四向的近似参照，但本复现
包含下视、没有后视，不能直接等同。

## 2. 研究目录

```text
research/tio_former/
├── README.md
├── pyproject.toml
├── requirements.txt
├── configs/
│   ├── sim/{sensors,scenes,trajectory}.yaml
│   ├── data/{dataset,generation}.yaml
│   ├── model/base.yaml
│   ├── train/default.yaml
│   └── eval/{main,ablation,robustness}.yaml
├── sim/
│   ├── worlds/                 # 由 generate_worlds.py 生成（gitignore）
│   ├── models/tof_rig/         # 4 向 ToF 机体
│   └── gz_plugins/trajectory_player/
├── scripts/
│   ├── generate_worlds.py
│   ├── generate_trajectories.py
│   ├── generate_dataset.py     # 批量轨迹 + 录制 + 后处理
│   ├── build_dataset_index.py  # index/splits/stats/report
│   ├── record_sequence.py
│   ├── postprocess_sequence.py
│   ├── inspect_sequence.py
│   ├── build_gz_plugin.sh
│   ├── download_assets.sh
│   ├── export_tio.py           # checkpoint -> TorchScript
│   └── publish_processed_ros.py # 仿真序列 -> ROS 话题回放
├── tio_former/
│   ├── data/{dataset,imu,torch_dataset}.py
│   ├── models/{tio_former,geometry,losses}.py
│   ├── sim/{scene,trajectory,tof}.py
│   └── utils/{config,se3,logging,seed}.py
├── tests/
└── report/
```

- `configs/model`、`configs/train`、`configs/eval` 已用于 M3–M5 的模型、
  训练、主结果、消融和鲁棒性实验。
- 代码、配置、脚本、世界生成器、测试和报告纳入 git。
- `data/`、`assets/`、`runs/`、`checkpoints/` 不提交。
- 真实 mesh 从 Gazebo Fuel 下载，记录来源和 license，资产不提交。

## 3. 仿真与数据生成

### 3.1 场景

四个场景对应论文的典型环境：

| 场景 | 构造 | 目的 |
|---|---|---|
| `corridor` | 30×2.4×2.6 m 长走廊，重复墙面与立柱 | 纵向退化 |
| `office` | 10×8×3 m，隔断、桌椅、柜子 | 密集非凸遮挡 |
| `pantry` | 6×5×3 m，台面、桌子、瓷砖材质 | 反射/失效 |
| `meeting_room` | 8×6×3 m，长桌、椅子、屏幕 | 家具遮挡 |

场景采用程序化几何 + Gazebo Fuel 家具 mesh。M2 实际接入 8 个模型：
`Desk`、`OfficeChairGrey`、`Cabinet`、`Fridge`、`Table`、`SquareShelf`、
`MonitorAndKeyboard`、`Sofa`，均为 CC-BY/CC0，由 `download_assets.sh` 下载到
本地 `assets/models/`（不提交），并写入 `assets/LICENSES.md`。Replica 作为
可选的真实场景来源保留在脚本说明中，本阶段未下载、未使用。

### 3.2 机体与传感器

- 机体坐标系：FLU（+x 前，+y 左，+z 上）。
- 4 个 ToF 朝向：`+x`、`+y`、`-y`、`-z`。
- 每个 ToF：
  - 视场 45°×45°；
  - 量程 0.015–4.0 m；
  - 输出 8×8，15 Hz。
- Gazebo 侧每向渲染 64×64 depth，再聚合到 8×8，避免低分辨率渲染限制。
- 每个序列同时记录 GT 位姿和传感器时间戳。

### 3.3 轨迹

- 20% canonical：圆、8 字、方形、直线，变化半径、速度、高度。
- 80% random：自由空间随机 waypoint + min-jerk / 三次样条，限制速度
  0.5–2 m/s、加速度 1–3 m/s²，离障碍 ≥0.3 m。
- 训练序列 20–60 s；测试序列 4–7 m。
- 目标规模约 9,500 s / 2,200 m；按场景和序列分片并行生成，可断点续跑。

### 3.4 IMU

从解析轨迹计算：

```text
ω_body = vee(Rᵀ Ṙ)
f_body = Rᵀ (a_world - g_world),  g_world = [0, 0, -9.81]
```

- 200 Hz 采样；
- 白噪声使用配置中的噪声密度；
- 随机游走 bias；
- 可选 scale/misalignment 和初始化 bias；
- 默认无时间偏移，保留 `imu_time_offset_s` 配置，用 3.8 ms 做鲁棒性测试；
- 每个 ToF 间隔内重采样为 K=16；K=6 作为消融，对应论文 `R^{8×6}` 的表述差异。

### 3.5 ToF 噪声与失效

- 基础：AR(1) 噪声，γ=0.9，σ=0.01 m。
- 失效：随机 dropout、量程外、无回波、反射率丢失、空间 patch / 时间 burst。
- 状态映射到 VL53L5CX：
  - `target_status ∈ {5, 9}`：有效，权重 1.0；
  - 新捕获/低置信：模拟论文 `s=10`，权重 0.5，不作为插值 donor；
  - 其他：无效，权重 0；
  - 状态机模拟连续跟踪：无效→有效先给一帧 0.5，再给 1.0。
- 训练时在线施加 `ρ∈{0.1,0.3,0.5}` 缺失扰动；测试集固定生成 10/30/50%
  缺失和不同噪声等级。

### 3.6 数据格式

每个序列一个容器（当前阶段使用 `.npz`；HDF5 因训练环境未安装 h5py 而推迟）：

```text
tof            [T,4,8,8] float32   # 米
tof_status     [T,4,8,8] uint8
tof_ideal      [T,4,8,8] float32   # 可选
gt_pose        [T,4,4] float32     # T_world_body @ 15 Hz
gt_twist       [T,6] float32       # 可选
imu            [T,K,6] float32     # accel, gyro @ 200 Hz
t_tof          [T] float64
t_imu          [T,K] float64
scene_id, seq_id, traj_type, split
```

另生成 `index.csv`、训练集统计、场景/轨迹分布报告。

## 4. 模型

### 4.1 ToF 前端

- 可靠性 mask `A_t`。
- 因果插补：无效像素用空间邻域 + 历史有效值填充，插补像素权重保持 0。
- 双边门控差分：
  `ΔD = A_{t-1} ⊙ A_t ⊙ (D̃_t - D̃_{t-1})`。
- 输入 `X_t = Cat(D̃_{t-1}, D̃_t, ΔD_t) ∈ R^{3×8×8}`。

### 4.2 网络

- 共享 2D CNN：3 个 conv block，通道 32/64/128，全局池化到 `d_f=128`；
  加可学习 view embedding。
- 4-token view Transformer：默认 2 层、4 heads、FFN 256、pre-LN。
- IMU encoder：3 层 1D CNN（6→64→128→128），时间池化到 `d_f`。
- IMU-guided cross-attention：IMU 特征作 query，4 个 view token 作 K/V；
  4 heads，残差 + LayerNorm。
- Streaming Transformer：
  - Local KV cache `C=8`；
  - 每 `G=4` 个被驱逐 token 均值池化 + MLP `φ_m`，压入 Chunk-FIFO `S=4`；
  - 每步最多 attend 12 个 token，严格 causal mask；
  - 默认 2 层 streaming layer。
- Pose head：2 层 MLP → `Δp̂_t`、轴角 `Δφ̂_t`。
- 递归积分：
  - `ΔR̂_t = exp([Δφ̂_t]×)`
  - `R̂_t = R̂_{t-1} ΔR̂_t`
  - `p̂_t = p̂_{t-1} + R̂_{t-1} Δp̂_t`

论文未给出的维度、层数、学习率等全部写入配置，并在报告中说明来源和敏感性。

### 4.3 损失

严格按论文：

- `L_step`：Smooth L1，平移 β=0.05，旋转 β=0.02；
- 多 horizon：`H_p={4,8,16,32}`，`H_R={4,8,16}`；
- `L_scale`：β=0.10；
- `L_res`：β=0.02；
- 总损失：
  `L = L_step + 0.02 L_mh^p + 0.005(L_mh^R + L_scale + L_res)`。

## 5. 训练与评估

### 5.1 训练

- 窗口 64 帧（约 4.3 s），batch 32/GPU，梯度累积 2。
- AdamW，lr 1e-3，weight decay 1e-4，5% warmup + cosine，100 epochs，
  early stopping。
- 3 个种子（0/1/2），DDP + BF16 AMP，梯度裁剪 1.0。
- 先在 1 条序列上过拟合到接近零损失，再全量训练。
- 增强：缺失扰动、AR(1) 噪声、随机 view dropout（1–3 向）。

### 5.2 指标

- ATEp：SE(3) 对齐，不做尺度修正。
- RPEp / RPER：66.7 ms 间隔。
- EDR：终点位移 / GT 轨迹长度。
- 每序列 RMSE，再对测试集平均；3 seeds 报 mean±std。

### 5.3 实验

- 随机 70/15/15：整体 + 每场景表。
- 留一场景（LOSO）：4 个场景分别留出。
- 核心消融：
  1. IMU-only
  2. ToF-only
  3. Naive channel concat
  4. Memoryless
  5. GRU memory
  6. KV-only（无 FIFO）
  7. Full model
- 鲁棒性：缺失 1/2/3 向、噪声 0/0.01/0.02/0.05 m、
  状态 dropout 10/30/50%、反射率丢失。
- 实时性：batch=1 流式推理 p50/p95 latency、参数量、峰值 RSS，
  GPU 与 CPU 各测一次，目标 <66.7 ms/step。

## 6. 阶段与验收

| 阶段 | 内容 | 验收 |
|---|---|---|
| M0 | 目录、环境、配置、测试骨架 | `pytest` 通过，能加载配置 |
| M1 | 4 场景 + rig + 轨迹插件 | 单序列渲染成功，4×8×8@15 Hz，GT 对齐 |
| M2 | 全量数据生成 | ~9500 s / ~2200 m，场景与轨迹分布合理 |
| M3 | 模型 + 单元测试 | 流式 cache 与全因果注意力一致；单序列过拟合 |
| M4 | 主训练 + 核心消融 + 鲁棒性 + 实时性 | 主结果、消融趋势与负结果如实记录 |
| M5 | 鲁棒性 + LOSO + 实时性 + 报告 | 完整研究包 |

## 7. 风险与处理

- **4 向 vs 6 向**：无后视，主表不可直接对比；报告明确标注。
- **Gazebo 低分辨率 depth**：64×64 渲染后聚合到 8×8，必要时提高分辨率。
- **渲染吞吐**：headless、无 GUI、按场景并行、分片可续跑。
- **mesh 资产/license**：下载脚本 + 来源记录，资产不入库。
- **论文未公开细节**：clean-room 假设集中记录，关键项做敏感性/消融。
- **仿真到实机差异**：本阶段只验证方法和趋势；实机部署、真实状态分布、
  时间同步留到后续。

## 8. 交付物

- 完整代码、仿真世界、轨迹/数据生成脚本、训练/评估配置。
- 3 seeds checkpoint 与复现实验命令。
- 中文技术报告：方法、假设、数据、主结果、消融、鲁棒性、LOSO、
  实时性、局限。
- 论文式表格与图，包含失败/负结果。

## 9. 当前进度

- M0 完成：目录、配置、SE(3)/ToF/轨迹/IMU 工具、NPZ 数据容器和测试。
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q` 共 50 项通过。
- M1 完成：四个场景生成器、4 向 ToF rig、Gazebo 轨迹播放插件、
  单序列 headless 录制与 8×8 后处理。
- M1 实测（`corridor_demo`）：182 帧、12.07 s、1.80 m；4×8×8 @ 15 Hz；
  GT 时间对齐；有效 ToF zone 比例约 89%；范围 0.91–4.00 m。
- M2 完成：Gazebo Fuel mesh + 程序化几何、障碍物感知轨迹、批量并行录制、
  数据索引与随机/LOSO 划分。实测 **324 序列、9,728.9 s、2,081.6 m、
  146,265 帧**。随机划分 train/val/test = 165/56/103 序列；四场景有效
  zone 比例 75.2%–97.0%（总体 85.7%）。索引输出 `data/index.csv`、
  `data/splits.json`、`data/stats.json`、`data/report.md`。
- M3 完成：`tio_former/models/tio_former.py` 实现论文的 ToF 可靠性编码、
  4 视角共享 CNN + view embedding、view Transformer、IMU query 的 directional
  cross-attention、Local KV + Chunk-FIFO 流式 Transformer 和增量位姿头；
  `losses.py` 实现逐步、多视界、尺度与残差损失；`torch_dataset.py` 提供窗口
  数据集。模型 **1,129,542 参数**，流式与全序列因果输出一致。
  单序列 300 步过拟合损失 0.2658 → 0.0045（**−98.3%**），检查点
  `checkpoints/overfit_sequence.pt`（不提交）。
- M4 完成：单卡训练（AMP、warmup+cosine、梯度累积、早停、best/last 检查点）、
  流式缓存加速、整序列评估（SE(3) no-scale 对齐的 ATE/RPE/EDR）、核心消融、
  鲁棒性与 PyTorch 实时性。为修正“解析 IMU 过于干净导致 ToF 分支被忽略”的
  问题，提高了 IMU 噪声/零偏并用 `scripts/regenerate_imu.py` 重建全部 324 条
  序列的 IMU。主模型 40 epochs，best epoch 36，val 0.004567；103 条 test 序列
  结果 **ATE 0.4316 m、RPEp 0.0186 m、RPER 0.257°、EDR 9.49%**。四场景 ATE
  0.39–0.49 m。核心消融中 full 优于 IMU-only（0.4464 m）和 memoryless
  （0.4648 m），但 `naive_concat`（0.4013 m）与 `kv_only_no_fifo`
  （0.3925 m）更优；ToF-only 为 0.6636 m。这是本 4 向仿真下需要如实记录的
  负消融结果，不能据此声称复现了论文的 cross-attention/FIFO 增益。逐帧流式
  推理在 RTX 3060 上均值 1.95 ms、P95 2.31 ms，0/200 次超过 15 Hz 预算。
  结果汇总在 `report/m4_results.md`。
- M5 完成：3 seeds（0/1/2）主结果平均 **ATE 0.4073 ± 0.0218 m、
  RPEp 0.0179 ± 0.0005 m、RPER 0.315 ± 0.061°、EDR 8.98 ± 0.43%**；
  LOSO 平均 ATE **0.4652 m**（corridor 0.4702、office 0.4988、
  pantry 0.4621、meeting_room 0.4296）。新增
  `scripts/aggregate_seeds.py`、`scripts/run_loso.py`，场景过滤支持
  `train.py --exclude-scenes` 与 `evaluate.py --scenes`。中文技术报告
  `report/m5_report.md` 汇总方法、数据、主结果、消融、鲁棒性、LOSO、
  实时性与局限；M5 结果表与图在 `report/`。测试 50 项通过。
- 最佳模型诊断：seed1 测试 ATE 0.3788 m、EDR 8.42%，但预测轨迹长度只有
  GT 的 0.567；逐序列 oracle 尺度修正后 ATE 降至 0.2003 m。模型输出近似
  恒定单步位移（train/val/test 预测步长 1.24/1.32/1.44 cm，GT
  1.16/2.17/2.74 cm），说明速度相关的平移尺度塌缩是主要误差来源。
  图表与中文分析见 `research/tio_former/report/best_model/analysis.md`。
- 模型改进 v2：新增 `hybrid`、`gated_fifo`、`hybrid_gated` 三个变体，用
  可学习 gate 分别控制 cross-attention 和 Chunk-FIFO summary 分支，使模型
  从 `naive_concat` / `kv_only_no_fifo` 结构出发再学习增强。15-epoch 筛选
  中 `hybrid_gated` 为 0.4208 m；40-epoch 复核 ATE 0.4122 m，优于原 full
  主模型的 0.4316 m，但仍未超过 `naive_concat`（0.4013 m）和
  `kv_only_no_fifo`（0.3925 m），RPER 0.757° 也明显差于 full 的 0.257°；
  gate 基本未打开。位置/旋转双分支 `hybrid_split` 15-epoch ATE 0.4350 m、
  RPER 0.837°，没有改善。结论为负结果，保留原 full 模型与导出模型。结果记录在
  `report/model_v2_results.md`。
- 数据集 v3.1：按诊断结论重做数据分布，统一 train/val/test 速度 bin 并按目标速度
  重定时；共 **894 序列 / 22,927.5 s / 6,229.5 m / 344,829 帧**，split
  **618/135/141**，平均速度 **0.315/0.316/0.308 m/s**，平均步长
  2.10/2.11/2.05 cm。关闭合成 `status=10`，加入 3% dropout + 2% reflectivity
  dropout，并增加 0.6–2.2 m 垂直运动。审计通过平均速度偏移 2.56%、最小速度
  bin 77 条、test 路径 4–7 m、有效 ToF 0.783、无重复轨迹；仅 val–test KS
  0.173 未达 <0.1，属于分布形状的采样差异。分布图与审计见
  `research/tio_former/report/v3_dataset/analysis.md`。训练改为按
  `val_loss + 0.5*max(0, |log(val_path_ratio)| - log 1.2)` 选 checkpoint。
  seed 0 在 epoch 60 取得测试 **ATE 0.0880 m / RPEp 0.00353 m /
  RPER 0.193° / EDR 2.44%**，路径比 0.992，|speed–ATE| 0.344，v3 gate
  全部通过；结果见 `research/tio_former/report/v3_model/analysis.md`。
- C++/ROS 接入（实验性）：新增 `flyrat::tio`（`FLYRAT_BUILD_TIO`，CPU
  libtorch）和 ROS 2 `tio_odometry_node`，将导出的 TorchScript 模型
  `libflyrat/models/tio_former_window64.pt` 接入 `/flyrat/sensors/tof_0..3`
  与 `/flyrat/sensors/imu`，发布与骨架节点相同的 odometry/path/status/TF/reset
  接口。仿真回放和真实 4×VL53L5CX + IMU 都已验证 `TRACKING` 和有效里程计
  消息；默认构建不依赖 libtorch，但机载部署仍未验证。说明见
  `docs/agent/odometry/tio_former.md`。
- 真机验证（2026-11-08，旧布局）：四颗 ToF 设备端实测 14.8–15.2 Hz，0 读取
  错误；TIO 节点进入 `TRACKING`。但无人机静止放桌面 20 s 漂移约 0.98 m，
  主要受下视距桌面 0–5 mm、旧映射 front 槽位接后向和仿真域差异影响。
  当前模型不能用于飞行闭环；完整记录和下一步见
  `docs/agent/odometry/handoff.md`。
- 实机准备：`flyrat_sensor_forwarder` 默认改为 8×8 @ 15 Hz，并新增
  `--tof-resolution` / `--tof-rate`。机头改到原后向 ToF 一端后，传感器 ID 为
  0=下、1=前、2=右、3=左；TIO 节点用 `sensor_order: [1,3,2,0]` 映射到模型
  front/left/right/down。旧布局的 0.98 m 漂移需在新布局净空条件下复测。
- 运行说明见 `research/tio_former/README.md`。
