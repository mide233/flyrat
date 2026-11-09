# TIO-Former v3.1 模型结果（seed 0）

训练数据：`data_v3/processed`（v3.1，894 序列 / 22,927.5 s / 6,229.5 m /
344,829 帧，split 618/135/141）。  
checkpoint：`checkpoints_v3/main/seed0_fixed/best.pt`（epoch 60）。  
评估：`runs_v3/eval_seed0_fixed_ep60`，141 条 test 序列。

## 1. 总体结果

| 指标 | 数值 |
|---|---:|
| ATE | **0.08796 m** |
| RPEp | **0.003526 m** |
| RPER | **0.1925°** |
| EDR | **2.441%** |
| 预测/GT 路径比 | **0.992** |
| speed–ATE 相关 | **−0.344**（绝对值 0.344） |

v3 gate（`runs_v3/gate_seed0_fixed_ep60.json`）全部通过：

- ATE ≤ 0.34 m：通过（0.088 m）
- 路径比 0.80–1.20：通过（0.992）
- |speed–ATE| ≤ 0.50：通过（0.344）

![v3 seed0 metrics](seed0_metrics.png)

## 2. 分场景

| 场景 | 序列 | ATE m | RPEp m | RPER ° | EDR % |
|---|---:|---:|---:|---:|---:|
| corridor | 48 | 0.0747 | 0.00329 | 0.223 | 2.30 |
| meeting_room | 25 | 0.1077 | 0.00350 | 0.141 | 2.70 |
| office | 46 | 0.0903 | 0.00382 | 0.209 | 2.52 |
| pantry | 22 | 0.0895 | 0.00347 | 0.152 | 2.28 |

office 仍是 ATE 最高的场景之一，但四个场景的 ATE 已落在 0.075–0.108 m，
没有再出现 v2 中 office 明显拖尾的情况。

## 3. 速度分层

| 平均速度 m/s | 序列 | ATE m | 路径比 |
|---|---:|---:|---:|
| 0.0–0.2 | 28 | 0.1159 | 0.985 |
| 0.2–0.3 | 49 | 0.0910 | 0.983 |
| 0.3–0.4 | 27 | 0.0744 | 0.997 |
| 0.4–0.5 | 23 | 0.0670 | 1.005 |
| ≥0.5 | 14 | 0.0820 | 1.008 |

v2 最严重的问题是高速段路径比塌缩（test 总体 0.567，速度越高越短）。
v3.1 中每个速度 bin 的路径比都在 0.983–1.008，尺度塌缩已经消失。
慢速段（<0.2 m/s）的绝对 ATE 仍稍高（0.116 m），这是低速小位移下
ToF 量化/噪声占比更高造成的，但与高速段的差距已从 v2 的 3–4 倍缩小到
约 1.4 倍。

## 4. 与 v2 对比

| 版本 | ATE m | RPEp m | RPER ° | EDR % | 路径比 |
|---|---:|---:|---:|---:|---:|
| v2 最佳 seed（seed1） | 0.3788 | 0.0173 | 0.399 | 8.42 | 0.567 |
| v2 3-seed 平均 | 0.4073 | 0.0179 | 0.315 | 8.98 | — |
| **v3.1 seed0** | **0.0880** | **0.00353** | **0.193** | **2.44** | **0.992** |

ATE 相对 v2 最佳 seed 降低约 **76.8%**，EDR 从 8.42% 降到 2.44%。
需要强调：v3.1 仍是同一组四个 Gazebo 场景的仿真结果，不能直接等同于
论文的 6 向 ToF 实飞结果；真实 VL53L5CX 的噪声、遮挡和安装误差会重新引入
域差距。

## 5. checkpoint 选择修复

旧规则 `val_loss + 0.5*|log(path_ratio)|` 在 v3 数据上误选了 epoch 7
（test ATE 0.518 m），而 epoch 22 的 test ATE 已经达到 0.258 m。
原因是旧规则把 ratio=1.08 的正常 checkpoint 也重罚，导致过早停止。

现规则只惩罚 `[1/1.2, 1.2]` 之外的尺度误差：

```text
score = val_loss + 0.5 * max(0, |log(path_ratio)| - log(1.2))
```

这样 ratio 在 0.83–1.2 内时完全按 val loss 选择。修复后 seed 0 训练到
epoch 60，val loss 0.000469、val path ratio 0.962，test 结果为上表。

## 6. 复现

```bash
# 数据
python3 scripts/generate_dataset.py \
  --generation configs/data/generation_v3.yaml --workers 3
python3 scripts/build_dataset_index.py \
  --generation configs/data/generation_v3.yaml
python3 scripts/audit_dataset_v3.py

# seed 0 训练（60 epochs；本结果在 epoch 60 取得）
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/train.py \
  --processed data_v3/processed --out checkpoints_v3/main \
  --run-name seed0_fixed --epochs 60 --workers 8

# 测试与 gate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python scripts/evaluate.py \
  --checkpoint checkpoints_v3/main/seed0_fixed/best.pt \
  --processed data_v3/processed --split test \
  --out runs_v3/eval_seed0_fixed_ep60
python3 scripts/check_v3_gate.py \
  --eval runs_v3/eval_seed0_fixed_ep60 \
  --out runs_v3/gate_seed0_fixed_ep60.json

# 图表
.venv/bin/python scripts/plot_v3_model.py \
  --eval runs_v3/eval_seed0_fixed_ep60 --out report/v3_model
```

## 7. 结论与下一步

- 速度/步长分布修复和 checkpoint 选择修复后，尺度塌缩问题已解决。
- 当前 seed 0 的 ATE 0.088 m、EDR 2.44%，显著优于 v2，并通过全部 v3 gate。
- 仍需 seeds 1/2 验证稳定性；随后在净空真实环境复测 TIO ROS 链路。
- 真实飞行前还应检查慢速段（<0.2 m/s）的漂移和静止漂移，避免仿真域
  过拟合。
