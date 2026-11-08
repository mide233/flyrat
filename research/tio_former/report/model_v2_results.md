# TIO-Former 模型改进实验（v2）

## 背景

M4 的核心消融出现负结果：在真实感 IMU 仿真数据上，`naive_concat`
（0.4013 m）和 `kv_only_no_fifo`（0.3925 m）优于完整 cross-attention +
Chunk-FIFO 模型（0.4316 m）。本实验尝试两种“从无增益结构出发、再学习增强”
的变体，目标是让完整模型至少不劣于这两个基线。

## 实现

新增三个变体（`tio_former/models/tio_former.py`）：

- `hybrid`：保留原 cross-attention，同时并联一个 naive-concat 分支；用
  标量 gate（初始 −6，sigmoid ≈ 0.0025）控制 attention 分支。初始时等价于
  naive-concat，训练中可以逐步启用方向注意力。
- `gated_fifo`：保留原 cross-attention 融合，但把 local KV 注意力和 FIFO
  summary 注意力分开计算，用 gate 控制 summary 分支；初始时等价于
  kv-only/no-FIFO。
- `hybrid_gated`：两者同时启用。

参数量：full 1,129,542；hybrid 1,162,695；gated_fifo 1,129,544；
hybrid_gated 1,162,697。九个变体的 forward 形状与流式/全序列一致性测试
均通过。

## 15-epoch 筛选结果（seed 0，103 条 test 序列）

| variant | ATE (m) | RPEp (m) | RPER (°) | EDR (%) |
|---|---:|---:|---:|---:|
| full（40-epoch 主模型） | 0.4316 | 0.0186 | 0.257 | 9.49 |
| naive_concat（15ep） | 0.4013 | 0.0186 | 0.754 | 9.67 |
| kv_only_no_fifo（15ep） | 0.3925 | 0.0191 | 0.705 | 8.67 |
| hybrid（15ep） | 0.4815 | 0.0204 | 0.629 | 11.20 |
| gated_fifo（15ep） | 0.4929 | 0.0208 | 0.621 | 11.18 |
| hybrid_gated（15ep） | **0.4208** | 0.0218 | 0.842 | 9.60 |

15-epoch 筛选里 `hybrid_gated` 是三个新变体中最好的，ATE 优于 40-epoch
full 主模型，但仍未超过 `naive_concat` 和 `kv_only_no_fifo`。两个 gate 在
训练后都仍接近初始值（sigmoid ≈ 0.0025），说明模型没有主动启用
attention/FIFO 分支，性能主要来自 concat/local 路径。RPER 反而变差。

## 40-epoch 复核

`hybrid_gated` 以与主模型相同的 40 epochs、seed 0 训练，best epoch 10，
val 0.004751，早停于 epoch 25。103 条 test 序列结果：

| 指标 | hybrid_gated 40ep | full 40ep（主模型） |
|---|---:|---:|
| ATE (m) | **0.4122** | 0.4316 |
| RPEp (m) | 0.0183 | 0.0186 |
| RPER (°) | 0.757 | **0.257** |
| EDR (%) | **9.28** | 9.49 |

分场景 ATE：corridor 0.3837、meeting_room 0.3761、office 0.4602、
pantry 0.4234。`hybrid_gated` 的位置精度和终点漂移优于原 full 主模型，
但仍未超过 15-epoch 的 `naive_concat`（0.4013 m）和
`kv_only_no_fifo`（0.3925 m）；旋转误差明显更差。40-epoch 复核没有推翻
15-epoch 筛选的结论。

## 位置/旋转双分支 `hybrid_split`

`hybrid_split` 让位置与旋转使用不同的融合/时序路径：位置走
concat(mean view, IMU) + local/FIFO，旋转走 directional attention +
local/FIFO，两条流式分支分别输出 `delta_p` / `delta_phi`。参数量
1,708,108。15 epochs、seed 0、103 条 test 序列结果：

| 指标 | hybrid_split 15ep | naive_concat 15ep | full 40ep |
|---|---:|---:|---:|
| ATE (m) | 0.4350 | **0.4013** | 0.4316 |
| RPEp (m) | 0.0191 | 0.0186 | 0.0186 |
| RPER (°) | 0.837 | 0.754 | **0.257** |
| EDR (%) | 10.01 | 9.67 | 9.49 |

双分支没有带来预期收益：ATE 劣于 full 和两个负结果基线，旋转误差也没有
改善。best epoch 为 5（val 0.005089），随后验证损失回升。

## 结论

- 当前没有证据表明 hybrid/gated/split 结构能稳定解决 M4 的负消融。
  `naive_concat` 与 `kv_only_no_fifo` 仍是该 4 向仿真数据上的最强基线。
- `hybrid_gated` 40-epoch 的位置指标优于原 full 主模型，但旋转明显退化，
  且仍不及两个基线；`hybrid_split` 没有改善。
- 因此保留原 full 模型作为主模型与导出模型，不替换
  `libflyrat/models/tio_former_window64.pt`；本实验作为诚实的负结果记录。
- 所有数字均为 4 向 ToF 仿真，不能与论文 6 向主表直接比较。
