# TIO-Former 最佳模型：轨迹、精度与失效因素分析

生成脚本：`scripts/analyze_best_model.py`  
模型检查点：`checkpoints/main_imu/seed1/best.pt`  
分支：`dev/odom-tio`  
说明：这里展示 3 个 seed 中测试 ATE 最好的 seed 1；总体泛化结论仍以
3-seed 均值 **0.4073 ± 0.0218 m** 为准，不能把 best-of-3 当作无偏指标。

## 1. 结论摘要

- seed 1 在 **103 条 test 序列、1,468.6 s、562.7 m** 上：
  **ATE 0.3788 m、RPEp 0.0173 m、
  RPER 0.399°、EDR 8.42%**。
- 位置误差的主要来源不是旋转，而是**平移尺度塌缩**：
  预测轨迹长度只有 GT 的 **0.567**；对每条序列做 oracle 尺度修正后，
  ATE 从 **0.379 m 降到 0.200 m（约
  47%）**。
- 模型对速度的响应几乎饱和。train / val / test 的真实平均步长分别约为
  1.16 /
  2.17 /
  2.74 cm，
  但模型预测的平均步长只有
  1.24 /
  1.32 /
  1.44 cm，
  几乎不随真实步长变化。
  也就是说，模型学到的是训练集的“平均步长先验”，而不是从 ToF/IMU 中恢复真实尺度。
- 这解释了为什么 test 越快、ATE 越大：test 平均速度 0.383 m/s，
  train 只有 0.160 m/s。test 被系统性低估；
  train 上预测步长 1.24 cm 也略高于
  GT 的 1.16 cm，且序列更长，
  因而累积 EDR 达到 17.2%。

## 2. 轨迹叠加

![best/worst per scene](figures/trajectory_gallery.png)

每个场景给出 ATE 最好和最差的测试序列。GT 为黑色虚线，TIO-Former 为彩色实线，
绿色圆点为起点，黑色方块为 GT 终点，红色叉号为预测终点。较好的序列能保持轨迹形状，
但普遍“缩水”；较差的办公室 canonical 序列在转弯后明显丢失平移尺度。

![3D best/median/worst](figures/trajectory_3d.png)

![position error over time](figures/error_over_time.png)

## 3. 精度统计

![accuracy overview](figures/metrics_overview.png)

| scene | n | ATE m | RPEp m | RPER deg | EDR % | speed m/s | valid ratio | pred/GT path |
|---|---|---|---|---|---|---|---|---|
| corridor | 35 | 0.350 | 0.0165 | 0.411 | 8.22 | 0.390 | 0.891 | 0.567 |
| office | 32 | 0.427 | 0.0194 | 0.349 | 8.68 | 0.458 | 0.739 | 0.555 |
| pantry | 16 | 0.384 | 0.0170 | 0.493 | 7.77 | 0.402 | 0.960 | 0.572 |
| meeting_room | 20 | 0.348 | 0.0154 | 0.385 | 8.90 | 0.381 | 0.838 | 0.582 |

![scene and trajectory type](figures/scene_kind.png)

| split | n | ATE m | RPEp m | RPER deg | EDR % | speed m/s | GT step cm | pred step cm | pred/GT path |
|---|---|---|---|---|---|---|---|---|---|
| train | 165 | 0.525 | 0.0071 | 0.216 | 17.19 | 0.160 | 1.16 | 1.24 | 1.372 |
| val | 56 | 0.499 | 0.0139 | 0.337 | 9.68 | 0.294 | 2.17 | 1.32 | 0.811 |
| test | 103 | 0.379 | 0.0173 | 0.399 | 8.42 | 0.383 | 2.74 | 1.44 | 0.567 |

注：`pred/GT path` 是逐序列路径长度比的平均值，近静止序列会放大该值；
平均步长比见图 `speed_scale.png` 左下角。test 的两种统计都表明明显欠尺度。

canonical 轨迹的 ATE 明显高于 random：canonical 平均
**0.538 m**，
random 平均 **0.343 m**。
canonical 的旋转误差反而更低（RPER
0.130°），
说明问题主要是平移尺度，而不是朝向估计。

## 4. 关键诊断：尺度塌缩

![speed and scale diagnosis](figures/speed_scale.png)

- 测试集 speed 与 ATE 的相关系数为 **+0.743**；
  speed 与 pred/GT path ratio 的相关系数为 **-0.865**：
  越快的轨迹，预测越“缩水”。
- path ratio 与 ATE 的相关系数为 **-0.810**：
  尺度误差几乎直接决定 ATE。
- 平均最优尺度为 **1.83×**，说明模型平均只走了真实位移的
  56.7%。
- 分速度段：

| speed m/s | n | ATE m | oracle-scale ATE m | pred/GT path | mean optimal scale |
|---|---|---|---|---|---|
| 0.0-0.3 | 21 | 0.215 | 0.153 | 0.762 | 1.29 |
| 0.3-0.4 | 39 | 0.342 | 0.190 | 0.610 | 1.61 |
| 0.4-0.5 | 22 | 0.439 | 0.213 | 0.480 | 2.00 |
| 0.5-0.6 | 9 | 0.489 | 0.237 | 0.434 | 2.29 |
| 0.6-2.0 | 12 | 0.594 | 0.267 | 0.345 | 2.84 |

在低速段模型甚至过冲，在高速段严重欠冲；这不是简单的“整体偏小”，而是
**速度相关增益缺失**。

### 为什么会出现尺度塌缩

1. **ToF 对平移尺度的可观测性不足。** 4 个 8×8、45° FoV 的视角（缺后/上），
   8×8 像素角分辨率约 5.6°；在 1 m 距离上约对应 10 cm 的横向分辨，
   而 0.2–0.8 m/s、15 Hz 下单帧位移只有 1.3–5.3 cm。单帧 ToF 差分量与
   1 cm 级测距噪声、8×8 稀疏采样同量级，模型很难从中稳定读出速度。
2. **损失函数鼓励回归到条件中位数。** 每步位移用米制 Smooth L1 监督；
   当输入无法确定尺度时，最优策略是输出训练分布的平均步长。
   论文的 scale loss 权重只有 0.005，且是 64 帧窗口内的总位移比，
   不足以阻止尺度塌缩。
3. **训练/测试速度分布不一致。** 数据生成时 train 为 20–60 s，
   test 被限制为 4–7 m / 8–20 s，导致 test 平均速度约为 train 的
   2.4 倍。
   test 因此落在模型“平均步长”先验的右侧，系统性欠冲。
4. **训练窗口太短，缺少长时一致性。** 训练样本是独立的 64 帧窗口（约 4.3 s），
   推理却跑完整 8–20 s（train 20–60 s）序列，没有跨窗口的尺度/漂移约束。
   train 上 ATE 反而更高、EDR 17.2%，说明
   长时累积误差同样是问题。

## 5. 其他因素

![feature correlations](figures/feature_correlations.png)

| feature | Pearson r with ATE |
|---|---|
| speed | +0.743 |
| path length | +0.611 |
| duration | -0.501 |
| valid ToF ratio | -0.297 |
| left view valid | -0.366 |
| vertical range | -0.523 |
| yaw rate | +0.249 |
| pred/GT path ratio | -0.810 |

- **ToF 有效率是次要因素。** office 的有效 zone 比例最低（约 74%），
  ATE 最高（0.427 m）；
  但 valid ratio 与 ATE 的相关系数只有 -0.297，
  控制速度后的偏相关约 -0.130。
  遮挡会加剧误差，但不是主因。
- **旋转不是主因。** RPER 平均只有
  0.399°，
  最终朝向误差虽会累积，但轨迹误差的主要投影仍然是平移尺度。
- **垂直运动与误差负相关**（r=-0.523），
  这不是“垂直运动更容易”，而是垂直运动较多的序列往往更慢、更短，
  是速度混杂因素。

![view validity](figures/view_validity.png)

## 6. 架构与消融

![ablations](figures/ablation.png)

- full 主模型（seed0）ATE 0.4316 m；seed1 0.3788 m；seed2 0.4116 m。
- `naive_concat` 0.4013 m、`kv_only_no_fifo` 0.3925 m 仍优于 full；
  `imu_only` 0.4464 m，`tof_only` 0.6636 m。
- 丢弃全部 ToF 视角时 ATE 0.4257 m，和 full 0.4316 m 几乎相同：
  当前 4 向仿真数据下，ToF 分支没有提供稳定的尺度/漂移校正。
- 这说明瓶颈不是“cross-attention 不够复杂”，而是**输入可观测性 + 训练目标**
  没有给模型提供可学习的尺度信号。继续加结构很可能只会过拟合。

## 7. 建议的优先级

1. **先修数据分布**：让 train/val/test 的速度、步长分布一致；不要用
   “test 4–7 m / 8–20 s”这种会系统性提高 test 速度的采样方式。
2. **显式处理尺度**：增加序列级/长时尺度损失，或输出显式速度/尺度状态；
   不要只依赖窗口内的弱 scale loss。
3. **提高 ToF 可观测性**：补后/上视角，或评估 6 向布局；在 4 向硬件上，
   考虑用陀螺去旋转后的 range flow / 几何残差，而不是纯黑盒。
4. **做长时训练**：跨窗口状态传递或 TBPTT，加入绝对轨迹/漂移约束。
5. **真机前先做净空静态和匀速直线测试**；当前旧布局真机 20 s 漂移约 0.98 m，
   新布局尚未复测，不能用于闭环飞行。

## 8. 方法说明与限制

- 轨迹图为 SE(3) no-scale 对齐后的可视化；ATE 是位置 RMSE，EDR 是末端误差
  除以 GT 路径长度。
- “oracle per-seq scale”只是诊断上界，不是可部署方法；它说明尺度误差占多大比例。
- 所有结论基于 Gazebo 4 向 ToF 仿真，不能直接等同于论文的 6 向真实飞行结果。
- seed1 是 best-of-3，报告顶部同时给出 3-seed 均值以避免选择性偏差。
