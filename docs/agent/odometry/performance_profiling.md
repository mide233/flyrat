# 机载 SLAM 分阶段耗时

日期：2026-11-11

测量对象：`flyrat_pilot` + SchurVINS，RK3566，双目 1280x480 MJPEG，
`--decode mpp`，静态近距桌面场景。每次 600 帧，统计时跳过前 100 帧
（初始化/尺度收敛阶段会明显拉高 p95）。

复现：

```bash
adb shell '/tmp/flyrat_pilot --camera-config /tmp/flyrat/config/camera.yaml \
  --imu-config /tmp/flyrat/config/imu.yaml --decode mpp \
  --trace /tmp/trace --max-frames 600'
adb pull /tmp/trace .
python3 flyrat_pilot/tools/analyze_trace.py trace --skip 100
```

## 25 fps

| 项目 | p50 (ms) | p95 (ms) |
|---|---:|---:|
| queue（解码完成 → SLAM 取帧） | 1.12 | 1.99 |
| decode（MPP） | 6.96 | 8.15 |
| imu_wait | 0.001 | 0.002 |
| collect | 0.03 | 0.05 |
| **track()** | **27.51** | **31.13** |
| publish | 0.02 | 0.03 |
| age（图像时间戳 → 开始跟踪） | 48.67 | 50.42 |

`track()` 内部（SVO 计时器，单位已换算为 ms）：

| 阶段 | p50 | p95 | 占 frontend 比例 |
|---|---:|---:|---:|
| `pyramid_creation`（含 2× `imgs[i].clone()`） | 2.93 | 4.23 | 11.9% |
| `sparse_img_align`（SchurVINS `Forward`） | 0.98 | 1.53 | 4.0% |
| `reproject`（`projectMapInFrame`） | 12.70 | 19.81 | 53.1% |
| `pose_optimizer`（SchurVINS `Backward`） | 6.83 | 8.29 | 26.4% |
| `point_optimizer` | 1.18 | 1.49 | 4.5% |
| `local_ba` | 0.00 | 0.00 | 0.0% |
| `frontend_time` | 24.18 | 28.61 | — |
| 未计入阶段（`frontend_time` − 上述阶段） | 1.77 | 2.25 | — |
| `pyramid_creation` + `frontend_time` | 27.30 | 30.94 | — |

## 30 fps

| 项目 | p50 (ms) | p95 (ms) |
|---|---:|---:|
| queue | 0.99 | 1.99 |
| decode | 8.04 | 8.28 |
| **track()** | **21.61** | **23.85** |
| age | 42.05 | 46.47 |

| 阶段 | p50 | p95 | 占比 |
|---|---:|---:|---:|
| `pyramid_creation` | 3.94 | 4.11 | 18.1% |
| `sparse_img_align` | 1.15 | 1.45 | 5.5% |
| `reproject` | 14.12 | 15.60 | 66.5% |
| `pose_optimizer` | 1.82 | 2.01 | 9.0% |
| `point_optimizer` | 0.18 | 0.24 | 0.9% |
| `frontend_time` | 18.15 | 19.66 | — |
| 未计入阶段 | 0.58 | 0.71 | — |

## 结论

1. `reproject` / `projectMapInFrame` 是单帧最大开销，25/30 fps 分别约
   12.7 / 14.1 ms，占 `frontend_time` 的 53–66%，且 25 fps 的 p95 接近
   20 ms，是延迟波动的主要来源。
2. `pose_optimizer`（`SchurVINS::Backward`）在 25 fps 约 6.8 ms，但 30 fps
   只有 1.8 ms，随跟踪状态/窗口内容变化明显；它不是唯一热点。
3. `pyramid_creation` 约 3–4 ms，其中包含每帧两次 `imgs[i].clone()`；
   图像所有权和金字塔缓冲池有直接优化空间。
4. `Frontend` 的 `Forward`（`sparse_img_align`）只有约 1 ms，不是瓶颈。
5. 未计入阶段在稳态只有 0.6–2.3 ms；启动阶段（前几十帧）会出现 20 ms
   以上的尖峰，主要来自初始化/尺度调整路径，分析时应跳过预热帧。
6. `track()` 与 `pyramid_creation + frontend_time` 的差约 0.2 ms，说明
   `track()` 的计时闭环可信，`--trace` 的文件写入开销不明显。
7. `local_ba` 在两组 600 帧中均为 0，样本里没有触发关键帧 BA；关键帧
   路径的尖峰尚未覆盖。
8. `decode` 不在关键路径上：25 fps 下 `age` ≈ 一个帧周期 + 解码 + 排队，
   继续优化解码不会降低稳态帧年龄。

## 后续优化候选（只改实现，不动算法参数）

按数据排序：

1. `projectMapInFrame()`：逐候选点循环中的重复投影、临时对象和每帧分配；
   先加更细计时，再决定是否缓存/预分配。
2. `pyramid_creation`：让 `StereoJpegSource`/MPP 输出直接被 `Frame` 接管，
   去掉 `clone()` 和每帧金字塔分配。
3. `SchurVINS::Backward`：`Solve3` 中的 Eigen 动态矩阵临时量、`V.inverse()`
   和逐点 `state_factors` 的组合；在 25 fps 场景下再细分 Jacobian /
   Schur 补 / `StateUpdate`。
4. 关键帧路径：用更长或更运动的序列触发 `local_ba`，确认尖峰来源。
5. 线程与发布：`publish` 已只有 0.02 ms，暂不需要移出 SLAM 线程。
