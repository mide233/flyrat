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

## 优化实验（2026-11-11）

在不改算法参数的前提下，逐项试验了以下实现级优化，并在同一设备上用
`--trace` 对比。场景会随时间变化（`invalid` 帧数波动），因此每轮交替运行
不同二进制，取 `invalid` 较少的健康轮次比较。

### 方案 A：启用 ARM NEON 路径

问题根因：vendored 代码用 `#ifdef __ARM_NEON__` 判断 NEON，而 aarch64 GCC
只定义 `__ARM_NEON`，导致金字塔、patch score、feature alignment 的 NEON
实现全部未编译。

改动文件：

- `viking_common/src/vision.cpp`（`halfSample`，3 处）
- `svo_direct/include/svo/direct/patch_score.h`（3 处）
- `svo_direct/src/feature_alignment.cpp`（4 处）
- `fast/include/fast/faster_corner_utilities.h`（1 处）

统一改为 `#if defined(__ARM_NEON) || defined(__ARM_NEON__)`。

效果：

- `pyramid_creation` 3–4 ms → **1.2–1.5 ms**，约省 2 ms/帧；
- `reproject`（SSD patch 匹配 + 1D/2D 对齐）再省 1–3 ms；
- 金字塔 NEON 与标量实现都是整数 `/4` 截断，输出逐位一致；patch score 的
  ZMSSD 也是整数运算。属于无算法行为变化的加速。

### 方案 B：双目重投影并行

`base.use_async_reprojectors = true`，让 `projectMapInFrame()` 对左右目
各起一个 `std::async` 任务并等待完成。原来两目顺序执行，墙钟是
`cam0 + cam1`；并行后约为 `max(cam0, cam1)`。

效果（`invalid` 较少的轮次）：`reproject` 18–20 ms → **10–13 ms**，
约省 6–8 ms/帧。

注意：

- `--slam-cpu` 与并行重投影冲突。`std::async` 工作线程继承 SLAM 线程的
  CPU 亲和性，单核会退化为串行甚至更慢；启用并行时不要固定 SLAM 线程。
- 两目共享点上的 `n_failed/succeeded_reproj_` 计数存在良性竞争，上游本来
  就提供该选项；点删除在 join 之后统一执行。

### 方案 C：减少帧拷贝（未采用）

去掉 `addImageBundle()` 里的 `imgs[i].clone()`，预计只省 0.15–0.3 ms，
而且要求 MPP 输出直接成为 `Frame` 拥有的 `cv::Mat`。当前 `FramePacket`
用 `shared_ptr<vector<uint8_t>>` 持有 luma 并用 stride 视图切左右目，
强行复用会让关键帧图像的所有权变复杂，收益与风险不成比例，暂不采用。

### 方案 D：OpenCV 线程数（无收益）

`--opencv-threads 1` 与默认 2 对比，`track`/`reproject` 无稳定差异，
保持默认 2。

### 最终对比（600 帧，`--decode mpp`，跳过前 100 帧，p50/p95 ms）

| 指标 | 基线 25 fps | 优化后 25 fps | 基线 30 fps | 优化后 30 fps |
|---|---:|---:|---:|---:|
| `track` | 27.51 / 31.13 | **16.57 / 18.20** | 21.61 / 23.85 | **15.99 / 18.26** |
| `pyramid_creation` | 2.93 / 4.23 | 1.43 / 1.52 | 3.94 / 4.11 | 1.39 / 1.58 |
| `reproject` | 12.70 / 19.81 | 10.41 / 10.70 | 14.12 / 15.60 | 10.72 / 12.55 |
| `pose_optimizer` | 6.83 / 8.29 | 1.85 / 3.02 | 1.82 / 2.01 | 1.59 / 2.30 |
| `frontend_time` | 24.18 / 28.61 | 14.89 / 16.44 | 18.15 / 19.66 | 14.38 / 16.51 |
| 进程 CPU | 约 125–142% | 约 123% | 约 142% | 约 144% |
| 帧年龄 | 48.7 / 50.4 | 49.8 / 51.1 | 42.1 / 46.5 | 42.3 / 46.4 |

（`pose_optimizer` 随窗口/场景状态在 1.6–8.4 ms 间波动，是整轮对比中最大
的不确定项；优化后 25 fps 的 `track` 在健康场景下稳定在 16–17 ms。）

### 结论

最优组合是 **方案 A + 方案 B**：

- 单帧 `track()` 从 25–27 ms 降到 **16–17 ms**（约 −35%），30 fps 也有
  15–16 ms；
- 25 fps 帧预算 40 ms，优化后余量约 23 ms；30 fps 预算 33 ms，余量约 17 ms；
- `reproject` 仍是最大头（约 10.5 ms，占 67–72%），但已从 p95 接近 20 ms
  降到 11–13 ms，延迟波动明显收敛；
- 这两项都不改变算法参数：NEON 是同一套整数运算，异步重投影只是把两目的
  独立工作并行执行。

后续若要继续压低 `track()`，优先级应是 `reproject` 内部（候选点循环与临时
对象）和 `pose_optimizer`（`SchurVINS::Backward`）的微优化，而不是解码。
