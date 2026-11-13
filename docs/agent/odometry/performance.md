# 性能与解码

本文汇总机载 SchurVINS 的基线测量、分阶段计时、已完成优化，以及
MJPEG 硬件解码的调研与结论。历史实验过程见 git 记录。

## 1. 测量条件

- 设备：RK3566，4 核 Cortex-A55，2 GB RAM，`/dev/video10` 1280x480 MJPEG。
- 解码：`--decode mpp`（除特别标注）；交付帧率 25 或 30 fps。
- 时长：600 帧；分析跳过前 100 帧，避免初始化尖峰。
- 场景：静态桌面；`track()` 与 `pose_optimizer` 会随场景/窗口状态波动，
  对比时优先看同一轮次内的相对变化。

复现：

```sh
./.script/build_pilot_arm.sh
adb push build/arm/flyrat_pilot /tmp/
adb shell '/tmp/flyrat_pilot \
    --camera-config /tmp/flyrat/config/camera.yaml \
    --imu-config /tmp/flyrat/config/imu.yaml \
    --decode mpp --max-frames 600 --trace /tmp/trace25'
adb pull /tmp/trace25 ./trace25
python3 flyrat_pilot/tools/analyze_trace.py ./trace25 --skip 100
```

`--trace DIR` 会生成：

- `trace_pilot.csv`：`queue_ms, decode_ms, imu_wait_ms, collect_ms,
  track_ms, publish_ms, age_ms`，以及 `state/valid`。
- `trace_frontend.csv`：SVO 内部计时，单位为秒；包含
  `pyramid_creation, sparse_img_align, reproject, pose_optimizer,
  point_optimizer, local_ba, frontend_time`。

## 2. 优化前基线（25 fps）

pilot 时间线，p50/p95（ms）：

| 阶段 | p50 | p95 |
|---|---:|---:|
| queue | 1.12 | 1.99 |
| decode | 6.96 | 8.15 |
| imu_wait | 0.001 | 0.002 |
| collect | 0.03 | 0.05 |
| **track** | **27.51** | **31.13** |
| publish | 0.02 | 0.03 |
| age | 48.67 | 50.42 |

SVO 阶段（ms，括号为 `frontend_time` 占比）：

| 阶段 | p50 | p95 | 占比 |
|---|---:|---:|---:|
| pyramid_creation | 2.93 | 4.23 | 11.9% |
| sparse_img_align（Forward） | 0.98 | 1.53 | 4.0% |
| **reproject** | **12.70** | **19.81** | **53.1%** |
| pose_optimizer（Backward） | 6.83 | 8.29 | 26.4% |
| point_optimizer | 1.18 | 1.49 | 4.5% |
| local_ba | 0 | 0 | 0 |
| frontend_time | 24.18 | 28.61 | 100% |
| 未计入 | 1.77 | 2.25 | — |
| pyramid + frontend | 27.30 | 30.94 | — |

`track()` 与 `pyramid + frontend` 只差约 0.2 ms，说明 `track_ms` 已覆盖
克隆、金字塔和整个前端。

30 fps 基线：`track` 21.61/23.85 ms，`age` 42.05/46.47 ms。SVO 阶段：
pyramid 3.94/4.11（18.1%）、align 1.15/1.45（5.5%）、reproject
14.12/15.60（66.5%）、pose 1.82/2.01（9.0%）、point 0.18/0.24（0.9%）、
frontend 18.15/19.66、未计入 0.58/0.71。

## 3. 关键结论

1. `reproject`（`projectMapInFrame`）是最大热点，25/30 fps 占 53–66%，
   25 fps p95 接近 20 ms，是延迟波动的主要来源。
2. `pose_optimizer`（`SchurVINS::Backward`）随状态/窗口变化很大，
   实测 1.8–6.8 ms。
3. `pyramid_creation` 3–4 ms，其中每帧两次 `imgs[i].clone()`
   （2×614 KB）有优化空间。
4. `sparse_img_align`（Forward）约 1 ms，不是瓶颈。
5. 稳态未计入仅 0.6–2.3 ms；启动前几十帧可到 20+ ms，分析需 `--skip 100`。
6. `decode` 不在关键路径：帧年龄约等于帧周期 + decode + queue，
   继续降低解码耗时不会明显降低稳态 `age_ms`。
7. `local_ba` 在两次 600 帧中均为 0，未触发关键帧 BA，关键帧尖峰未覆盖。
8. trace 写文件开销不明显（`track` 与阶段和仅差 0.2 ms）。

## 4. 已完成的优化

### 4.1 ARM NEON 路径修复

vendored 代码使用 `__ARM_NEON__` 判断，aarch64 GCC 只定义 `__ARM_NEON`，
导致 NEON 路径被静默关闭。以下文件改为
`#if defined(__ARM_NEON) || defined(__ARM_NEON__)`：

- `viking/vikit_common/src/vision.cpp`（`halfSample`）
- `svo_direct/include/svo/direct/patch_score.h`（ZMSSD）
- `svo_direct/src/feature_alignment.cpp`（1D/2D 对齐）
- `fast/include/fast/faster_corner_utilities.h`

`halfSample` 的 NEON 与标量实现都是 `/4` 截断，结果逐位一致；金字塔
从 3–4 ms 降到 1.2–1.5 ms，`reproject` 再省 1–3 ms。

### 4.2 双目并行重投影

`libflyrat/src/odometry/schurvins_backend.cpp` 设置
`base.use_async_reprojectors = true`。`projectMapInFrame` 用
`std::async` 按相机并行，重投影墙钟时间从 `sum(cam)` 降为 `max(cam)`，
18–20 ms → 10–13 ms。

注意：启用后不能用 `--slam-cpu`，否则 `std::async` 工作线程继承单核
亲和性，重投影反而变慢；bringup 脚本会忽略 `slam_cpu:=` 并警告。

### 4.3 未采用的方案

- 去掉 `imgs[i].clone()`：只省 0.15–0.3 ms，但要求 MPP 的 Y 平面成为
  自持 `cv::Mat`，关键帧图像所有权复杂，收益不足。
- OpenCV 线程 1 vs 2：无可测差异，保持默认 2。

### 4.4 优化后对比

p50/p95（ms）：

| 指标 | 25 fps 优化前 | 25 fps 优化后 | 30 fps 优化前 | 30 fps 优化后 |
|---|---:|---:|---:|---:|
| track | 27.51/31.13 | **16.57/18.20** | 21.61/23.85 | **15.99/18.26** |
| pyramid | 2.93/4.23 | 1.43/1.52 | 3.94/4.11 | 1.39/1.58 |
| reproject | 12.70/19.81 | 10.41/10.70 | 14.12/15.60 | 10.72/12.55 |
| pose_optimizer | 6.83/8.29 | 1.85/3.02 | 1.82/2.01 | 1.59/2.30 |
| frontend_time | 24.18/28.61 | 14.89/16.44 | 18.15/19.66 | 14.38/16.51 |
| age | 48.67/50.42 | 49.8/51.1 | 42.05/46.47 | 42.3/46.4 |

CPU 约 123–144%。`track()` 降幅约 35%，25 fps 余量约 23 ms，
30 fps 余量约 17 ms。`reproject` 仍占 67–72%，是后续首要目标。

## 5. MJPEG 硬件解码

### 5.1 `MppJpegDecoder`

`libflyrat/src/hardware/mpp_jpeg_decoder.cpp`。设备根文件系统没有 MPP
开发头文件，因此用 `dlopen` 解析 `librockchip_mpp.so.1`（回退 `.so.0`），
只声明用到的 `MppApi` 前缀并用 `static_assert` 固定偏移。

关键点：

- MJPEG 不能使用同步 `MppApi::decode`，必须走
  `decode_put_packet` / `decode_get_frame`。
- 异步路径会丢弃没有 `MppBuffer` 的输入包。因此每帧先把 JPEG 复制进
  复用的 ION/DRM 缓冲区，再 `mpp_packet_init_with_buffer`，用
  `mpp_packet_set_length` 设置真实长度。
- 输出帧通过 `mpp_packet_get_meta` + `mpp_meta_set_frame(KEY_OUTPUT_FRAME)`
  预分配并复用，缓冲区按 `hor_stride * ver_stride * 2` 计算，兼容 NV12/NV16。
- 启动时通过 `MPP_DEC_SET_OUTPUT_FORMAT` 请求 NV12，并设置
  `MPP_SET_OUTPUT_TIMEOUT` 让 `decode_get_frame` 内部阻塞，避免 1 ms 轮询。
- 若仍返回 info-change 帧，则创建输出 buffer group，执行
  `MPP_DEC_SET_EXT_BUF_GROUP` 和 `MPP_DEC_SET_INFO_CHANGE_READY` 后继续。
- 只复制 Y 平面，`stride` 为 MPP 的 `hor_stride`；宽高从 JPEG SOF 解析。

`StereoCamera` 的 GStreamer `v4l2src + mppjpegdec` 路径已删除，只保留
YUYV；历史实现可从 git 找到。

### 5.2 `decode_bench`

`flyrat_pilot/tools/decode_bench.cpp`，只使用实时相机：

```sh
adb push build/arm/decode_bench /tmp/
adb shell '/tmp/decode_bench --camera /dev/video10 --mode both --seconds 60'
```

`--mode mpp|cpu|both`，`--verify-every N` 每 N 帧用 `cv::imdecode` 对比
同一 JPEG 的 Y 平面。失败条件：帧数低于期望的 98%、出现超过 100 ms 的
间隔、解码错误、平均误差达到 2 LSB，或硬件 p99 不低于软件 p99
（仅 mpp 模式时阈值 15 ms）。

### 5.3 实测

1280x480@25 MJPEG：

| 运行 | 帧数 | 帧率 | 间隔 >100 ms | CPU | 平均 | p99 | 最大 |
|---|---:|---:|---:|---:|---:|---:|---:|
| mpp，60 s | 1496 | 24.91 | 0 | 27.0% | 11.27 ms | 12.14 ms | 12.48 ms |
| cpu，10 s | 246 | 24.48 | 0 | 56.4% | 22.05 ms | 22.92 ms | 22.98 ms |
| both，60 s | 1496 | 24.91 | 0 | 61.6% | mpp 11.89 / cpu 13.72 | 13.22 / 16.69 | 13.48 / 23.48 |

- both 模式 60 s 内 59 次逐像素校验：平均绝对误差 0.01 LSB，最大 1 LSB。
- MPP 输出 NV12，`info_changes=0`、`source_drops=0`、`source_errors=0`。
- 解码 CPU 从约 0.5 核降到约 0.2 核（进程 CPU 含采集线程）。
- 同步调用下每帧硬件解码约 11–12 ms，与软件解码同量级；收益主要是
  CPU 和功耗，而不是单帧延迟。

### 5.4 pilot 集成

`flyrat_pilot --decode cpu|mpp`，二进制默认 `cpu`，bringup 脚本默认 `mpp`。
`mpp` 模式下 Y 平面由 `shared_ptr` 持有，左右目按解码 `stride` 构造
`cv::Mat`；初始化失败或连续 10 帧解码失败时自动回退到 `cv::imdecode`，
统计行累加 `decode_fallbacks`。

完整 SchurVINS 进程，每组 600 帧：

| 配置 | CPU | RSS | `track()` | 帧年龄 | 帧率 |
|---|---:|---:|---:|---:|---:|
| 25 fps，`--decode mpp` ×3 | 126–130% | 20–22 MB | 25–26 ms | 约 50 ms | 25.0 fps |
| 25 fps，`--decode cpu` | 135% | 18–19 MB | 25.5 ms | 51.5 ms | 25.0 fps |
| 30 fps，`--decode mpp` ×3 | 136–140% | 20–22 MB | 21–23 ms | 42–43 ms | 30.0 fps |
| 30 fps，`--decode cpu` | 142% | 18–19 MB | 22.4 ms | 42.4 ms | 30.0 fps |

6 次 `mpp` 运行均为 `camera_drops=0`、`decode_fallbacks=0`、
`cam_errors=0`。按线程统计，解码线程从约 27–32% 降到 12–13%
（另加 `mpp_dec_parser` 约 1.2%），即解码阶段节省约 0.15–0.20 核；
整机 CPU 只下降约 4–7 个百分点，帧年龄基本不变，因为解码位于帧预算内。

## 6. 当前建议

1. 保持 `--decode cpu` 为二进制默认；`mpp` 作为已验证的可选路径，继续
   收集长时间和高温稳定性数据，再决定是否切换默认。
2. 性能优化优先继续看 `reproject` 内部和 `SchurVINS::Backward` 的
   Jacobian / Schur 补 / `StateUpdate`。
3. 若再次出现 UVC 停流，用 `usbmon`、`dmesg`、`perf` 抓现场，并评估
   解码独立进程/共享内存或 MIPI CSI。
4. 需要降低端到端延迟时，优先使用 `stamp_source:=host`、bridge 立即发布
   和 10 ms lookahead；硬件解码的收益主要在 CPU 和功耗。
