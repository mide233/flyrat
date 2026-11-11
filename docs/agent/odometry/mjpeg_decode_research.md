# 机载 MJPEG 硬件解码

日期：2026-11-11

## 结论

RK3566 的 VPU 可以连续解码 1280x480 MJPEG，硬件解码明显低于软件解码的
CPU 占用，输出与 OpenCV 一致。当前实现只作为独立解码器和验收工具存在，
**尚未接入 `flyrat_pilot` 默认路径**，机载默认仍是 CPU `cv::imdecode`。

```text
V4L2 MMAP 采集 -> StereoJpegSource -> 解码线程 -> SchurVINS
                                      |-- cv::imdecode（默认）
                                      `-- MppJpegDecoder（已实现，未接入）
```

## 实现

### `MppJpegDecoder`

`libflyrat/src/hardware/mpp_jpeg_decoder.cpp`，设备根文件系统没有 MPP
开发头文件，因此用 `dlopen` 解析 `librockchip_mpp.so.1`（回退 `.so.0`），
只声明用到的 `MppApi` 前缀并用 `static_assert` 固定偏移。

关键点：

- MJPEG 不能使用同步 `MppApi::decode`，必须走
  `decode_put_packet` / `decode_get_frame`。
- 异步路径会丢弃没有 `MppBuffer` 的输入包（`mpp_dec_normal.c` 中
  `mpp_packet_get_buffer(packet) == NULL` 时直接 deinit）。因此每帧先把
  JPEG 复制进复用的 ION/DRM 缓冲区，再 `mpp_packet_init_with_buffer`，
  用 `mpp_packet_set_length` 设置真实长度。
- 输出帧通过 `mpp_packet_get_meta` + `mpp_meta_set_frame(KEY_OUTPUT_FRAME)`
  预分配并复用，缓冲区大小按 `hor_stride * ver_stride * 2` 计算，兼容
  NV12/NV16。
- 启动时通过 `MPP_DEC_SET_OUTPUT_FORMAT` 请求 NV12，并设置
  `MPP_SET_OUTPUT_TIMEOUT` 让 `decode_get_frame` 内部阻塞，避免 1 ms 轮询。
- 若解码器仍返回 info-change 帧，则创建输出 buffer group，执行
  `MPP_DEC_SET_EXT_BUF_GROUP` 和 `MPP_DEC_SET_INFO_CHANGE_READY` 后继续。
- 只复制 Y 平面，`stride` 为 MPP 的 `hor_stride`；宽度/高度从 JPEG SOF
  段解析。

### 删除的 GStreamer 路径

`v4l2src + mppjpegdec + appsink` 实现已从 `libflyrat` 删除，
`StereoCamera` 只保留 YUYV。历史实现仍可从 git 找到。

### `decode_bench`

`flyrat_pilot/tools/decode_bench.cpp`，只使用实时相机：

```sh
adb push build-arm/decode_bench /tmp/
adb shell '/tmp/decode_bench --camera /dev/video10 --mode both --seconds 60'
```

`--mode mpp|cpu|both`，`--verify-every N` 每 N 帧用 `cv::imdecode` 对比
同一 JPEG 的 Y 平面。失败条件：帧数低于期望的 98%、出现超过 100 ms 的
间隔、解码错误、平均误差达到 2 LSB，或硬件 p99 不低于软件 p99
（仅 mpp 模式时阈值 15 ms）。

## 实测

RK3566（4 核 A55，2 GB），`/dev/video10`，1280x480@25 MJPEG：

| 运行 | 帧数 | 帧率 | 间隔 >100 ms | CPU | 平均 | p99 | 最大 |
|---|---:|---:|---:|---:|---:|---:|---:|
| mpp，60 s | 1496 | 24.91 | 0 | 27.0% | 11.27 ms | 12.14 ms | 12.48 ms |
| cpu，10 s | 246 | 24.48 | 0 | 56.4% | 22.05 ms | 22.92 ms | 22.98 ms |
| both，60 s | 1496 | 24.91 | 0 | 61.6% | mpp 11.89 / cpu 13.72 | 13.22 / 16.69 | 13.48 / 23.48 |

- both 模式 60 s 内 59 次逐像素校验：平均绝对误差 0.01 LSB，最大 1 LSB。
- MPP 输出格式为 NV12，60 s 内 `info_changes=0`，`source_drops=0`，
  `source_errors=0`。
- 硬件解码把解码 CPU 从约 0.5 核降到约 0.2 核（进程 CPU 含采集线程）。

需要注意：同步调用下每帧硬件解码延迟约 11–12 ms，与软件解码同量级，
因此收益主要是 CPU 和功耗，而不是单帧延迟。GStreamer 管线可以重叠
put/get，但完整 pilot 中会出现 UVC 停流；直接 MPP 路径暂未在 pilot 中
复测。

## 后续

1. 在 pilot 中增加 `--decode cpu|mpp`（默认 cpu），用 `shared_ptr` 持有
   Y 平面并按 `stride` 构造左右目 `cv::Mat`，保留 CPU 回退。
2. 验证与 SchurVINS 并发时相机是否稳定：至少 3 次 600 帧 25/30 fps，
   记录 `cam_frames`、`camera_drops`、`track_ms`、`age_ms`。
3. 若仍出现 UVC 停流，用 `usbmon`、`dmesg`、`perf` 抓停流瞬间，并评估
   解码独立进程/共享内存或 MIPI CSI。
4. 接入前保持 25 fps CPU 默认；需要降低延迟时优先使用
   `stamp_source:=host`、bridge 立即发布和 10 ms lookahead。
