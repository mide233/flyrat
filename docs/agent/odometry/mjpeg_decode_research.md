# 机载 MJPEG 硬件解码调研

日期：2026-11-11

## 结论

RK3566 的 VPU/`mppjpegdec` 单独解码 1280x480 MJPEG 时明显省 CPU，但在
`flyrat_pilot` 的完整 SchurVINS 进程中，GStreamer `v4l2src + mppjpegdec`
在 25/30 fps 下会在数帧到数百帧内停止出流。20 fps 可以跑较长一段，但
CPU/单帧耗时并不优于当前 CPU 解码路径。

因此当前默认仍是：

```text
V4L2 MMAP 采集 -> 独立解码线程 cv::imdecode -> SchurVINS
```

硬件路径保留在 `libflyrat` 的 `StereoCamera` 中，不接入默认 pilot 启动流程。

## 两条候选路径

### A. GStreamer + mppjpegdec

`StereoCamera` 的 MJPEG 模式：

```text
v4l2src io-mode=2
  ! image/jpeg,width=1280,height=480,framerate=25/1
  ! jpegparse
  ! mppjpegdec
  ! video/x-raw,format=NV12
  ! appsink max-buffers=1 drop=true sync=false
```

输出 NV12 的 Y 平面直接作为灰度图，左右目按 640 列切开。

已实现的改进：

- 从 `GstBuffer` 读取 PTS，并按首帧建立 `steady_clock` 映射，避免使用
  解码到达时间；
- `StereoCamera::takeLatest()` 移动帧，避免 Y 平面复制；
- `StereoCamera` 无回调时移动帧而不是复制。

### B. 直接调用 librockchip_mpp

尝试用 `dlopen("librockchip_mpp.so.1")` + `mpp_create` / `mpp_init` /
`MppApi::decode` 直接把 JPEG 解码到 NV12，绕开 GStreamer 和 `v4l2src`。

结果：同步 `MppApi::decode` 在 MJPEG 上会段错误。查阅 MPP 的
`test/mpi_dec_test.c` 后确认：

```c
cmd->simple = (cmd->type != MPP_VIDEO_CodingMJPEG) ? 1 : 0;
```

MJPEG 不能走 simple 的 `decode` 路径，必须使用
`decode_put_packet` + `decode_get_frame`，并准备输出帧 buffer group
（MJPEG 可能输出 YUV420/422，需要足够的输出缓冲）。该路径尚未实现完成。

## 实测数据

### 单独相机

`StereoCamera` MJPEG 25 fps，30 s：

| 指标 | 结果 |
|---|---|
| 解码帧数 | 743 |
| 实际帧率 | 约 24.8 fps |
| 相机错误 | 0 |
| 进程稳定性 | 正常 |

说明 MPP/GStreamer 管线本身可以持续工作。

### 完整 pilot + 硬件解码

| 配置 | 结果 |
|---|---|
| 25 fps | 约 5–100 帧后相机停止出流；`cam_running=1`、`decoder_errors=0` |
| 30 fps | 同样在数帧后停止 |
| 20 fps / 600 帧 | 曾完整跑完：CPU 约 146%，`track_ms` 35–38 ms，`age_ms` 约 16 ms |
| 20 fps 再次启动 | 受上一次非正常退出影响，可能首帧后停流；单独相机测试仍正常 |

### 对照实验

- `StereoCamera + MSP 200 Hz`：25 fps 连续 20 s 正常，约 493 帧。
- 再加 `stress-ng --cpu 2/4 --cpu-method matrix/fft`：仍正常。
- 说明单纯 CPU 负载不是充分原因；问题只在完整 pilot（含 SchurVINS）时出现。

### 解码 CPU 对比

`gst-launch` 8 s、25 fps：

| 解码器 | user+sys | 约合 |
|---|---|---|
| `mppjpegdec` | 0.496 + 0.203 s | 约 0.09 核 |
| 软件 `jpegdec` | 2.947 s | 约 0.37 核 |

硬件解码单独可省约 0.3 核，但完整 pilot 中未转化为稳定收益。

## 当前判断

1. 硬件解码不是“代码没接上”的问题：单独相机和 MSP 组合都稳定。
2. 完整 pilot 下 UVC 流被停止，`cam_running` 仍为 1、`cam_errors=0`，
   更像 USB/VPU 驱动或调度层面的停流，而不是 GStreamer 报错。
3. 直接 MPP 同步接口不适用于 MJPEG，需要异步输出帧管理；这是下一步
   最值得做的方向，可绕开 `v4l2src` 和 GStreamer。
4. 在解决停流之前，不应把硬件解码设为默认，否则会出现“启动几帧后
   没有新图像、SLAM 停在第一帧”的不可接受行为。

## 后续建议

按优先级：

1. 实现 MPP 异步解码：`mpp_frame_init` + buffer group +
   `decode_put_packet` / `decode_get_frame`，先用离线 JPEG 序列验证
   输出格式和 PTS。
2. 把 MPP 解码放到独立进程或共享内存生产者，验证是否能隔离
   `v4l2src`/SVO 的相互影响。
3. 用 `usbmon`、`dmesg`、`perf` 和 `GST_DEBUG=3` 抓停流瞬间，确认是
   UVC isochronous 停流、MPP 缓冲耗尽还是调度问题。
4. 若 UVC 路径无法稳定，评估 MIPI CSI `ov5695` + ISP，或使用外接供电的
   USB 相机，绕开当前 UVC 供电/带宽边界。
5. 保持当前 CPU 解码默认值 25 fps；需要更低延迟时优先用
   `stamp_source:=host`、bridge 立即发布和 10 ms lookahead，而不是牺牲
   稳定性启用未验证的硬件路径。
