# 原始传感器录制与重新标定

本文说明如何为双目和 IMU 重新标定采集**未经旧标定修正**的数据。现有转发链路
会把 IMU 的 scale/misalignment、轴映射和时间偏移烘焙进
`/flyrat/sensors/imu`；相机话题虽然没有去畸变，但和 IMU 一起录制时无法还原
原始计数。因此重新标定使用独立的 `flyrat_raw_recorder`。

## 1. 录制内容

| 数据 | 内容 | 是否应用旧标定 |
|---|---|---|
| IMU | MSP `MSP_RAW_IMU` 的 `int16` acc/gyro/mag | 否 |
| IMU 时间 | `steady_clock` 采集时间戳（ns） | 否，不减 `imu_time_offset` |
| 相机 | V4L2 原生 MJPEG，1280x480 左右拼接 | 否，不拆分、不去畸变 |
| 相机时间 | V4L2 buffer 时间戳（ns） | 否 |
| 元数据 | lsb、轴映射名称、设备、分辨率、采集/交付 fps、时钟对 | 仅记录，不应用 |

录制器只做两件事：把 MSP 回调里的 `int16` 原样写入队列，把 V4L2 的 JPEG
原样写入队列。磁盘写入在独立线程完成，采集回调不阻塞。

## 2. 构建与推送

```sh
./.script/build_pilot_arm.sh
```

产物：

```text
build/arm/flyrat_raw_recorder
```

也可以只在本机验证格式，不接硬件：

```sh
cmake -S flyrat_pilot -B build/pilot-native -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/pilot-native --target flyrat_raw_recorder
./build/pilot-native/flyrat_raw_recorder --self-test
```

## 3. 检查设备

先确认飞控串口和相机没有被 forwarder 或 pilot 占用：

```sh
.script/bringup.sh raw check:=true
```

输出示例：

```text
raw recorder check: camera /dev/video10 1280x480 capture 60.0 fps, delivery 20 fps, throttled 0
raw recorder check: MSP /dev/ttyS3 @ 921600, IMU samples 300 in 1.5 s, timeouts 0, latency 1.2 ms
```

`check` 只打开设备，不创建 bag。

## 4. 录制

```sh
.script/bringup.sh raw \
  output:=/userdata/flyrat_raw/calib1 \
  duration:=10m \
  fps:=20 \
  capture_fps:=60 \
  export:=true
```

关键参数：

```text
output:=DIR          设备端输出目录；默认 /userdata/flyrat_raw/<时间戳>
duration:=10m        10 分钟；也支持 30s/3h；0 表示直到 Ctrl-C
fps:=20              交付/保留帧率
capture_fps:=60      向相机请求的采集帧率
imu_hz:=200          MSP_RAW_IMU 轮询率
local:=DIR           主机拉取目录；默认 <repo>/flyrat_raw
export:=true         拉取后自动运行 raw_export.py
require_capture:=true  相机协商不到 capture_fps 时直接失败
preview:=true        边录边把同一份 raw 数据转发到主机
preview_port:=5000   预览监听端口（设备与 adb forward 共用）
foxglove:=true       预览时自动启动 Foxglove bridge
imu_corrections:=false  预览 IMU 是否套用旧标定，默认 false
```

相机路径的行为是**按 60 fps 请求采集，再按 `fps` 丢帧**：

- `StereoJpegSource` 用 `VIDIOC_S_PARM` 请求 `capture_fps`；
- 采集线程每个 V4L2 buffer 都取回，但只有到交付周期的帧才复制 JPEG 并写 bag；
- 这样自动曝光上限仍是 1/60 s，暗处拖影比直接按 20 fps 采集短；
- `stats.throttled` 记录被交付节流丢掉的帧，`stats.dropped` 记录驱动层
  sequence 缺口。

如果相机协商不到 60 fps，录制器会打印 warning；加 `require_capture:=true`
可让脚本直接失败。

推荐采集动作：

- 双目：相机对准标定板，覆盖画面中心和四角，慢速平移/倾斜；
- 双目外参：保持标定板同时出现在左右目，覆盖不同深度；
- IMU：静止 2–5 分钟做 Allan/噪声，再充分转动三个轴；
- 相机-IMU：让相机和 IMU 一起做 6 轴激励，避免只有平移。

### 4.1 边录边预览（Foxglove）

加 `preview:=true` 后，录制器在写 bag 的同时，用与
`flyrat_sensor_forwarder` 相同的 TCP 协议把**同一份** raw IMU 计数和
MJPEG 帧发到主机。脚本会自动完成 `adb forward`、启动
`sensor_receiver_node`，并默认启动 Foxglove bridge：

```sh
.script/bringup.sh raw \
  output:=/userdata/flyrat_raw/calib1 \
  duration:=10m \
  fps:=20 capture_fps:=60 \
  preview:=true
```

打开 Foxglove，连接 `ws://<主机>:8765`，可看：

```text
/flyrat/sensors/imu
/flyrat/sensors/stereo/left/image_raw
/flyrat/sensors/stereo/right/image_raw
/flyrat/sensors/stereo/compressed
```

默认 `imu_corrections:=false`，预览的 IMU 只做 lsb→SI 和轴映射，不套用旧的
`M/A/C` 和 `imu_time_offset`，因此和正在录制的原始数据一致。需要对比旧标定
效果时可以加 `imu_corrections:=true`，但这只影响预览话题，**不会改变 bag**。

相关参数：

```text
preview:=true/false       是否边录边转发，默认 false
preview_port:=5000        设备监听端口和主机 adb forward 端口
foxglove:=true/false      是否自动启动 Foxglove bridge，默认 true
foxglove_port:=8765       Foxglove WebSocket 端口
imu_corrections:=false    预览 IMU 是否应用旧标定，默认 false
```

预览端口与 forwarder 默认的 5000 冲突，因此脚本会先停掉
`flyrat_sensor_forwarder` 和 `flyrat_pilot`；停止录制后自动移除
`adb forward` 并关闭预览进程。

## 5. 停止与拉取

`duration` 到期或按 `Ctrl-C` 后，脚本会：

1. 停止 MSP 和相机采集；
2. 排空写队列，写入 footer 和结束时钟对；
3. `adb pull` 到主机；
4. 可选运行导出。

输出结构：

```text
calib1/
  raw_sensors.bin
```

`raw_sensors.bin` 内部：

```text
"FLYRATRW" + version + JSON header
record(type, size, sequence, t_ns, payload)*
footer
```

记录类型：`1=IMU`（18 字节，`int16[9]`）、`2=IMAGE`（原始 JPEG）、
`3=EVENT`（JSON，含结束时钟对和统计）、`4=FOOTER`。

## 6. 导出

```sh
python3 flyrat_pilot/tools/raw_export.py \
  --input flyrat_raw/calib1 \
  --output flyrat_raw/calib1_export
```

输出：

```text
calib1_export/
  imu.csv              # timestamp,ax,ay,az,gx,gy,gz（SI，Kalibr 用）
  imu_counts.csv       # 原始 counts + t_ns/t_epoch_ns
  cam0/                # 左目 PNG
  cam1/                # 右目 PNG
  cam0.csv cam1.csv    # timestamp,filename
  metadata.json        # 原始 header + 导出统计
  make_kalibr_bag.sh
```

`--no-split` 只把整幅图像写到 `cam0/`，用于检查图像质量，不会生成
`cam1/` 或 `make_kalibr_bag.sh`，不能直接用于 Kalibr。

时间戳：

- 默认用录制起止的 `steady ↔ system` 时钟对映射到 epoch，写入 CSV 和文件名；
- `--no-epoch` 保留原始 `steady_clock` 纳秒，适合自己对齐；
- IMU 和相机共用设备端 `steady_clock`，相对时间关系不受 epoch 映射影响。

单位与轴：

- `imu.csv` 的 acc 为 m/s²，gyro 为 rad/s；
- `imu_counts.csv` 保留 MSP 原始计数和 mag；
- 默认不翻转轴；`--accel-axis frd_to_flu` / `--gyro-axis frd_to_flu`
  只做符号翻转，不改变量纲；
- 导出不会应用旧 `M/A/C`，也不会减旧 `imu_time_offset`。

## 7. 生成 Kalibr bag

导出目录已经符合 Kalibr 的 folder 结构。安装 `kalibr_bagcreater` 后运行：

```sh
cd flyrat_raw/calib1_export
./make_kalibr_bag.sh
```

或手动：

```sh
kalibr_bagcreater --folder . --output-bag calibration.bag
```

然后按标定目标选择：

```sh
# 双目内参 + 外参
kalibr_calibrate_cameras --bag calibration.bag --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan --target april_6x6.yaml

# 相机-IMU
kalibr_calibrate_imu_camera --bag calibration.bag \
  --cam cam0.yaml --imu imu.yaml --target april_6x6.yaml
```

Kalibr 产物写回仓库前，保持现有方向约定：

- 相机 YAML：`camera.T_left_right` 表示 `p_left = T_left_right * p_right`；
- `imu.T_body_camera` 表示相机→body；
- 当前后端只支持 `pinhole-radtan` 的 `k1,k2,p1,p2`，不支持 k3 或鱼眼；
- 图像分辨率必须与 `camera.left/right.width/height` 一致，否则 odometry 会
  抛 `image size does not match the camera calibration`。

## 8. 与旧数据的关系

| 参数 | 旧 bag / 话题是否已烘焙 | 重新标定后 |
|---|---|---|
| 相机内参、畸变 | 否 | 换 YAML 即可 |
| 双目外参 | 否 | 换 YAML 即可 |
| 相机-IMU 外参 | 否 | 换 YAML 即可 |
| IMU scale/misalignment | 是 | 旧数据不能直接套新参数 |
| IMU 时间偏移 | 是 | 旧数据时间戳需按新旧 offset 差修正 |
| IMU 零偏 | 否（后端在线估计） | 不受影响 |

因此：

- 旧的相机 bag 仍可用新 YAML 回放，只要分辨率不变；
- 旧的 IMU bag / `/flyrat/sensors/imu` 不能直接用于新标定；
- 需要复用旧录制时，必须先按旧 `M/A/C` 和旧 `imu_time_offset` 反解，
  再重新标定；实际更稳妥的是用 raw recorder 重录。

## 9. 常见问题

- `EBUSY` 或打不开 `/dev/video10`：先停掉 `flyrat_sensor_forwarder`、
  `flyrat_pilot` 和旧的 recorder；bringup 脚本会做这一步。
- `no IMU samples were recorded`：检查 `/dev/ttyS3`、波特率和飞控供电；
  用 `check:=true` 先确认。
- 相机协商帧率低于 60：确认当前分辨率/格式支持 60 fps；如果加了
  `--require-capture-fps`，录制会直接失败，这是预期行为，不要靠降低要求
  掩盖相机模式问题。
- 图像数量明显少于 `fps × duration`：看 footer 里的 `dropped_images` 和
  `throttled`；前者是队列满，后者是正常的交付丢帧。
- IMU 数量少于 `imu_hz × duration`：看 `dropped_imu` 和
  `last_imu_latency_ms`；USB/串口吞吐不足时降低 `imu_hz`。
