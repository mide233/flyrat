# 实机传感器转发到 ROS 2

本文说明如何把泰山派上的 IMU、4 颗 VL53L5CX ToF 和双目相机，通过 adb 转发到
开发机，并在 ROS 2 中发布为标准/自定义话题，用于算法调试。项目整体状态见
[`project_status.md`](project_status.md)。

## 1. 数据路径

```text
泰山派 flyrat_sensor_forwarder
  ├─ MspInteract    /dev/ttyS3       IMU，原始计数
  ├─ TofArray       i2c2             4 × VL53L5CX
  └─ StereoJpegSource /dev/video10   原生 MJPEG，左右拼接
              │
              ▼
       TCP 127.0.0.1:5000
              ▲
              │ adb forward tcp:5000 tcp:5000
              │
开发机 flyrat_sensors/sensor_receiver_node
  ├─ sensor_msgs/Imu
  ├─ flyrat_sensors_interfaces/TofFrame ×4
  └─ sensor_msgs/Image 左右目
```

设备端不解码双目 MJPEG。当前 adb 链路实测约 30 MB/s：

| 数据 | 带宽 |
|---|---:|
| 原生 MJPEG 1280x480@60 | 约 3.2 MB/s |
| 原始 GRAY8 640x480/眼@60 | 约 36.9 MB/s |
| 原始 NV12 640x480/眼@60 | 约 55 MB/s |

因此默认只用原生 MJPEG 过 adb，开发机再用 OpenCV 解码、拆左右。

## 2. 设备端

构建：

设备端需要交叉编译 `flyrat_sensor_forwarder`。`libflyrat` 的 ULD 是 C 源码，
建议分别用 `aarch64-linux-gnu-gcc` 编译 C 文件、用 `aarch64-linux-gnu-g++`
编译 C++ 文件，最后链接 `-pthread -ldl`。也可以按 `flyrat_pilot/CMakeLists.txt`
中的 target 配置自己的 CMake toolchain。

注意：VL53L5CX 的输出裁剪宏（`VL53L5CX_DISABLE_*`）统一写在
`libflyrat/third_party/vl53l5cx/platform/platform.h`，C 和 C++ 不需要再传
`-D`。这些宏会改变 `VL53L5CX_ResultsData` 的内存布局；如果只给 C 文件传、
不给 C++ 文件传，驱动和读取端会按不同布局解释同一块缓冲区，表现为距离和
`target_status` 全为 0。不要再按翻译单元单独定义这些宏。

当前设备的可执行文件可以直接放在 `/tmp/flyrat_sensor_forwarder` 运行。

运行：

```sh
./flyrat_sensor_forwarder --listen 127.0.0.1:5000
```

常用参数：

```text
--msp-device /dev/ttyS3   飞控 MSP 串口
--msp-baud 921600
--camera-device /dev/video10
--camera-width 1280       整幅左右拼接宽度
--camera-height 480
--camera-fps 60
--no-imu / --no-tof / --no-stereo
```

只监听回环地址；adb forward 会从开发机连到设备的 5000 端口。

ToF 的 4 个 I2C 地址已经硬编码为 `0x60/0x62/0x64/0x66`，不再支持单独配置。
每次启动会逐颗释放 LPn，自动探测传感器上次残留的地址，初始化后再统一改回这四个
固定地址；即使没有断电、传感器停在旧地址，也能自动恢复。四颗 ToF 初始化大约需要
7.2 秒，主要耗时在 ULD 固件下载和参数配置。

## 3. 开发机

```sh
adb -s 4c24d56142f3d0e3 forward tcp:5000 tcp:5000
```

或使用安装到 ROS 包里的脚本：

```sh
ros2 run flyrat_sensors adb_forward.sh 2>/dev/null || \
  $(ros2 pkg prefix flyrat_sensors)/share/flyrat_sensors/scripts/adb_forward.sh
```

启动接收节点：

```sh
cd flyrat_play/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch flyrat_sensors sensors.launch.py
```

参数：

```text
host=127.0.0.1
port=5000
reconnect_ms=1000
acc_lsb_per_g=2048
gyro_lsb_per_dps=16.4
stereo_color=false
tof_count=4
```

`MSP_RAW_IMU` 的尺度跟飞控板有关，默认值只是常见起点；需要按实际板子校准。

## 4. 话题

```text
/flyrat/sensors/imu
/flyrat/sensors/tof_0
/flyrat/sensors/tof_1
/flyrat/sensors/tof_2
/flyrat/sensors/tof_3
/flyrat/sensors/stereo/left/image_raw
/flyrat/sensors/stereo/right/image_raw
```

- IMU：`sensor_msgs/msg/Imu`，orientation 暂无，协方差首项为 `-1`。
- ToF：每颗一个 `flyrat_sensors_interfaces/msg/TofFrame`，`zones[64]`，
  `valid = target_status in {5, 9}`。
- 双目：默认 `mono8`，`stereo_color:=true` 时 `bgr8`；宽高为单眼尺寸。

## 5. 验收

```sh
ros2 topic hz /flyrat/sensors/imu
ros2 topic hz /flyrat/sensors/tof_0
ros2 topic hz /flyrat/sensors/stereo/left/image_raw
ros2 topic echo --once /flyrat/sensors/tof_0
```

预期：

- IMU 约 200 Hz；
- 每颗 ToF 实测约 57–58 Hz（请求 60 Hz，受传感器实际帧周期限制）；
- 左右图像各约 60 Hz，`1280x480` 模式下单眼 `640x480`。

拔掉 adb 后 receiver 会每秒重连；重新执行 `adb forward` 后恢复。

## 6. 已知限制

- ToF 地址是易失的，掉电后会回到 `0x52`；当前的启动探测会自动恢复并改回固定地址。
  LPn 拉低只关闭 I2C，不复位地址。
- 四颗 ToF 初始化约 7.2 秒，forwarder 在这段时间内还不能提供 ToF 数据；
  IMU 和双目也会等 `start()` 完成，因此从启动到监听约 7 秒。
- 双目是原生 MJPEG，有损；需要原始像素时应改为低帧率 GRAY8/NV12 模式。
- 设备时间戳和主机时间戳通过 `TIME` 帧做偏移估计，不是硬实时同步。
- MJPEG 解码在开发机完成，设备端 CPU 只负责采集和发送。
- 双目和 ToF 没有硬件同步，跨传感器时间戳只能按到达时间/设备时间近似对齐。
