# 实机传感器转发到 ROS 2

本文说明如何把泰山派上的 IMU、4 颗 VL53L5CX ToF 和黑白单目相机，通过 adb 转发到
开发机，并在 ROS 2 中发布为标准/自定义话题，用于算法调试。项目整体状态见
[`status/project_status.md`](../status/project_status.md)，编译与运行命令见
[`build_and_run.md`](../build_and_run.md)。

## 1. 数据路径

```text
泰山派 flyrat_sensor_forwarder
  ├─ MspInteract    /dev/ttyS3       IMU，原始计数
  ├─ TofArray       i2c2             4 × VL53L5CX
  └─ MonoJpegSource  /dev/video10   原生 MJPEG，640x480 黑白
              │
              ▼
       TCP 127.0.0.1:5000
              ▲
              │ adb forward tcp:5000 tcp:5000
              │
开发机 flyrat_sensors/sensor_receiver_node
  ├─ sensor_msgs/Imu
  ├─ flyrat_sensors_interfaces/TofFrame ×4
  └─ sensor_msgs/Image 单目 mono8
```

设备端不解码相机 MJPEG。当前 adb 链路实测约 30 MB/s；640x480@60
黑白 MJPEG 通常远低于 1 MB/s。默认只把原生 MJPEG 过 adb，开发机再用
OpenCV 解码为 `mono8`。

## 2. 设备端

交叉编译和推送命令见
[`build_and_run.md`](../build_and_run.md) 的“flyrat_pilot 与传感器转发”。

运行时只监听回环地址：

```sh
./flyrat_sensor_forwarder --listen 127.0.0.1:5000
```

常用参数：

```text
--msp-device /dev/ttyS3   飞控 MSP 串口
--msp-baud 921600
--camera-device /dev/video10
--camera-width 640        单目图像宽度
--camera-height 480
--camera-fps 15         输出帧率（相机实际采集后节流）
--camera-exposure-us 5000  手动曝光，单位微秒；0 表示保留相机自动曝光
--camera-gain -1        手动增益；-1 表示不修改（通常仍是自动增益）
--tof-resolution 8      4 或 8；默认 8
--tof-rate 15           4x4 最高 60，8x8 最高 15；默认 15
--no-imu / --no-tof / --no-camera
```

### 相机曝光与运动模糊

`MonoJpegSource` 在打开 `/dev/video10` 后会写入 UVC 曝光控制：

- `exposure_auto=manual`
- `exposure_absolute=exposure_us/100`（UVC 以 100 µs 为单位）
- `exposure_auto_priority=0`，锁定帧间隔，避免相机用降帧换取长曝光

默认 **5 ms** 曝光，明显短于自动曝光常见的 20–30 ms。启动日志会打印每个
控制的写入值和读回值，例如：

```text
camera /dev/video10: exposure_auto=1 (readback 1)
camera /dev/video10: exposure_absolute=50 (readback 50)
camera /dev/video10: exposure_auto_priority=0 (readback 0)
```

若某控制不被相机支持，会打印 `unsupported` 并继续采集。曝光变短后画面
变暗属于预期，可用 `--camera-gain`（或 bringup 的 `camera_gain:=`）提高增益，
或把 `camera_exposure_us:=` 调大。建议先用 5 ms 录一段，再根据平均亮度和
模糊程度微调。

TIO-Former 需要 4 颗 ToF 都输出 **8×8 @ 15 Hz**，因此
`flyrat_sensor_forwarder` 默认已改为 8×8、15 Hz。若要临时回到 4×4，
可显式传 `--tof-resolution 4 --tof-rate 60`。

ToF 的安装方向由 INT/LPn 的电气连接确定。RK3566 的 Linux GPIO 编号为
`bank*32 + group*8 + pin`（A=0，B=1，C=2，D=3），当前实机对应关系为：

| 传感器 ID | 方向 | INT | LPn | Linux GPIO |
|---:|---|---|---|---|
| 0 | 下 | 3B3 | 3B4 | 107 / 108 |
| 1 | 前 | 3B0 | 3C2 | 104 / 114 |
| 2 | 右 | 3A4 | 3A5 | 100 / 101 |
| 3 | 左 | 3A1 | 3A2 | 97 / 98 |

`TofArray` 的 `int_gpio` / `lpn_gpio` 数组顺序必须与上表一致：
`int = {107, 104, 100, 97}`，`lpn = {108, 114, 101, 98}`。
ROS 话题 `/flyrat/sensors/tof_0..3` 也按这个传感器 ID 发布。TIO-Former
节点再通过 `sensor_order` 把它映射到模型的
front/left/right/down 视角。当前机头已改到原后向 ToF 一端，因此映射为
`[1, 3, 2, 0]`，详见
[`../odometry/tio_former.md`](../odometry/tio_former.md)。

VL53L5CX 的输出裁剪宏（`VL53L5CX_DISABLE_*`）统一写在
`libflyrat/third_party/vl53l5cx/platform/platform.h`。这些宏会改变
`VL53L5CX_ResultsData` 的内存布局，C 和 C++ 必须使用同一套；不要再按翻译单元
单独传 `-D`，否则会出现距离和 `target_status` 全为 0。

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
bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source install/setup.bash && \
  ros2 launch flyrat_sensors sensors.launch.py'
```

如果还要启动里程计节点，不需要手动执行上面的 `adb forward`、设备 forwarder 和
接收节点；用一键启动即可：

```sh
.script/bringup_real.sh
```

它会把 adb forward、设备端 forwarder、`sensor_receiver_node` 和
`odometry_node` 一起拉起，详见
[`../build_and_run.md`](../build_and_run.md) 的“一键 bringup”。

`sensors.launch.py` 暴露的参数：

```text
host=127.0.0.1
port=5000
acc_lsb_per_g=2048.0
gyro_lsb_per_dps=16.4
imu_accel_axis=none
imu_gyro_axis=none
```

节点还支持 `reconnect_ms`、`tof_count`、`imu_frame_id`、`tof_frame_id`、
`camera_frame_id`（默认 `camera_front_optical_frame`），直接运行节点时可用
`--ros-args -p 名字:=值` 传入。

`MSP_RAW_IMU` 的尺度跟飞控板有关。当前实机的
`flyrat_sensors/config/imu_intrinsics.yaml` 同时设置了：

```text
imu_intrinsics_enable: true
imu_time_offset: 0.0038053510703300744
imu_acc_M / imu_gyr_M / imu_gyr_A / imu_gyr_C_gyro_i
```

`sensor_receiver_node` 先用 `acc_lsb_per_g` / `gyro_lsb_per_dps` 把原始计数
转成 SI 单位、按 `imu_*_axis` 对齐 FLU，再应用 Kalibr 的尺度/轴失准矩阵：

```text
a_body = inv(M_acc) * a_raw
a_gyro = C_gyro_i * a_body
w_body = C_gyro_i^T * inv(M_gyr) * (w_raw - A * a_gyro)
```

零偏仍由里程计后端在线估计（待实现）。`imu_time_offset` 会从 IMU 时间戳中减去，
用来补偿 Kalibr 估计的相机-IMU 时间偏移；关闭 intrinsics 时该偏移不生效。
`T_i_b` 为单位阵。标定文件和数值来自 `kalibr_ws/calib-imu.yaml`。

`imu_accel_axis` 和 `imu_gyro_axis` 分别控制加速度计和陀螺仪的轴向：

- 加速度计：Betaflight 输出 NWU（x 前、y 左、z 上），已经和 ROS FLU
  一致，默认 `imu_accel_axis=none`。只有原始数据确实是 FRD 时才需要
  `frd_to_flu`（把 y、z 取反）。
- 陀螺仪：当前实机原始数据已经是 FLU。实测机头向左偏航时
  `angular_velocity.z ≈ +0.43 rad/s`，向右偏航为负，因此默认
  `imu_gyro_axis=none`，不做翻转。
- 如果某些飞控的偏航是顺时针为正（左偏航为负），把 `imu_gyro_axis` 设为
  `bf_to_flu`，只翻转 z；如果原始数据的 y 也相反（真正的 FRD 陀螺仪），
  设为 `frd_to_flu`，同时翻转 y、z。
- 当前实机静止时 `linear_acceleration.z ≈ +9.8 m/s²`。可以用下面的动作
  快速检查陀螺仪：机体水平绕竖直轴向左（俯视逆时针）转，
  `angular_velocity.z` 应为正；抬头时 `angular_velocity.y` 应为负；
  右侧下沉（向右滚转）时 `angular_velocity.x` 应为正。
  更完整的坐标链和调试步骤见
  [`../odometry/coordinates.md`](../odometry/coordinates.md)。

这两个参数只做轴向映射，不包含安装角旋转；飞控板或 IMU 相对机体的安装角在
`IMU.T_b_c1` 中处理，尺度/轴失准在 `imu_intrinsics.yaml` 中处理。

## 4. 话题

```text
/flyrat/sensors/imu
/flyrat/sensors/tof_0
/flyrat/sensors/tof_1
/flyrat/sensors/tof_2
/flyrat/sensors/tof_3
/flyrat/sensors/camera/image_raw
```

- IMU：`sensor_msgs/msg/Imu`，线加速度为 m/s²、角速度为 rad/s，orientation 暂无，
  协方差首项为 `-1`。`imu_frame_id` 默认 `imu_link`。
- ToF：每颗一个 `flyrat_sensors_interfaces/msg/TofFrame`，`zones[64]`，
  `valid = target_status in {5, 9}`。
- 单目相机：`sensor_msgs/msg/Image`，`mono8`，默认 640x480，请求 60 Hz。

## 5. 验收

```sh
ros2 topic hz /flyrat/sensors/imu
ros2 topic hz /flyrat/sensors/tof_0
ros2 topic hz /flyrat/sensors/camera/image_raw
ros2 topic echo --once /flyrat/sensors/tof_0
```

预期：

- IMU 约 200 Hz；静止时 `linear_acceleration.z` 约 `+9.8 m/s²`；
- 每颗 ToF 约 15 Hz，`TofFrame.resolution=8`（8×8）；若显式使用
  `--tof-resolution 4 --tof-rate 60`，则约 57–58 Hz、`resolution=4`；
- 单目图像 `640x480`、`mono8`；2026-11-08 主机实测约 30 Hz（请求 60 Hz，
  受 UVC/adb/解码链路限制）。

2026-11-08 设备端 `tof_bench` 实测（10 s，8×8@15）：

| Sensor | 帧数 | 实测频率 | 读取错误 | 平均读取 |
|---:|---:|---:|---:|---:|
| 0（下） | 151 | 15.1 Hz | 0 | 4.66 ms |
| 1（前） | 152 | 15.2 Hz | 0 | 4.68 ms |
| 2（右） | 150 | 15.0 Hz | 0 | 4.69 ms |
| 3（左） | 148 | 14.8 Hz | 0 | 4.68 ms |

主机侧 `ros2 topic hz` 在 TIO 推理负载下可能读到约 13 Hz，这是测量进程丢消息，
不是传感器实际频率；以设备端 `tof_bench` 或节点 `processed_frames` 为准。

拔掉 adb 后 receiver 会每秒重连；重新执行 `adb forward` 后恢复。

## 6. 已知限制

- ToF 地址是易失的，掉电后会回到 `0x52`；当前的启动探测会自动恢复并改回固定地址。
  LPn 拉低只关闭 I2C，不复位地址。
- 四颗 ToF 初始化约 7.2 秒，forwarder 在这段时间内还不能提供 ToF 数据；
  IMU 和单目相机也会等 `start()` 完成，因此从启动到监听约 7 秒。
- 相机是原生 MJPEG，有损；需要原始像素时应改用低帧率原始格式。
- 设备时间戳和主机时间戳通过 `TIME` 帧做偏移估计，不是硬实时同步。
- MJPEG 解码在开发机完成，设备端 CPU 只负责采集和发送。
- 相机、ToF 和 IMU 没有硬件同步，跨传感器时间戳只能按到达时间/设备时间近似对齐。
