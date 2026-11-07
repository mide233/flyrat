# VINS-Fusion configuration

`uvc_640x480_imu.yaml` is the stereo-inertial configuration for the Flyrat UVC
stereo module.  It expects 640x480 grayscale images per eye and IMU samples in
seconds, m/s^2 and rad/s.

## Conventions

- `body_T_cam0` / `body_T_cam1` map camera coordinates into the IMU/body frame
  (`p_body = body_T_cam * p_camera`).  They are the poses of the cameras in the
  body frame, which is also what the ROS static TF `imu_link -> camera_*`
  broadcasts.
- The camera frames are standard optical frames: x right, y down, z forward.
- The IMU/body frame is FLU: x forward, y left, z up.
- VINS defines the world frame during initialization.  It is gravity aligned,
  but the initial yaw is arbitrary.

## Calibration provenance

The intrinsics, extrinsics and IMU random walks come from the existing Flyrat
stereo/IMU calibration.  `body_T_cam1` is
`body_T_cam0 * Stereo.T_c1_c2`, where `Stereo.T_c1_c2` maps the right camera
into the left camera frame.  If the camera rig or IMU is recalibrated, update
both camera files, both `body_T_cam*` matrices and the noise values together.

The `acc_n` / `gyr_n` values are **not** the stationary Allan-variance floor.
The bench floor for this IMU is about `0.003` m/s^2/sqrt(Hz) and
`8.8e-05` rad/s/sqrt(Hz), but motor vibration and unmodelled dynamics make the
in-flight IMU much noisier.  With the bench values VINS treats the IMU factor as
almost exact, ignores the stereo constraints once the vehicle starts moving and
the solution can fly away.  The shipped values are the VINS-Fusion EuRoC
baseline (`0.08` / `0.004`) and provide a realistic flight margin.

## Tuning

- `multiple_thread` is forced to 0 by the library adapter for deterministic
  synchronous tracking.
- `max_cnt` / `min_dist` control the feature budget.  Lower `max_cnt` reduces
  CPU load; raise it if tracking is fragile.
- `max_solver_time` and `max_num_iterations` bound the sliding-window solve.
- `acc_n`, `gyr_n` are the continuous-time IMU white-noise densities used by
  the preintegration covariance.  `acc_w`, `gyr_w` are the bias random walks.
  The random walks match the Allan calibration; the white-noise terms include
  the flight vibration margin described above.  Do not lower `acc_n` / `gyr_n`
  to the bench Allan floor just because the vehicle is stationary on a desk.
- `estimate_td` is 0 because `sensor_receiver_node` already applies the IMU time
  offset from `imu_intrinsics.yaml`.
