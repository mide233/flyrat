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

The intrinsics and IMU noise values come from the existing Flyrat stereo/IMU
calibration.  `body_T_cam1` is
`body_T_cam0 * Stereo.T_c1_c2`, where `Stereo.T_c1_c2` maps the right camera
into the left camera frame.  If the camera rig or IMU is recalibrated, update
both camera files, both `body_T_cam*` matrices and the noise values together.

## Tuning

- `multiple_thread` is forced to 0 by the library adapter for deterministic
  synchronous tracking.
- `max_cnt` / `min_dist` control the feature budget.  Lower `max_cnt` reduces
  CPU load; raise it if tracking is fragile.
- `max_solver_time` and `max_num_iterations` bound the sliding-window solve.
- `acc_n`, `gyr_n`, `acc_w`, `gyr_w` are the continuous-time IMU noise densities
  and random walks.  They should match the IMU calibration.
- `estimate_td` is 0 because `sensor_receiver_node` already applies the IMU time
  offset from `imu_intrinsics.yaml`.
