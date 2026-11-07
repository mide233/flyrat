# VINS-Fusion (vendored)

This directory contains the runtime subset of
[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion), pinned to
upstream commit `be55a937a57436548ddfb1bd324bc1e9a9e828e0`.

Upstream is licensed under the GNU General Public License v3.0.  The full
license text is in `LICENSE`.

## What is included

- `vins_estimator/src/estimator`
- `vins_estimator/src/featureTracker`
- `vins_estimator/src/factor`
- `vins_estimator/src/initial`
- `vins_estimator/src/utility` (without the ROS visualization module)
- `camera_models` runtime sources and headers

The ROS 1 node (`rosNodeTest.cpp`), visualization publishers, calibration
applications, chessboard assets and test data are intentionally excluded.

## Portability patches

The following changes were made so the estimator can be built as a plain C++
library with OpenCV 4 and Ceres 2.2:

- Removed ROS 1 headers, parameters, publishers and topic/output-path handling.
  `vins_estimator/src/utility/vins_log.hpp` provides the `ROS_*` logging and
  assertion macros used by the estimator.
- Configuration parsing now throws on missing/invalid files, resolves camera
  calibration paths relative to the main configuration file with
  `std::filesystem`, clears the process-global parameter vectors before each
  read, and validates the image dimensions.
- Initialized the estimator's pre-integration and marginalization pointers
  before `clearState()` and guarded the worker-thread join.
- Added empty-buffer checks to `getIMUInterval()` and kept a feature queued when
  the IMU interval is not yet available.
- Replaced `ceres::LocalParameterization` with `ceres::Manifold` for the pose
  parameter block, and `ceres::QuaternionParameterization` with
  `ceres::QuaternionManifold` in the SFM initializer.
- Removed the unused `EigenQuaternionParameterization` source from the build.
- Removed per-frame ROS visualization calls and noisy per-frame prints.

Algorithmic code is otherwise kept as close to upstream as possible.
