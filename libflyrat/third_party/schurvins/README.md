# SchurVINS (vendored)

This directory contains the ROS-independent core of
[SchurVINS](https://github.com/bytedance/SchurVINS), a stereo-inertial visual
odometry system (CVPR 2024) built on SVO 2.0.

- Upstream commit: `d8ab6dff20860694d8d879d716d7fd6cb2c2530e`
- License: **GPL-3.0** (see `LICENSE`). Linking this code into
  `flyrat_odometry` makes the combined binary subject to the GPL-3.0.
- `minkindr/include` comes from
  [ethz-asl/minkindr](https://github.com/ethz-asl/minkindr) commit
  `564f12639a8447d4d3e5e7707851424302941056` (BSD-3-Clause).
- `fast` comes from [uzh-rpg/fast](https://github.com/uzh-rpg/fast) commit
  `1153981a7d0263909e998789a5b5029412c28f5a` (BSD-2-Clause).

Only the libraries needed by `svo::FrameHandlerStereo` are kept: `svo`,
`svo_common`, `svo_direct`, `svo_img_align`, `svo_tracker`, `svo_vio_common`,
`vikit` (`vikit_common`, `vikit_cameras`, `vikit_solver`), `rpg_common`,
`minkindr` and `fast`. ROS 1, catkin, Ceres-dependent and GPU code are excluded.

## Local modifications

1. `svo/include/svo/frame_handler_base.h`: removed
   `#include <vikit/params_helper.h>`. That header lives in `vikit_ros` and
   pulls in ROS, but the core never uses it.
2. `svo_common/include/svo/common/logging.h`: `SVO_DEBUG_STREAM` is compiled out
   unless `SVO_ENABLE_DEBUG_LOGGING` is defined. The upstream frontend logs
   several lines per frame, which is unusable in the ROS node. Configure with
   `-DFLYRAT_SCHURVINS_VERBOSE=ON` to restore it.
3. `svo_common/src/frame.cpp`: implemented the shrinking branch of
   `Frame::resizeFeatureStorage()`. Upstream logs
   `Downsizing storage not implemented` whenever fewer features survive than
   the preallocated capacity, which happens on almost every frame.
4. `svo/src/frame_handler_base.cpp`: the "updated seeds are unconverged"
   warning is throttled to once per 100 reports. It is expected on close-range
   or low-parallax sequences and was otherwise printed for every keyframe.
5. `svo/include/svo/initialization.h` and `svo/src/initialization.cpp`:
   `StereoInit` now honours near-field depth limits through new
   `InitializationOptions::stereo_{mean,min,max}_depth_inv` fields. Upstream
   always used `StereoTriangulationOptions` defaults (1-50 m), which made
   close-range desk scenes fail with "Not enough triangulated points".
6. `svo/src/schur_vins.cpp`: removed the unused `opencv2/optflow.hpp` include;
   it belongs to opencv_contrib and is not needed by the filter.
7. `svo_direct/src/feature_detection_utils.cpp`: fixed the non-SSE FAST
   fallback to index `img_pyr[level]` instead of the undefined `img_pyr[L]`.
8. `fast/src/faster_corner_10_sse.cpp` is only compiled on x86; the file is
   x86 SSE code and cannot be built for the aarch64 target.

The adapter that parses the flyrat calibration and drives the frontend lives in
`libflyrat/src/odometry/schurvins_backend.cpp`.
