# SPDX-License-Identifier: GPL-3.0-or-later
#
# ROS-free build of the VINS-Fusion estimator.  See README.md for the upstream
# commit and the small set of portability patches applied to the vendored code.

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

set(VINS_ESTIMATOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vins_estimator/src")
set(CAMERA_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/camera_models")

add_library(flyrat_vins_core STATIC
  "${VINS_ESTIMATOR_DIR}/estimator/estimator.cpp"
  "${VINS_ESTIMATOR_DIR}/estimator/parameters.cpp"
  "${VINS_ESTIMATOR_DIR}/estimator/feature_manager.cpp"
  "${VINS_ESTIMATOR_DIR}/featureTracker/feature_tracker.cpp"
  "${VINS_ESTIMATOR_DIR}/factor/marginalization_factor.cpp"
  "${VINS_ESTIMATOR_DIR}/factor/pose_local_parameterization.cpp"
  "${VINS_ESTIMATOR_DIR}/factor/projectionOneFrameTwoCamFactor.cpp"
  "${VINS_ESTIMATOR_DIR}/factor/projectionTwoFrameOneCamFactor.cpp"
  "${VINS_ESTIMATOR_DIR}/factor/projectionTwoFrameTwoCamFactor.cpp"
  "${VINS_ESTIMATOR_DIR}/factor/projection_factor.cpp"
  "${VINS_ESTIMATOR_DIR}/initial/initial_aligment.cpp"
  "${VINS_ESTIMATOR_DIR}/initial/initial_ex_rotation.cpp"
  "${VINS_ESTIMATOR_DIR}/initial/initial_sfm.cpp"
  "${VINS_ESTIMATOR_DIR}/initial/solve_5pts.cpp"
  "${VINS_ESTIMATOR_DIR}/utility/utility.cpp"
  "${CAMERA_MODELS_DIR}/src/camera_models/Camera.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/CameraFactory.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/CataCamera.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/CostFunctionFactory.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/EquidistantCamera.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/PinholeCamera.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/PinholeFullCamera.cc"
  "${CAMERA_MODELS_DIR}/src/camera_models/ScaramuzzaCamera.cc"
  "${CAMERA_MODELS_DIR}/src/gpl/gpl.cc"
  "${CAMERA_MODELS_DIR}/src/sparse_graph/Transform.cc"
)

target_include_directories(flyrat_vins_core PUBLIC
  "${VINS_ESTIMATOR_DIR}"
  "${CAMERA_MODELS_DIR}/include"
)

target_link_libraries(flyrat_vins_core PUBLIC
  ${OpenCV_LIBS}
  Ceres::ceres
  Eigen3::Eigen
  Boost::boost
  Threads::Threads
)

target_compile_features(flyrat_vins_core PUBLIC cxx_std_17)
target_compile_options(flyrat_vins_core PRIVATE -O3)
