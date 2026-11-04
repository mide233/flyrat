# SPDX-License-Identifier: GPL-3.0-or-later
#
# Vendored ORB-SLAM3 core, built headless as a static library.
#
# The upstream project builds a shared library and a Pangolin viewer. This
# build keeps only the tracking/mapping pipeline: Viewer, MapDrawer and the
# Pangolin dependency are removed. FrameDrawer is retained because it produces
# the tracking image and only needs OpenCV.
#
# This directory is added by libflyrat/CMakeLists.txt when FLYRAT_BUILD_SLAM is
# enabled, so it does not call project() or cmake_minimum_required() itself.

find_package(OpenCV REQUIRED COMPONENTS
  core imgproc features2d calib3d imgcodecs videoio)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

# DBoW2 / DUtils, the place-recognition vocabulary backend.
add_library(flyrat_orb_dbow2 STATIC
  Thirdparty/DBoW2/DBoW2/BowVector.cpp
  Thirdparty/DBoW2/DBoW2/FORB.cpp
  Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
  Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
  Thirdparty/DBoW2/DUtils/Random.cpp
  Thirdparty/DBoW2/DUtils/Timestamp.cpp)
target_include_directories(flyrat_orb_dbow2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flyrat_orb_dbow2 PUBLIC ${OpenCV_LIBS})
target_compile_options(flyrat_orb_dbow2 PRIVATE -Wall -Wextra)

# g2o. Its headers include "../../config.h" relative to g2o/core, so the
# generated file has to sit at Thirdparty/g2o/config.h. It is git-ignored.
configure_file(Thirdparty/g2o/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/config.h)

add_library(flyrat_orb_g2o STATIC
  Thirdparty/g2o/g2o/types/types_sba.cpp
  Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
  Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
  Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
  Thirdparty/g2o/g2o/core/hyper_graph.cpp
  Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
  Thirdparty/g2o/g2o/core/matrix_structure.cpp
  Thirdparty/g2o/g2o/core/batch_stats.cpp
  Thirdparty/g2o/g2o/core/parameter.cpp
  Thirdparty/g2o/g2o/core/cache.cpp
  Thirdparty/g2o/g2o/core/optimizable_graph.cpp
  Thirdparty/g2o/g2o/core/solver.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
  Thirdparty/g2o/g2o/core/estimate_propagator.cpp
  Thirdparty/g2o/g2o/core/factory.cpp
  Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
  Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
  Thirdparty/g2o/g2o/core/parameter_container.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
  Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
  Thirdparty/g2o/g2o/core/robust_kernel.cpp
  Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
  Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
  Thirdparty/g2o/g2o/stuff/timeutil.cpp
  Thirdparty/g2o/g2o/stuff/os_specific.c
  Thirdparty/g2o/g2o/stuff/string_tools.cpp
  Thirdparty/g2o/g2o/stuff/property.cpp)
target_include_directories(flyrat_orb_g2o PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o
  ${EIGEN3_INCLUDE_DIR})
target_compile_options(flyrat_orb_g2o PRIVATE -Wall -Wextra)

# Sophus is header-only.
add_library(flyrat_orb_sophus INTERFACE)
target_include_directories(flyrat_orb_sophus INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus)

# ORB-SLAM3 tracking and mapping core. Viewer.cc and MapDrawer.cc are omitted.
add_library(flyrat_orb_core STATIC
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc)

target_include_directories(flyrat_orb_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(flyrat_orb_core PUBLIC
  flyrat_orb_dbow2
  flyrat_orb_g2o
  flyrat_orb_sophus
  ${OpenCV_LIBS}
  Eigen3::Eigen
  OpenSSL::Crypto
  Boost::serialization)
target_compile_options(flyrat_orb_core PRIVATE -Wall -Wextra)

add_library(flyrat::orb_slam3 ALIAS flyrat_orb_core)
