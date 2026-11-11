# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. .script/export_compile_commands.sh merges the per-project databases
# into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat VERSION 0.1.0 LANGUAGES C CXX)

# libflyrat is the ROS-free core shared by the simulator and the real machine:
# the autopilot algorithms plus the Betaflight MSP / UDP transport. It builds as
# a standalone CMake project (cmake -B build && cmake --build build && ctest
# --test-dir build) and can also be dropped into a ROS 2 package via
# add_subdirectory().
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(FLYRAT_STANDALONE ON)
else()
  set(FLYRAT_STANDALONE OFF)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

# Eigen for the manifold maths, OSQP for the QP the MPC solves every cycle.
# The pilot/SLAM runtime does not use the MPC, so it can disable this core and
# avoid pulling OSQP into the cross build.
find_package(Eigen3 REQUIRED NO_MODULE)
option(FLYRAT_BUILD_CORE "Build the attitude/MPC core (requires OSQP)" ON)
if(FLYRAT_BUILD_CORE)
  find_package(osqp CONFIG QUIET)
  if(NOT osqp_FOUND)
    # Fall back to a plain system install (libosqp-dev, Homebrew, or a build
    # pointed at by OSQP_ROOT). Keeps libflyrat usable outside a ROS workspace.
    find_path(FLYRAT_OSQP_INCLUDE_DIR osqp/osqp.h)
    find_library(FLYRAT_OSQP_LIBRARY NAMES osqp)
    if(NOT FLYRAT_OSQP_INCLUDE_DIR OR NOT FLYRAT_OSQP_LIBRARY)
      message(FATAL_ERROR "OSQP not found: install libosqp-dev or set OSQP_ROOT")
    endif()
  endif()

  add_library(flyrat
    src/attitude_hold.cpp
    src/local_estimator.cpp
    src/ommpc.cpp
    src/rc_receiver.cpp)
  add_library(flyrat::flyrat ALIAS flyrat)

  target_include_directories(flyrat PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  if(osqp_FOUND)
    target_link_libraries(flyrat PUBLIC Threads::Threads Eigen3::Eigen osqp::osqp)
  else()
    target_include_directories(flyrat PUBLIC ${FLYRAT_OSQP_INCLUDE_DIR})
    target_link_libraries(flyrat PUBLIC Threads::Threads Eigen3::Eigen ${FLYRAT_OSQP_LIBRARY})
  endif()
  target_compile_options(flyrat PRIVATE -Wall -Wextra)
endif()

# Odometry interface. With SchurVINS enabled this is a stereo-inertial VIO
# backend; without it the stub validates the calibration and reports
# NOT_INITIALIZED so the ROS pipeline can still be developed.
#
# SchurVINS is GPL-3.0. Linking it into flyrat_odometry makes the combined
# binary subject to the GPL; libflyrat's own sources remain MIT.
option(FLYRAT_WITH_SCHURVINS "Build the SchurVINS stereo-inertial odometry backend" ON)
option(FLYRAT_SCHURVINS_VERBOSE "Enable the very chatty SVO debug traces" OFF)

if(FLYRAT_WITH_SCHURVINS)
  option(FLYRAT_STATIC_DEPS
    "Use static OpenCV/glog/yaml-cpp from CMAKE_SYSROOT (cross builds)" OFF)
  if(FLYRAT_STATIC_DEPS)
    set(FLYRAT_STATIC_PREFIX "${CMAKE_SYSROOT}/usr/local" CACHE PATH
      "Prefix of the minimal static OpenCV build")
    set(_flyrat_libdir "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu")
    set(OpenCV_DIR "${FLYRAT_STATIC_PREFIX}/lib/cmake/opencv4" CACHE PATH "" FORCE)
    set(OpenCV_STATIC ON)
    find_package(OpenCV CONFIG QUIET COMPONENTS
      core imgproc imgcodecs highgui calib3d video)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR
        "Static OpenCV not found under ${FLYRAT_STATIC_PREFIX}. Build OpenCV "
        "with BUILD_SHARED_LIBS=OFF and install it there.")
    endif()

    add_library(gflags STATIC IMPORTED GLOBAL)
    set_target_properties(gflags PROPERTIES
      IMPORTED_LOCATION "${_flyrat_libdir}/libgflags.a"
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SYSROOT}/usr/include")
    add_library(glog::glog STATIC IMPORTED GLOBAL)
    set_target_properties(glog::glog PROPERTIES
      IMPORTED_LOCATION "${_flyrat_libdir}/libglog.a"
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SYSROOT}/usr/include"
      INTERFACE_LINK_LIBRARIES gflags)
    add_library(yaml-cpp STATIC IMPORTED GLOBAL)
    set_target_properties(yaml-cpp PROPERTIES
      IMPORTED_LOCATION "${_flyrat_libdir}/libyaml-cpp.a"
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SYSROOT}/usr/include")
    set(yaml-cpp_FOUND TRUE)
    set(glog_FOUND TRUE)
    # Visible to the parent project (flyrat_pilot) as well.
    set(FLYRAT_OPENCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}" CACHE INTERNAL "")
    set(FLYRAT_OPENCV_LIBS "${OpenCV_LIBS}" CACHE INTERNAL "")
  else()
    find_package(OpenCV QUIET COMPONENTS
      core imgproc imgcodecs highgui calib3d video)
    find_package(yaml-cpp QUIET)
    find_package(glog QUIET)
  endif()
  if(NOT OpenCV_FOUND OR NOT yaml-cpp_FOUND OR NOT glog_FOUND)
    message(FATAL_ERROR
      "FLYRAT_WITH_SCHURVINS=ON needs OpenCV (core/imgproc/imgcodecs/highgui/video), "
      "yaml-cpp and glog. Install them or configure with "
      "-DFLYRAT_WITH_SCHURVINS=OFF.")
  endif()

  set(SCHURVINS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/schurvins)
  file(GLOB SCHURVINS_SOURCES
    ${SCHURVINS_DIR}/svo/src/*.cpp
    ${SCHURVINS_DIR}/svo_common/src/*.cpp
    ${SCHURVINS_DIR}/svo_direct/src/*.cpp
    ${SCHURVINS_DIR}/svo_img_align/src/sparse_img_align.cpp
    ${SCHURVINS_DIR}/svo_img_align/src/sparse_img_align_base.cpp
    ${SCHURVINS_DIR}/svo_tracker/src/*.cpp
    ${SCHURVINS_DIR}/svo_vio_common/src/vio_common.cpp
    ${SCHURVINS_DIR}/vikit/vikit_common/src/*.cpp
    ${SCHURVINS_DIR}/vikit/vikit_cameras/src/*.cpp
    ${SCHURVINS_DIR}/vikit/vikit_solver/src/*.cpp
    ${SCHURVINS_DIR}/fast/src/fast_10.cpp
    ${SCHURVINS_DIR}/fast/src/fast_10_score.cpp
    ${SCHURVINS_DIR}/fast/src/nonmax_3x3.cpp)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i.86")
    list(APPEND SCHURVINS_SOURCES
      ${SCHURVINS_DIR}/fast/src/faster_corner_10_sse.cpp)
  endif()

  add_library(flyrat_schurvins STATIC ${SCHURVINS_SOURCES})
  target_include_directories(flyrat_schurvins PUBLIC
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/svo/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/svo_common/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/svo_direct/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/svo_img_align/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/svo_tracker/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/svo_vio_common/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/vikit/vikit_common/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/vikit/vikit_cameras/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/vikit/vikit_solver/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/rpg_common/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/minkindr/include>
    $<BUILD_INTERFACE:${SCHURVINS_DIR}/fast/include>
    $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>)
  # USE_SCHUR_VINS selects the EKF/Schur-complement path in the SVO frontend.
  # svo/global.h defines it for every translation unit; redefining it here
  # would only add a warning.
  # The vendored code predates modern warning hygiene; keep its warnings out of
  # the flyrat build log.
  target_compile_options(flyrat_schurvins PRIVATE -w)
  if(FLYRAT_SCHURVINS_VERBOSE)
    target_compile_definitions(flyrat_schurvins PRIVATE SVO_ENABLE_DEBUG_LOGGING)
  endif()
  target_link_libraries(flyrat_schurvins PUBLIC
    ${OpenCV_LIBS} yaml-cpp glog::glog Threads::Threads Eigen3::Eigen)
endif()

add_library(flyrat_odometry src/odometry/odometry.cpp)
add_library(flyrat::odometry ALIAS flyrat_odometry)
target_include_directories(flyrat_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flyrat_odometry PUBLIC Threads::Threads Eigen3::Eigen)
target_compile_options(flyrat_odometry PRIVATE -Wall -Wextra)
if(FLYRAT_WITH_SCHURVINS)
  target_sources(flyrat_odometry PRIVATE
    src/odometry/schurvins_backend.cpp)
  target_compile_definitions(flyrat_odometry PRIVATE
    FLYRAT_ODOMETRY_WITH_SCHURVINS)
  target_link_libraries(flyrat_odometry PRIVATE flyrat_schurvins)
endif()

# ST VL53L5CX user-space driver (ULD). The output is trimmed to the blocks the
# flight stack needs: distance, target status, target count and temperature.
# Four sensors on one 750 kHz bus cannot afford the full 532-byte result block.
add_library(flyrat_vl53l5cx STATIC
  third_party/vl53l5cx/src/vl53l5cx_api.c
  third_party/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.c
  third_party/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.c
  third_party/vl53l5cx/src/vl53l5cx_plugin_xtalk.c
  third_party/vl53l5cx/platform/platform.c)
target_include_directories(flyrat_vl53l5cx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/vl53l5cx/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/vl53l5cx/platform>)
# The VL53L5CX output-selection macros live in platform/platform.h, not here.
# They change the layout of VL53L5CX_ResultsData, so they must be identical for
# the C driver and every C++ consumer; defining them per-target broke that.

# Hardware access lives in its own target so the protocol layer can depend on
# the header-only Transport interface without pulling in a particular backend.
add_library(flyrat_hardware
  src/hardware/mjpeg_decoder.cpp
  src/hardware/serial_transport.cpp
  src/hardware/stereo_camera.cpp
  src/hardware/stereo_jpeg_source.cpp
  src/hardware/tcp_transport.cpp
  src/hardware/tof.cpp)
add_library(flyrat::flyrat_hardware ALIAS flyrat_hardware)
target_include_directories(flyrat_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flyrat_hardware PUBLIC Threads::Threads)
target_link_libraries(flyrat_hardware PRIVATE flyrat_vl53l5cx ${CMAKE_DL_LIBS})
target_compile_options(flyrat_hardware PRIVATE -Wall -Wextra)

# MSP protocol and scheduling. It only depends on the header-only Transport
# interface, so it can be tested with a fake transport and stays free of
# Eigen/OSQP.
add_library(flyrat_msp_interact
  src/msp_frame.cpp
  src/msp_parse.cpp
  src/msp_interact.cpp)
add_library(flyrat::flyrat_msp_interact ALIAS flyrat_msp_interact)
target_include_directories(flyrat_msp_interact PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flyrat_msp_interact PUBLIC Threads::Threads)
target_compile_options(flyrat_msp_interact PRIVATE -Wall -Wextra)

# The IMU facade reads MspInteract snapshots, so hardware depends on the
# protocol layer (not the other way around).
target_link_libraries(flyrat_hardware PUBLIC flyrat_msp_interact)

set(FLYRAT_INSTALL_TARGETS
  flyrat_odometry flyrat_hardware flyrat_msp_interact flyrat_vl53l5cx)
if(FLYRAT_BUILD_CORE)
  list(APPEND FLYRAT_INSTALL_TARGETS flyrat)
endif()
install(TARGETS ${FLYRAT_INSTALL_TARGETS}
  EXPORT flyrat_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
if(FLYRAT_WITH_SCHURVINS)
  install(TARGETS flyrat_schurvins
    EXPORT flyrat_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
  install(FILES third_party/schurvins/LICENSE third_party/schurvins/README.md
    DESTINATION share/doc/flyrat)
endif()
install(DIRECTORY include/ DESTINATION include)
install(FILES third_party/vl53l5cx/LICENSE DESTINATION share/doc/flyrat)
if(FLYRAT_STANDALONE)
  install(EXPORT flyrat_targets
    NAMESPACE flyrat:: DESTINATION lib/cmake/flyrat)
endif()

# Tests are only built when this project owns the build (standalone) or when a
# parent explicitly enables them. Downstream ROS packages skip them by default.
if(FLYRAT_STANDALONE)
  set(FLYRAT_BUILD_TESTS ON)
endif()
option(FLYRAT_BUILD_TESTS "Build libflyrat unit tests" OFF)
if(FLYRAT_STANDALONE)
  option(FLYRAT_BUILD_TOOLS "Build libflyrat command line tools" ON)
else()
  option(FLYRAT_BUILD_TOOLS "Build libflyrat command line tools" OFF)
endif()

if(FLYRAT_BUILD_TOOLS)
  add_executable(msp_interact_bench tools/msp_interact_bench.cpp)
  target_link_libraries(msp_interact_bench PRIVATE flyrat_msp_interact flyrat_hardware)
  target_compile_options(msp_interact_bench PRIVATE -Wall -Wextra)

  add_executable(tof_bench tools/tof_bench.cpp)
  target_link_libraries(tof_bench PRIVATE flyrat_hardware)
  target_compile_options(tof_bench PRIVATE -Wall -Wextra)

  add_executable(stereo_bench tools/stereo_bench.cpp)
  target_link_libraries(stereo_bench PRIVATE flyrat_hardware)
  target_compile_options(stereo_bench PRIVATE -Wall -Wextra)
endif()

if(FLYRAT_BUILD_TESTS)
  enable_testing()
  add_executable(test_attitude_hold tests/test_attitude_hold.cpp)
  target_link_libraries(test_attitude_hold PRIVATE flyrat)
  target_compile_options(test_attitude_hold PRIVATE -Wall -Wextra)
  add_test(NAME test_attitude_hold COMMAND test_attitude_hold)

  add_executable(test_ommpc tests/test_ommpc.cpp)
  target_link_libraries(test_ommpc PRIVATE flyrat)
  target_compile_options(test_ommpc PRIVATE -Wall -Wextra)
  add_test(NAME test_ommpc COMMAND test_ommpc)

  add_executable(test_local_estimator tests/test_local_estimator.cpp)
  target_link_libraries(test_local_estimator PRIVATE flyrat)
  target_compile_options(test_local_estimator PRIVATE -Wall -Wextra)
  add_test(NAME test_local_estimator COMMAND test_local_estimator)

  add_executable(test_transport tests/test_transport.cpp)
  target_link_libraries(test_transport PRIVATE flyrat_hardware)
  target_compile_options(test_transport PRIVATE -Wall -Wextra)
  # openpty() lives in libutil on glibc < 2.34.
  find_library(FLYRAT_UTIL_LIBRARY util)
  if(FLYRAT_UTIL_LIBRARY)
    target_link_libraries(test_transport PRIVATE ${FLYRAT_UTIL_LIBRARY})
  endif()
  add_test(NAME test_transport COMMAND test_transport)

  add_executable(test_msp_framing tests/test_msp_framing.cpp)
  target_link_libraries(test_msp_framing PRIVATE flyrat_msp_interact)
  target_compile_options(test_msp_framing PRIVATE -Wall -Wextra)
  add_test(NAME test_msp_framing COMMAND test_msp_framing)

  add_executable(test_msp_interact tests/test_msp_interact.cpp)
  target_link_libraries(test_msp_interact PRIVATE flyrat_msp_interact)
  target_compile_options(test_msp_interact PRIVATE -Wall -Wextra)
  add_test(NAME test_msp_interact COMMAND test_msp_interact)

  add_executable(test_hardware_headers tests/test_hardware_headers.cpp)
  target_link_libraries(test_hardware_headers PRIVATE flyrat_hardware)
  target_compile_options(test_hardware_headers PRIVATE -Wall -Wextra)
  add_test(NAME test_hardware_headers COMMAND test_hardware_headers)

  add_executable(test_sensor_protocol tests/test_sensor_protocol.cpp)
  target_include_directories(test_sensor_protocol PRIVATE include)
  target_compile_options(test_sensor_protocol PRIVATE -Wall -Wextra)
  add_test(NAME test_sensor_protocol COMMAND test_sensor_protocol)

  add_executable(test_odometry_api tests/test_odometry_api.cpp)
  target_link_libraries(test_odometry_api PRIVATE flyrat_odometry)
  target_compile_options(test_odometry_api PRIVATE -Wall -Wextra)
  target_compile_definitions(test_odometry_api PRIVATE
    FLYRAT_TEST_CAMERA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/configs/camera/uvc_640x480.yaml")
  if(FLYRAT_WITH_SCHURVINS)
    target_compile_definitions(test_odometry_api PRIVATE FLYRAT_TEST_SCHURVINS=1)
  endif()
  add_test(NAME test_odometry_api COMMAND test_odometry_api)

  if(FLYRAT_WITH_SCHURVINS)
    add_executable(test_schurvins_integration tests/test_schurvins_integration.cpp)
    target_link_libraries(test_schurvins_integration PRIVATE flyrat_odometry)
    target_include_directories(test_schurvins_integration PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_compile_options(test_schurvins_integration PRIVATE -Wall -Wextra)
    target_compile_definitions(test_schurvins_integration PRIVATE
      FLYRAT_TEST_CAMERA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/configs/camera/uvc_640x480.yaml")
    add_test(NAME test_schurvins_integration COMMAND test_schurvins_integration)
  endif()
endif()
