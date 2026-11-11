# SPDX-License-Identifier: MIT
#
# Cross-compile toolchain for the Taishan Pi (RK3566, aarch64).
# The sysroot is an Ubuntu 24.04 arm64 rootfs containing static OpenCV,
# yaml-cpp and glog. Override with -DFLYRAT_SYSROOT=/path/to/sysroot.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(FLYRAT_SYSROOT "/opt/sysroots/arm64" CACHE PATH "aarch64 sysroot")
set(CMAKE_SYSROOT "${FLYRAT_SYSROOT}")

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_AR aarch64-linux-gnu-ar)
set(CMAKE_RANLIB aarch64-linux-gnu-ranlib)
set(CMAKE_STRIP aarch64-linux-gnu-strip)

set(CMAKE_FIND_ROOT_PATH "${FLYRAT_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
