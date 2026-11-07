// SPDX-License-Identifier: GPL-3.0-or-later
//
// Minimal ROS-free logging shim for the vendored VINS-Fusion core.
//
// Upstream VINS-Fusion uses ROS_INFO/ROS_WARN/ROS_ASSERT throughout the
// estimator.  The Flyrat build links the estimator as a plain library, so this
// header provides the same macros without pulling in roscpp.

#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <iostream>

namespace flyrat_vins {

inline void log(const char* level, const char* format, ...)
{
    std::fprintf(stderr, "[vins][%s] ", level);
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);
    std::fputc('\n', stderr);
}

inline void assert_failed(const char* expression, const char* file, int line,
    const char* format, ...)
{
    std::fprintf(stderr, "[vins][assert] %s:%d: %s\n", file, line, expression);
    if (format != nullptr && format[0] != '\0') {
        std::fprintf(stderr, "[vins][assert] ");
        va_list args;
        va_start(args, format);
        std::vfprintf(stderr, format, args);
        va_end(args);
        std::fputc('\n', stderr);
    }
    std::abort();
}

} // namespace flyrat_vins

#define ROS_INFO(...) ::flyrat_vins::log("info", __VA_ARGS__)
#define ROS_WARN(...) ::flyrat_vins::log("warn", __VA_ARGS__)
#define ROS_ERROR(...) ::flyrat_vins::log("error", __VA_ARGS__)
#define ROS_INFO_STREAM(message) \
    do { std::cerr << "[vins][info] " << message << std::endl; } while (0)
#define ROS_WARN_STREAM(message) \
    do { std::cerr << "[vins][warn] " << message << std::endl; } while (0)
#define ROS_ERROR_STREAM(message) \
    do { std::cerr << "[vins][error] " << message << std::endl; } while (0)

#ifdef VINS_DEBUG
#define ROS_DEBUG(...) ::flyrat_vins::log("debug", __VA_ARGS__)
#define ROS_DEBUG_STREAM(message) \
    do { std::cerr << "[vins][debug] " << message << std::endl; } while (0)
#else
#define ROS_DEBUG(...) do { } while (0)
#define ROS_DEBUG_STREAM(message) do { } while (0)
#endif

#define ROS_ASSERT(expression) \
    do { \
        if (!(expression)) { \
            ::flyrat_vins::assert_failed(#expression, __FILE__, __LINE__, ""); \
        } \
    } while (0)

#define ROS_ASSERT_MSG(expression, ...) \
    do { \
        if (!(expression)) { \
            ::flyrat_vins::assert_failed(#expression, __FILE__, __LINE__, __VA_ARGS__); \
        } \
    } while (0)

#define ROS_BREAK() std::abort()
