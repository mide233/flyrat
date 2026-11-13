// SPDX-License-Identifier: MIT
//
// Small clock helpers shared by the onboard programs and the sensor stream.
// Timestamps inside the device pipeline use steady_clock so they are immune to
// wall-clock jumps; system_now_ns() is only for bag metadata and epoch mapping.

#pragma once

#include <chrono>
#include <cstdint>

namespace flyrat {

using SteadyClock = std::chrono::steady_clock;

inline std::int64_t to_ns(SteadyClock::time_point stamp)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        stamp.time_since_epoch()).count();
}

inline std::int64_t steady_now_ns()
{
    return to_ns(SteadyClock::now());
}

inline std::int64_t system_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

} // namespace flyrat
