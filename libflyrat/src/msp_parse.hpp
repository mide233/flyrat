// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <vector>

#include "flyrat/msp/types.hpp"

namespace flyrat::detail {

bool parse_api_version(const std::vector<std::uint8_t>& payload, ApiVersion& out);
bool parse_attitude(const std::vector<std::uint8_t>& payload, Attitude& out);
bool parse_rc(const std::vector<std::uint8_t>& payload, RcFrame& out);
bool parse_raw_rc(const std::vector<std::uint8_t>& payload, RawRcFrame& out);
bool parse_motor(const std::vector<std::uint8_t>& payload, MotorFrame& out);
bool parse_raw_imu(const std::vector<std::uint8_t>& payload, ImuSample& out);
bool parse_status(
    const std::vector<std::uint8_t>& payload, bool extended, StatusFrame& out);
bool parse_analog(const std::vector<std::uint8_t>& payload, AnalogFrame& out);
bool parse_altitude(const std::vector<std::uint8_t>& payload, AltitudeFrame& out);
bool parse_debug(const std::vector<std::uint8_t>& payload, DebugFrame& out);
bool parse_raw_gps(const std::vector<std::uint8_t>& payload, GpsFrame& out);

} // namespace flyrat::detail
