// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "flyrat/gps.hpp"

namespace flyrat {

// MSP v1 command identifiers used by flyrat.
enum class MspCommand : std::uint8_t {
    kApiVersion = 1,
    kStatus = 101,
    kRawImu = 102,
    kMotor = 104,
    kRc = 105,
    kRawGps = 106,
    kAttitude = 108,
    kAltitude = 109,
    kAnalog = 110,
    kStatusEx = 150,
    kAttitudeQuaternion = 167,
    kSetRawRc = 200,
    kRawRcOriginal = 243,
    kDebug = 254,
};

enum class BoxId : std::uint16_t {
    kBoxArm = 0,
    kBoxMspOverride = 50,
};

struct ApiVersion {
    std::uint8_t protocol = 0;
    std::uint8_t api_major = 0;
    std::uint8_t api_minor = 0;
};

struct Attitude {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
};

struct RcFrame {
    std::vector<std::uint16_t> channels;
};

// Pre-override RC channels reported by newer Betaflight firmware. `raw` holds
// what the receiver delivered, `actual` what the controller used after the MSP
// override. Both arrays are indexed like the MSP channel order (AETR1234 by
// default). The firmware only refreshes the snapshot while BOXMSPOVERRIDE is
// active, so `valid` can be false while the override is disarmed.
struct RawRcFrame {
    std::array<std::uint16_t, 16> raw {};
    std::array<std::uint16_t, 16> actual {};
    std::uint16_t mask = 0;
    std::uint16_t age_ms = 0;
    bool active = false;
    bool valid = false;
};

struct MotorFrame {
    std::vector<std::uint16_t> motors;
};

struct ImuSample {
    std::array<std::int16_t, 3> acc {};
    std::array<std::int16_t, 3> gyro {};
    std::array<std::int16_t, 3> mag {};
};

struct StatusFrame {
    std::uint16_t cycletime = 0;
    std::uint16_t i2c_errors = 0;
    std::uint16_t sensors = 0;
    std::uint32_t flight_mode_flags = 0;
    bool armed = false;
    bool msp_override = false;
    std::uint32_t arming_disable_flags = 0;
};

struct AnalogFrame {
    double voltage_v = 0.0;
    double amperage_a = 0.0;
    std::uint16_t mah = 0;
    std::uint8_t rssi = 0;
};

struct AltitudeFrame {
    double altitude_m = 0.0;
    double vario_m_s = 0.0;
    bool valid = false;
};

struct DebugFrame {
    std::array<std::int16_t, 8> value {};
};

class MspError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

} // namespace flyrat
