// SPDX-License-Identifier: MIT
//
// MSP telemetry parsers. The transport lives in msp_client.cpp; keeping
// the wire format separate makes both files small enough to read.
#include "flyrat/msp_client.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "msp_codec.hpp"

namespace flyrat {

using namespace codec;

ApiVersion MspClient::api_version() {
    const auto p = request(kApiVersion, {}, -1, 16);
    if (p.size() < 3)
        throw MspError("short MSP_API_VERSION");
    return ApiVersion{p[0], p[1], p[2]};
}

Attitude MspClient::attitude() {
    const auto p = request(kAttitude, {}, -1, 16);
    Attitude a;
    a.roll_deg = read_i16(p, 0, "MSP_ATTITUDE") / 10.0;
    a.pitch_deg = read_i16(p, 2, "MSP_ATTITUDE") / 10.0;
    // Betaflight reports yaw in whole degrees, unsigned, in [0, 360). Put it
    // on the same [-180, 180) convention as the other two axes so a caller can
    // subtract headings without a 2*pi jump at north.
    a.yaw_deg = read_i16(p, 4, "MSP_ATTITUDE");
    if (a.yaw_deg > 180.0)
        a.yaw_deg -= 360.0;
    return a;
}

RcFrame MspClient::rc() {
    const auto p = request(kRc, {}, -1, 16);
    RcFrame f;
    for (size_t o = 0; o + 2 <= p.size(); o += 2) {
        f.channels.push_back(read_u16(p, o, "MSP_RC"));
    }
    return f;
}

MotorFrame MspClient::motor() {
    const auto p = request(kMotor, {}, -1, 16);
    MotorFrame f;
    for (size_t o = 0; o + 2 <= p.size(); o += 2) {
        f.motors.push_back(read_u16(p, o, "MSP_MOTOR"));
    }
    return f;
}

ImuSample MspClient::raw_imu() {
    const auto p = request(kRawImu, {}, -1, 16);
    ImuSample s;
    for (int i = 0; i < 3; ++i) {
        s.acc[i] = read_i16(p, 2 * i, "MSP_RAW_IMU");
        s.gyro[i] = read_i16(p, 6 + 2 * i, "MSP_RAW_IMU");
        s.mag[i] = read_i16(p, 12 + 2 * i, "MSP_RAW_IMU");
    }
    return s;
}

StatusFrame MspClient::status(bool extended) {
    const auto p = request(extended ? kStatusEx : kStatus, {}, -1, 16);
    StatusFrame s;
    s.cycletime = read_u16(p, 0, "MSP_STATUS");
    s.i2c_errors = read_u16(p, 2, "MSP_STATUS");
    s.sensors = read_u16(p, 4, "MSP_STATUS");
    s.flight_mode_flags = read_u32(p, 6, "MSP_STATUS");
    s.armed = box_active(p, kBoxArm);
    s.msp_override = box_active(p, kBoxMspOverride);
    if (extended && p.size() >= 4)
        s.arming_disable_flags = read_u32(p, p.size() - 4, "MSP_STATUS_EX");
    return s;
}

AnalogFrame MspClient::analog() {
    const auto p = request(kAnalog, {}, -1, 16);
    if (p.size() < 9)
        throw MspError("short MSP_ANALOG");
    AnalogFrame a;
    a.voltage_v = p[0] / 10.0;
    a.mah = read_u16(p, 1, "MSP_ANALOG");
    a.rssi = read_u16(p, 3, "MSP_ANALOG");
    a.amperage_a = read_u16(p, 5, "MSP_ANALOG") / 100.0;
    a.voltage_v = read_u16(p, 7, "MSP_ANALOG") / 100.0;
    return a;
}

AltitudeFrame MspClient::altitude() {
    const auto p = request(kAltitude, {}, -1, 16);
    AltitudeFrame a;
    const uint32_t cm = read_u32(p, 0, "MSP_ALTITUDE");
    a.vario_m_s = read_i16(p, 4, "MSP_ALTITUDE") / 100.0;
    // The SITL reports 0xFFFFFFFx when it has no estimate.
    a.valid = cm < 0xFFFFFF00u;
    a.altitude_m = a.valid ? cm / 100.0 : 0.0;
    return a;
}

DebugFrame MspClient::debug() {
    const auto p = request(kDebug, {}, -1, 16);
    DebugFrame f;
    for (size_t i = 0; i < f.value.size() && (i + 1) * 2 <= p.size(); ++i)
        f.value[i] = read_i16(p, i * 2, "MSP_DEBUG");
    return f;
}

GpsFrame MspClient::raw_gps() {
    const auto p = request(kRawGps, {}, -1, 16);
    if (p.size() < 16)
        throw MspError("short MSP_RAW_GPS");
    GpsFrame g;
    g.fix = p[0] != 0;
    g.satellites = p[1];
    // Latitude/longitude are signed but travel as u32 two's complement.
    g.lat_deg = static_cast<int32_t>(read_u32(p, 2, "MSP_RAW_GPS")) / 1e7;
    g.lon_deg = static_cast<int32_t>(read_u32(p, 6, "MSP_RAW_GPS")) / 1e7;
    g.altitude_m = read_u16(p, 10, "MSP_RAW_GPS");
    g.ground_speed_mps = read_u16(p, 12, "MSP_RAW_GPS") / 100.0;
    g.ground_course_deg = read_u16(p, 14, "MSP_RAW_GPS") / 10.0;
    // DOP fields were appended over several MSP API revisions; tolerate short
    // replies from older firmware.
    if (p.size() >= 18)
        g.pdop = read_u16(p, 16, "MSP_RAW_GPS") / 100.0;
    if (p.size() >= 20)
        g.hdop = read_u16(p, 18, "MSP_RAW_GPS") / 100.0;
    if (p.size() >= 22)
        g.vdop = read_u16(p, 20, "MSP_RAW_GPS") / 100.0;
    return g;
}

bool MspClient::is_armed() { return status(true).armed; }

bool MspClient::is_msp_override_active() { return status(true).msp_override; }

bool MspClient::is_box_active(uint16_t box) {
    const auto p = request(kStatusEx, {}, -1, 16);
    return box_active(p, box);
}

} // namespace flyrat
