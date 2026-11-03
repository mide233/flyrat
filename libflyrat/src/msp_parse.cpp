// SPDX-License-Identifier: MIT
#include "msp_parse.hpp"

namespace flyrat::detail {
namespace {

bool read_u8(const std::vector<std::uint8_t>& p, std::size_t offset, std::uint8_t& value)
{
    if (offset + 1U > p.size()) {
        return false;
    }
    value = p[offset];
    return true;
}

bool read_u16(const std::vector<std::uint8_t>& p, std::size_t offset, std::uint16_t& value)
{
    if (offset + 2U > p.size()) {
        return false;
    }
    value = static_cast<std::uint16_t>(p[offset])
        | static_cast<std::uint16_t>(p[offset + 1U] << 8);
    return true;
}

bool read_i16(const std::vector<std::uint8_t>& p, std::size_t offset, std::int16_t& value)
{
    std::uint16_t raw = 0;
    if (!read_u16(p, offset, raw)) {
        return false;
    }
    value = static_cast<std::int16_t>(raw);
    return true;
}

bool read_u32(const std::vector<std::uint8_t>& p, std::size_t offset, std::uint32_t& value)
{
    if (offset + 4U > p.size()) {
        return false;
    }
    value = static_cast<std::uint32_t>(p[offset])
        | (static_cast<std::uint32_t>(p[offset + 1U]) << 8)
        | (static_cast<std::uint32_t>(p[offset + 2U]) << 16)
        | (static_cast<std::uint32_t>(p[offset + 3U]) << 24);
    return true;
}

bool read_i32(const std::vector<std::uint8_t>& p, std::size_t offset, std::int32_t& value)
{
    std::uint32_t raw = 0;
    if (!read_u32(p, offset, raw)) {
        return false;
    }
    value = static_cast<std::int32_t>(raw);
    return true;
}

bool box_active(const std::vector<std::uint8_t>& p, std::uint16_t box)
{
    if (box < 32U) {
        std::uint32_t flags = 0;
        if (!read_u32(p, 6, flags)) {
            return false;
        }
        return (flags & (1U << box)) != 0U;
    }

    const std::size_t index = box - 32U;
    const std::size_t offset = 16U + index / 8U;
    if (offset >= p.size()) {
        return false;
    }
    return (p[offset] & (1U << (index % 8U))) != 0U;
}

} // namespace

bool parse_api_version(const std::vector<std::uint8_t>& payload, ApiVersion& out)
{
    if (payload.size() < 3U) {
        return false;
    }
    out.protocol = payload[0];
    out.api_major = payload[1];
    out.api_minor = payload[2];
    return true;
}

bool parse_attitude(const std::vector<std::uint8_t>& payload, Attitude& out)
{
    std::int16_t roll = 0;
    std::int16_t pitch = 0;
    std::int16_t yaw = 0;
    if (!read_i16(payload, 0, roll) || !read_i16(payload, 2, pitch)
        || !read_i16(payload, 4, yaw)) {
        return false;
    }
    out.roll_deg = roll / 10.0;
    out.pitch_deg = pitch / 10.0;
    out.yaw_deg = yaw > 180 ? yaw - 360 : yaw;
    return true;
}

bool parse_rc(const std::vector<std::uint8_t>& payload, RcFrame& out)
{
    if (payload.size() < 4U || (payload.size() % 2U) != 0U) {
        return false;
    }
    out.channels.resize(payload.size() / 2U);
    for (std::size_t i = 0; i < out.channels.size(); ++i) {
        out.channels[i] = static_cast<std::uint16_t>(payload[2U * i])
            | static_cast<std::uint16_t>(payload[2U * i + 1U] << 8);
    }
    return true;
}

bool parse_motor(const std::vector<std::uint8_t>& payload, MotorFrame& out)
{
    if ((payload.size() % 2U) != 0U) {
        return false;
    }
    out.motors.resize(payload.size() / 2U);
    for (std::size_t i = 0; i < out.motors.size(); ++i) {
        out.motors[i] = static_cast<std::uint16_t>(payload[2U * i])
            | static_cast<std::uint16_t>(payload[2U * i + 1U] << 8);
    }
    return true;
}

bool parse_raw_imu(const std::vector<std::uint8_t>& payload, ImuSample& out)
{
    if (payload.size() < 18U) {
        return false;
    }
    for (std::size_t i = 0; i < 3U; ++i) {
        if (!read_i16(payload, 2U * i, out.acc[i])
            || !read_i16(payload, 6U + 2U * i, out.gyro[i])
            || !read_i16(payload, 12U + 2U * i, out.mag[i])) {
            return false;
        }
    }
    return true;
}

bool parse_status(
    const std::vector<std::uint8_t>& payload, bool extended, StatusFrame& out)
{
    if (payload.size() < 8U) {
        return false;
    }
    if (!read_u16(payload, 0, out.cycletime) || !read_u16(payload, 2, out.i2c_errors)
        || !read_u16(payload, 4, out.sensors)
        || !read_u32(payload, 6, out.flight_mode_flags)) {
        return false;
    }
    out.armed = box_active(payload, static_cast<std::uint16_t>(BoxId::kBoxArm));
    out.msp_override = box_active(payload, static_cast<std::uint16_t>(BoxId::kBoxMspOverride));
    out.arming_disable_flags = 0;
    if (extended && payload.size() >= 12U) {
        read_u32(payload, payload.size() - 4U, out.arming_disable_flags);
    }
    return true;
}

bool parse_analog(const std::vector<std::uint8_t>& payload, AnalogFrame& out)
{
    if (payload.size() < 8U) {
        return false;
    }
    out.voltage_v = payload[0] / 10.0;
    out.mah = payload[1];
    out.rssi = payload[3];
    std::uint16_t amperage = 0;
    if (!read_u16(payload, 5, amperage)) {
        return false;
    }
    out.amperage_a = amperage / 100.0;
    if (payload.size() >= 9U) {
        std::uint16_t voltage = 0;
        if (read_u16(payload, 7, voltage)) {
            out.voltage_v = voltage / 100.0;
        }
    }
    return true;
}

bool parse_altitude(const std::vector<std::uint8_t>& payload, AltitudeFrame& out)
{
    std::uint32_t centimeters = 0;
    std::int16_t vario = 0;
    if (!read_u32(payload, 0, centimeters) || !read_i16(payload, 4, vario)) {
        return false;
    }
    out.valid = centimeters < 0xFFFFFF00U;
    out.altitude_m = centimeters / 100.0;
    out.vario_m_s = vario / 100.0;
    return true;
}

bool parse_debug(const std::vector<std::uint8_t>& payload, DebugFrame& out)
{
    if (payload.size() < 16U) {
        return false;
    }
    for (std::size_t i = 0; i < out.value.size(); ++i) {
        if (!read_i16(payload, 2U * i, out.value[i])) {
            return false;
        }
    }
    return true;
}

bool parse_raw_gps(const std::vector<std::uint8_t>& payload, GpsFrame& out)
{
    if (payload.size() < 16U) {
        return false;
    }

    std::int32_t latitude = 0;
    std::int32_t longitude = 0;
    std::int16_t altitude = 0;
    std::int16_t speed = 0;
    std::uint16_t course = 0;
    if (!read_u8(payload, 0, out.fix) || !read_u8(payload, 1, out.satellites)
        || !read_i32(payload, 2, latitude) || !read_i32(payload, 6, longitude)
        || !read_i16(payload, 10, altitude) || !read_i16(payload, 12, speed)
        || !read_u16(payload, 14, course)) {
        return false;
    }

    out.lat_deg = latitude / 1e7;
    out.lon_deg = longitude / 1e7;
    out.altitude_m = altitude;
    out.ground_speed_mps = speed / 100.0;
    out.ground_course_deg = course / 10.0;
    out.pdop = 0.0;
    out.hdop = 0.0;
    out.vdop = 0.0;

    std::uint16_t dop = 0;
    if (read_u16(payload, 16, dop)) {
        out.pdop = dop / 100.0;
    }
    if (read_u16(payload, 18, dop)) {
        out.hdop = dop / 100.0;
    }
    if (read_u16(payload, 20, dop)) {
        out.vdop = dop / 100.0;
    }
    return true;
}

} // namespace flyrat::detail
