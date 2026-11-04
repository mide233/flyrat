// SPDX-License-Identifier: MIT
//
// Wire protocol shared by the real-vehicle sensor forwarder and the ROS 2
// receiver. Everything is explicitly little-endian; do not memcpy C++ structs
// onto the socket because padding and endianness would make the two sides
// diverge.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace flyrat::sensors {

constexpr std::uint32_t kMagic = 0x46525331U; // "FRS1"
constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kHeaderSize = 28;
constexpr std::size_t kMaxPayload = 8U * 1024U * 1024U;

enum class MessageType : std::uint16_t {
    kTime = 1,
    kImu = 2,
    kTof = 3,
    kStereoJpeg = 4,
    kStats = 5,
};

struct Header {
    std::uint32_t magic = kMagic;
    std::uint16_t version = kVersion;
    MessageType type = MessageType::kTime;
    std::uint32_t payload_size = 0;
    std::uint64_t sequence = 0;
    std::int64_t device_time_ns = 0;
};

struct ImuPayload {
    std::array<std::int16_t, 3> acc {};
    std::array<std::int16_t, 3> gyro {};
    std::array<std::int16_t, 3> mag {};
};

struct TofZoneWire {
    std::int16_t distance_mm = 0;
    std::uint8_t target_status = 0;
    std::uint8_t nb_target = 0;
};

struct TofPayload {
    std::uint8_t sensor_id = 0;
    std::uint8_t resolution = 4;
    std::int8_t temperature_c = 0;
    std::uint8_t reserved = 0;
    std::array<TofZoneWire, 64> zones {};
};

struct StereoJpegView {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    const std::uint8_t* jpeg = nullptr;
    std::size_t size = 0;
};

inline void put_u16(std::vector<std::uint8_t>& out, std::uint16_t value)
{
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

inline void put_u32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

inline void put_u64(std::vector<std::uint8_t>& out, std::uint64_t value)
{
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>((value >> (i * 8)) & 0xFFU));
    }
}

inline void put_i16(std::vector<std::uint8_t>& out, std::int16_t value)
{
    put_u16(out, static_cast<std::uint16_t>(value));
}

inline void put_i64(std::vector<std::uint8_t>& out, std::int64_t value)
{
    put_u64(out, static_cast<std::uint64_t>(value));
}

inline std::uint16_t get_u16(const std::uint8_t* data)
{
    return static_cast<std::uint16_t>(data[0]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

inline std::uint32_t get_u32(const std::uint8_t* data)
{
    return static_cast<std::uint32_t>(data[0]) |
        (static_cast<std::uint32_t>(data[1]) << 8U) |
        (static_cast<std::uint32_t>(data[2]) << 16U) |
        (static_cast<std::uint32_t>(data[3]) << 24U);
}

inline std::uint64_t get_u64(const std::uint8_t* data)
{
    std::uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<std::uint64_t>(data[i]) << (i * 8);
    }
    return value;
}

inline std::int16_t get_i16(const std::uint8_t* data)
{
    return static_cast<std::int16_t>(get_u16(data));
}

inline std::int64_t get_i64(const std::uint8_t* data)
{
    return static_cast<std::int64_t>(get_u64(data));
}

inline std::vector<std::uint8_t> encode_header(const Header& header)
{
    std::vector<std::uint8_t> out;
    out.reserve(kHeaderSize);
    put_u32(out, header.magic);
    put_u16(out, header.version);
    put_u16(out, static_cast<std::uint16_t>(header.type));
    put_u32(out, header.payload_size);
    put_u64(out, header.sequence);
    put_i64(out, header.device_time_ns);
    return out;
}

inline bool decode_header(const std::uint8_t* data, std::size_t size, Header& header)
{
    if (size < kHeaderSize) {
        return false;
    }
    header.magic = get_u32(data);
    header.version = get_u16(data + 4);
    header.type = static_cast<MessageType>(get_u16(data + 6));
    header.payload_size = get_u32(data + 8);
    header.sequence = get_u64(data + 12);
    header.device_time_ns = get_i64(data + 20);
    return header.magic == kMagic && header.version == kVersion &&
        header.payload_size <= kMaxPayload;
}

inline std::vector<std::uint8_t> encode_frame(MessageType type, std::uint64_t sequence,
    std::int64_t device_time_ns, const std::vector<std::uint8_t>& payload)
{
    Header header;
    header.type = type;
    header.payload_size = static_cast<std::uint32_t>(payload.size());
    header.sequence = sequence;
    header.device_time_ns = device_time_ns;
    std::vector<std::uint8_t> out = encode_header(header);
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

inline std::vector<std::uint8_t> encode_imu(const ImuPayload& imu)
{
    std::vector<std::uint8_t> out;
    out.reserve(18);
    for (std::int16_t value : imu.acc) {
        put_i16(out, value);
    }
    for (std::int16_t value : imu.gyro) {
        put_i16(out, value);
    }
    for (std::int16_t value : imu.mag) {
        put_i16(out, value);
    }
    return out;
}

inline bool decode_imu(const std::uint8_t* data, std::size_t size, ImuPayload& imu)
{
    if (size != 18) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        imu.acc[static_cast<std::size_t>(i)] = get_i16(data + i * 2);
    }
    for (int i = 0; i < 3; ++i) {
        imu.gyro[static_cast<std::size_t>(i)] = get_i16(data + 6 + i * 2);
    }
    for (int i = 0; i < 3; ++i) {
        imu.mag[static_cast<std::size_t>(i)] = get_i16(data + 12 + i * 2);
    }
    return true;
}

inline std::vector<std::uint8_t> encode_tof(const TofPayload& tof)
{
    std::vector<std::uint8_t> out;
    out.reserve(4 + 64 * 4);
    out.push_back(tof.sensor_id);
    out.push_back(tof.resolution);
    out.push_back(static_cast<std::uint8_t>(tof.temperature_c));
    out.push_back(tof.reserved);
    for (const TofZoneWire& zone : tof.zones) {
        put_i16(out, zone.distance_mm);
        out.push_back(zone.target_status);
        out.push_back(zone.nb_target);
    }
    return out;
}

inline bool decode_tof(const std::uint8_t* data, std::size_t size, TofPayload& tof)
{
    if (size != 4 + 64 * 4) {
        return false;
    }
    tof.sensor_id = data[0];
    tof.resolution = data[1];
    tof.temperature_c = static_cast<std::int8_t>(data[2]);
    tof.reserved = data[3];
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint8_t* zone = data + 4 + i * 4;
        tof.zones[i].distance_mm = get_i16(zone);
        tof.zones[i].target_status = zone[2];
        tof.zones[i].nb_target = zone[3];
    }
    return true;
}

inline std::vector<std::uint8_t> encode_stereo_jpeg(std::uint32_t width,
    std::uint32_t height, const std::uint8_t* jpeg, std::size_t jpeg_size)
{
    std::vector<std::uint8_t> out;
    out.reserve(8 + jpeg_size);
    put_u32(out, width);
    put_u32(out, height);
    if (jpeg != nullptr && jpeg_size != 0) {
        out.insert(out.end(), jpeg, jpeg + jpeg_size);
    }
    return out;
}

inline bool decode_stereo_jpeg(const std::uint8_t* data, std::size_t size,
    StereoJpegView& view)
{
    if (size < 8) {
        return false;
    }
    view.width = get_u32(data);
    view.height = get_u32(data + 4);
    view.jpeg = data + 8;
    view.size = size - 8;
    return view.width != 0 && view.height != 0 && view.size != 0;
}

} // namespace flyrat::sensors
