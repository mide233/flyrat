// SPDX-License-Identifier: MIT
//
// Wire protocol round-trip and malformed-header checks shared by the vehicle
// forwarder and the ROS 2 receiver.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "flyrat/sensors/protocol.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

} // namespace

int main()
{
    using namespace flyrat::sensors;

    check(kHeaderSize == 28, "header size");
    check(kMaxPayload == 8U * 1024U * 1024U, "max payload");

    Header header;
    header.type = MessageType::kCameraJpeg;
    header.payload_size = 1234;
    header.sequence = 0x1122334455667788ULL;
    header.device_time_ns = -123456789;
    const std::vector<std::uint8_t> header_bytes = encode_header(header);
    check(header_bytes.size() == kHeaderSize, "encoded header size");

    Header decoded;
    check(decode_header(header_bytes.data(), header_bytes.size(), decoded),
        "decode valid header");
    check(decoded.type == MessageType::kCameraJpeg, "header type");
    check(decoded.payload_size == 1234, "header payload size");
    check(decoded.sequence == 0x1122334455667788ULL, "header sequence");
    check(decoded.device_time_ns == -123456789, "header signed time");

    std::vector<std::uint8_t> bad_magic = header_bytes;
    bad_magic[0] = 0;
    check(!decode_header(bad_magic.data(), bad_magic.size(), decoded),
        "reject bad magic");

    std::vector<std::uint8_t> bad_version = header_bytes;
    bad_version[4] = 2;
    check(!decode_header(bad_version.data(), bad_version.size(), decoded),
        "reject bad version");

    std::vector<std::uint8_t> too_large = header_bytes;
    const std::uint32_t oversized = static_cast<std::uint32_t>(kMaxPayload + 1);
    too_large[8] = static_cast<std::uint8_t>(oversized & 0xFFU);
    too_large[9] = static_cast<std::uint8_t>((oversized >> 8U) & 0xFFU);
    too_large[10] = static_cast<std::uint8_t>((oversized >> 16U) & 0xFFU);
    too_large[11] = static_cast<std::uint8_t>((oversized >> 24U) & 0xFFU);
    check(!decode_header(too_large.data(), too_large.size(), decoded),
        "reject oversized payload");

    check(!decode_header(header_bytes.data(), kHeaderSize - 1, decoded),
        "reject truncated header");

    const std::vector<std::uint8_t> imu_payload = encode_imu([] {
        ImuPayload imu;
        imu.acc = { 1, -2, 300 };
        imu.gyro = { -4, 5, -600 };
        imu.mag = { 7, -8, 900 };
        return imu;
    }());
    check(imu_payload.size() == 18, "IMU payload size");
    ImuPayload imu;
    check(decode_imu(imu_payload.data(), imu_payload.size(), imu), "decode IMU");
    check(imu.acc[1] == -2 && imu.gyro[2] == -600 && imu.mag[2] == 900,
        "IMU signed values");
    check(!decode_imu(imu_payload.data(), imu_payload.size() - 1, imu),
        "reject short IMU");

    TofPayload tof;
    tof.sensor_id = 2;
    tof.resolution = 4;
    tof.temperature_c = -12;
    tof.zones[0] = { 1234, 5, 1 };
    tof.zones[63] = { -1, 255, 0 };
    const std::vector<std::uint8_t> tof_payload = encode_tof(tof);
    check(tof_payload.size() == 4 + 64 * 4, "ToF payload size");
    TofPayload decoded_tof;
    check(decode_tof(tof_payload.data(), tof_payload.size(), decoded_tof),
        "decode ToF");
    check(decoded_tof.sensor_id == 2 && decoded_tof.resolution == 4,
        "ToF header fields");
    check(decoded_tof.temperature_c == -12, "ToF signed temperature");
    check(decoded_tof.zones[0].distance_mm == 1234 &&
            decoded_tof.zones[0].target_status == 5 &&
            decoded_tof.zones[0].nb_target == 1,
        "ToF first zone");
    check(decoded_tof.zones[63].distance_mm == -1 &&
            decoded_tof.zones[63].target_status == 255,
        "ToF last zone");
    check(!decode_tof(tof_payload.data(), tof_payload.size() - 1, decoded_tof),
        "reject short ToF");

    const std::uint8_t jpeg[] = { 0xFF, 0xD8, 0x00, 0x11, 0xFF, 0xD9 };
    const std::vector<std::uint8_t> camera_payload = encode_jpeg(
        640, 480, jpeg, sizeof(jpeg));
    check(camera_payload.size() == 8 + sizeof(jpeg), "camera payload size");
    JpegView view;
    check(decode_jpeg(camera_payload.data(), camera_payload.size(), view),
        "decode camera header");
    check(view.width == 640 && view.height == 480, "camera dimensions");
    check(view.size == sizeof(jpeg) && view.jpeg[0] == 0xFF && view.jpeg[1] == 0xD8,
        "camera JPEG bytes");
    check(!decode_jpeg(camera_payload.data(), 7, view),
        "reject short camera header");

    const std::vector<std::uint8_t> frame = encode_frame(
        MessageType::kImu, 42, 987654321, imu_payload);
    check(frame.size() == kHeaderSize + imu_payload.size(), "frame size");
    Header frame_header;
    check(decode_header(frame.data(), frame.size(), frame_header), "decode frame header");
    check(frame_header.type == MessageType::kImu &&
            frame_header.sequence == 42 &&
            frame_header.device_time_ns == 987654321,
        "frame header values");
    ImuPayload frame_imu;
    check(decode_imu(frame.data() + kHeaderSize,
              frame.size() - kHeaderSize, frame_imu),
        "decode frame payload");

    if (g_failures != 0) {
        std::printf("%d sensor protocol check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("sensor protocol checks passed\n");
    return 0;
}
