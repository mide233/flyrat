// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/msp_frame.hpp"
#include "../src/msp_parse.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

std::vector<std::uint8_t> response(
    std::uint8_t cmd, const std::vector<std::uint8_t>& payload)
{
    std::vector<std::uint8_t> frame { '$', 'M', '>' };
    frame.push_back(static_cast<std::uint8_t>(payload.size()));
    frame.push_back(cmd);
    std::uint8_t checksum = static_cast<std::uint8_t>(payload.size()) ^ cmd;
    for (std::uint8_t byte : payload) {
        frame.push_back(byte);
        checksum ^= byte;
    }
    frame.push_back(checksum);
    return frame;
}

void test_round_trip()
{
    const auto encoded = flyrat::detail::encode_request(
        flyrat::MspCommand::kRawImu, nullptr, 0);
    check(encoded.size() == 6, "request size");
    check(encoded[0] == '$' && encoded[1] == 'M' && encoded[2] == '<', "request header");
    check(encoded[3] == 0 && encoded[4] == 102, "request size/cmd");

    flyrat::detail::FrameParser parser;
    const auto frame = response(102, { 1, 2, 3 });
    parser.append(frame.data(), frame.size());
    flyrat::detail::Frame parsed;
    check(parser.next(parsed), "parse response");
    check(parsed.cmd == 102 && parsed.direction == '>', "response header");
    check(parsed.payload == std::vector<std::uint8_t>({ 1, 2, 3 }), "response payload");
    check(!parser.next(parsed), "no second frame");
}

void test_split_and_resync()
{
    flyrat::detail::FrameParser parser;
    const auto frame = response(108, { 0, 1, 0, 2, 0, 3 });
    parser.append(frame.data(), 3);
    flyrat::detail::Frame parsed;
    check(!parser.next(parsed), "incomplete frame");
    parser.append(frame.data() + 3, frame.size() - 3);
    check(parser.next(parsed), "completed frame");
    check(parsed.cmd == 108, "completed cmd");

    const std::uint8_t junk[] = { 0x00, 0x11, 0x22 };
    parser.append(junk, sizeof(junk));
    check(!parser.next(parsed), "junk waits");

    const auto good = response(1, { 0, 1, 46 });
    parser.append(good.data(), good.size());
    check(parser.next(parsed), "resync after junk");
    check(parsed.cmd == 1 && parsed.payload.size() == 3, "resync payload");
}

void test_bad_checksum()
{
    flyrat::detail::FrameParser parser;
    auto frame = response(102, { 1, 2, 3, 4 });
    frame.back() ^= 0xFF;
    parser.append(frame.data(), frame.size());

    const auto good = response(108, { 0, 0, 0, 0, 0, 0 });
    parser.append(good.data(), good.size());

    flyrat::detail::Frame parsed;
    check(parser.next(parsed), "parse after bad checksum");
    check(parsed.cmd == 108, "bad frame dropped");
    check(parser.checksum_errors() >= 1, "checksum counted");
}

void test_parsers()
{
    flyrat::Attitude attitude;
    check(flyrat::detail::parse_attitude({ 0x10, 0x00, 0xF0, 0xFF, 0x02, 0x00 }, attitude),
        "attitude parse");
    check(attitude.roll_deg == 1.6 && attitude.pitch_deg == -1.6 && attitude.yaw_deg == 2.0,
        "attitude values");

    flyrat::ImuSample imu;
    std::vector<std::uint8_t> imu_payload(18, 0);
    imu_payload[0] = 1;
    imu_payload[6] = 2;
    imu_payload[12] = 3;
    check(flyrat::detail::parse_raw_imu(imu_payload, imu), "imu parse");
    check(imu.acc[0] == 1 && imu.gyro[0] == 2 && imu.mag[0] == 3, "imu values");

    flyrat::StatusFrame status;
    std::vector<std::uint8_t> status_payload(16, 0);
    status_payload[6] = 0x01; // ARM box (box 0) active.
    check(flyrat::detail::parse_status(status_payload, true, status), "status parse");
    check(status.armed, "status armed");

    flyrat::GpsFrame gps;
    std::vector<std::uint8_t> gps_payload(22, 0);
    gps_payload[0] = 3;
    gps_payload[1] = 12;
    check(flyrat::detail::parse_raw_gps(gps_payload, gps), "gps parse");
    check(gps.fix == 3 && gps.satellites == 12, "gps values");

    auto raw_rc_payload = [](std::uint8_t version, std::uint8_t count,
                              std::uint8_t flags, std::uint16_t mask,
                              std::uint16_t age) {
        std::vector<std::uint8_t> payload {
            version, count, flags, 0,
            static_cast<std::uint8_t>(mask & 0xFF),
            static_cast<std::uint8_t>((mask >> 8) & 0xFF),
            static_cast<std::uint8_t>(age & 0xFF),
            static_cast<std::uint8_t>((age >> 8) & 0xFF),
        };
        for (std::uint8_t i = 0; i < count; ++i) {
            const std::uint16_t raw = static_cast<std::uint16_t>(1000 + i);
            payload.push_back(static_cast<std::uint8_t>(raw & 0xFF));
            payload.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
        }
        for (std::uint8_t i = 0; i < count; ++i) {
            const std::uint16_t actual = static_cast<std::uint16_t>(2000 + i);
            payload.push_back(static_cast<std::uint8_t>(actual & 0xFF));
            payload.push_back(static_cast<std::uint8_t>((actual >> 8) & 0xFF));
        }
        return payload;
    };

    const auto raw_payload = raw_rc_payload(1, 16, 0x03, 0x000F, 42);
    check(raw_payload.size() == 72, "raw rc payload size");
    flyrat::RawRcFrame raw_rc;
    check(flyrat::detail::parse_raw_rc(raw_payload, raw_rc), "raw rc parse");
    check(raw_rc.active && raw_rc.valid, "raw rc flags");
    check(raw_rc.mask == 0x000F && raw_rc.age_ms == 42, "raw rc meta");
    check(raw_rc.raw[0] == 1000 && raw_rc.raw[15] == 1015, "raw rc raw channels");
    check(raw_rc.actual[0] == 2000 && raw_rc.actual[15] == 2015, "raw rc actual channels");

    auto truncated = raw_payload;
    truncated.pop_back();
    check(!flyrat::detail::parse_raw_rc(truncated, raw_rc), "raw rc truncated");

    check(!flyrat::detail::parse_raw_rc(raw_rc_payload(2, 16, 0, 0, 0), raw_rc),
        "raw rc bad version");
    check(!flyrat::detail::parse_raw_rc(raw_rc_payload(1, 0, 0, 0, 0), raw_rc),
        "raw rc zero count");
    check(!flyrat::detail::parse_raw_rc(raw_rc_payload(1, 17, 0, 0, 0), raw_rc),
        "raw rc count too large");
}

} // namespace

int main()
{
    test_round_trip();
    test_split_and_resync();
    test_bad_checksum();
    test_parsers();

    if (g_failures != 0) {
        std::printf("%d framing check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("framing tests passed\n");
    return 0;
}
