// SPDX-License-Identifier: MIT
//
// Minimal MSP v1 client for the Betaflight SITL UART.
//
// The SITL exposes UART1 as a TCP socket (default 127.0.0.1:5761) that speaks
// the same MSP v1 framing as the real serial link:
//   request  $M< size cmd payload... xor
//   response $M> size cmd payload... xor
//   error    $M! ...
// where size counts the payload bytes only and xor covers size..payload.
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace flyrat_msp {

enum MspCommand : uint8_t {
    kApiVersion = 1,
    kStatus = 101,
    kRawImu = 102,
    kMotor = 104,
    kRc = 105,
    kAttitude = 108,
    kAltitude = 109,
    kAnalog = 110,
    kStatusEx = 150,
    kAttitudeQuaternion = 167,
    kSetRawRc = 200,
};

enum BoxId : uint16_t {
    kBoxArm = 0,
    kBoxMspOverride = 50,
};

struct ApiVersion {
    uint8_t protocol = 0;
    uint8_t api_major = 0;
    uint8_t api_minor = 0;
};

struct Attitude {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
};

struct RcFrame {
    // Raw receiver order: roll, pitch, throttle, yaw, aux1...
    std::vector<uint16_t> channels;
};

struct MotorFrame {
    std::vector<uint16_t> motors;
};

struct ImuSample {
    std::array<int16_t, 3> acc{{0, 0, 0}};
    std::array<int16_t, 3> gyro{{0, 0, 0}};
    std::array<int16_t, 3> mag{{0, 0, 0}};
};

struct StatusFrame {
    uint16_t cycletime = 0;
    uint16_t i2c_errors = 0;
    uint16_t sensors = 0;
    uint32_t flight_mode_flags = 0; // boxes 0..31 only
    bool armed = false;
    bool msp_override = false;
};

struct AnalogFrame {
    double voltage_v = 0.0;
    double amperage_a = 0.0;
    uint16_t mah = 0;
    uint16_t rssi = 0;
};

struct AltitudeFrame {
    double altitude_m = 0.0;
    double vario_m_s = 0.0;
    bool valid = false;
};

// Thrown when the link is gone or a response never arrives.
class MspError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class MspClient {
public:
    MspClient() = default;
    ~MspClient();

    MspClient(const MspClient&) = delete;
    MspClient& operator=(const MspClient&) = delete;

    void connect(const std::string& host, int port, double timeout_s = 0.1);
    void close();
    bool connected() const { return fd_ >= 0; }
    void set_timeout(double seconds) { timeout_ = seconds; }

    ApiVersion api_version();
    Attitude attitude();
    RcFrame rc();
    MotorFrame motor();
    ImuSample raw_imu();
    StatusFrame status(bool extended = false);
    AnalogFrame analog();
    AltitudeFrame altitude();

    bool is_armed();
    bool is_msp_override_active();
    bool is_box_active(uint16_t box);

    // Send MSP_SET_RAW_RC. `channels` is in raw receiver order. The flight
    // controller acknowledges every frame; those acks are flushed by the next
    // request(), so there is no need to wait for them here (waiting would stall
    // the RC stream). A positive `drain_timeout` (seconds) enables draining when
    // a caller explicitly wants it.
    void set_raw_rc(const std::vector<uint16_t>& channels, double drain_timeout = 0.0);

private:
    std::vector<uint8_t>
        request(uint8_t cmd, const std::vector<uint8_t>& payload, int expect, int attempts);
    void send_frame(uint8_t cmd, const std::vector<uint8_t>& payload);
    void flush();
    bool fill(double timeout_s);
    std::optional<std::pair<uint8_t, std::vector<uint8_t>>> recv_frame(double timeout_s);

    int fd_ = -1;
    double timeout_ = 0.1;
    std::vector<uint8_t> buf_;
    // Serializes writes so a sender (RC stream) and a reader (telemetry) can
    // share one socket without interleaving frames.
    std::mutex send_mutex_;
};

} // namespace flyrat_msp
