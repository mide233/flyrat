// SPDX-License-Identifier: MIT
//
// Asynchronous Betaflight MSP v1 client.
//
// The object owns a single I/O thread that talks to the flight controller and
// a dispatch thread that runs user callbacks. Production code never touches
// the serial port directly: it creates a hardware::Transport (serial for the
// vehicle, TCP for SITL) and reads either callbacks or the latest snapshots.
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "flyrat/hardware/transport.hpp"
#include "flyrat/msp/types.hpp"

namespace flyrat {

template <typename T>
struct Sample {
    T value;
    std::chrono::steady_clock::time_point stamp;
};

struct MspPollConfig {
    int attitude_hz = 50;
    int imu_hz = 200;
    int status_hz = 10;
    int altitude_hz = 10;
    int rc_hz = 5;
    int analog_hz = 1;
    int motor_hz = 0; // 0 disables polling; use request() for on-demand data.
    int gps_hz = 0;
    int debug_hz = 0;
    int raw_rc_hz = 0;
};

struct MspConfig {
    int handshake_timeout_ms = 500;
    int poll_timeout_ms = 20;    // per polled command; 200 Hz leaves ~5 ms
    int request_timeout_ms = 500; // one-shot requests such as motor/gps
    int write_timeout_ms = 50;
    int reconnect_backoff_ms = 1000;
    int max_inflight = 4;
    bool status_extended = true;
    MspPollConfig poll;
};

struct LinkStats {
    bool connected = false;
    std::uint64_t tx_frames = 0;
    std::uint64_t rx_frames = 0;
    std::uint64_t timeouts = 0;
    std::uint64_t rc_timeouts = 0;
    std::uint64_t checksum_errors = 0;
    std::uint64_t malformed_frames = 0;
    std::uint64_t reconnects = 0;
    std::uint64_t rc_sent = 0;
    std::uint64_t rc_ack = 0;
    double last_rx_age_ms = -1.0;
    double last_rc_ack_ms = -1.0; // latency of the most recent RC ACK
    double last_imu_latency_ms = -1.0; // request -> response, MSP_RAW_IMU
    std::string last_error;
};

class MspInteract {
public:
    MspInteract(MspConfig config, std::unique_ptr<hardware::Transport> transport);
    ~MspInteract();

    MspInteract(const MspInteract&) = delete;
    MspInteract& operator=(const MspInteract&) = delete;

    // Opens the transport and performs the MSP_ApiVersion handshake. Throws
    // MspError if the initial connection or handshake fails. Once started,
    // link failures are recovered automatically and reported via the link
    // callback; they never throw.
    void start();
    void stop();

    void set_attitude_callback(std::function<void(const Sample<Attitude>&)>);
    void set_imu_callback(std::function<void(const Sample<ImuSample>&)>);
    void set_status_callback(std::function<void(const Sample<StatusFrame>&)>);
    void set_altitude_callback(std::function<void(const Sample<AltitudeFrame>&)>);
    void set_rc_callback(std::function<void(const Sample<RcFrame>&)>);
    void set_raw_rc_callback(std::function<void(const Sample<RawRcFrame>&)>);
    void set_motor_callback(std::function<void(const Sample<MotorFrame>&)>);
    void set_analog_callback(std::function<void(const Sample<AnalogFrame>&)>);
    void set_gps_callback(std::function<void(const Sample<GpsFrame>&)>);
    void set_debug_callback(std::function<void(const Sample<DebugFrame>&)>);
    void set_link_callback(std::function<void(bool connected, const std::string& why)>);

    std::optional<Sample<Attitude>> attitude() const;
    std::optional<Sample<ImuSample>> imu() const;
    std::optional<Sample<StatusFrame>> status() const;
    std::optional<Sample<AltitudeFrame>> altitude() const;
    std::optional<Sample<RcFrame>> rc() const;
    std::optional<Sample<RawRcFrame>> rawRc() const;
    std::optional<Sample<MotorFrame>> motor() const;
    std::optional<Sample<AnalogFrame>> analog() const;
    std::optional<Sample<GpsFrame>> gps() const;
    std::optional<Sample<DebugFrame>> debug() const;
    std::optional<ApiVersion> api_version() const;

    // Non-blocking. Writes one frame to a single-slot mailbox; the I/O thread
    // sends it without waiting for an ACK. Returns false if disconnected or
    // if `channels` is not exactly 16 entries.
    bool send_rc(const std::vector<std::uint16_t>& channels);

    // One-shot asynchronous request. The completion runs on the dispatch
    // thread. `payload` is the raw response payload on success.
    void request(MspCommand cmd,
        std::function<void(bool ok, const std::vector<std::uint8_t>& payload)> done);

    LinkStats stats() const;
    bool connected() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat
