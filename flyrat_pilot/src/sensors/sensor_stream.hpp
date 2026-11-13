// SPDX-License-Identifier: MIT
//
// Reusable TCP server for the sensor stream protocol.
//
// The sensor forwarder and the raw recorder both publish the same protocol, so
// the socket handling, queueing and clock messages live here instead of being
// duplicated in each program. The server only ever accepts one host at a time.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "flyrat/sensors/protocol.hpp"

namespace flyrat::pilot {

struct SensorStreamStats {
    std::uint64_t priority_sent = 0;
    std::uint64_t stereo_sent = 0;
    std::uint64_t priority_dropped = 0;
    std::uint64_t stereo_dropped = 0;
    bool connected = false;
};

class SensorStreamServer {
public:
    explicit SensorStreamServer(std::string label = "sensor stream");
    ~SensorStreamServer();

    SensorStreamServer(const SensorStreamServer&) = delete;
    SensorStreamServer& operator=(const SensorStreamServer&) = delete;

    void start(const std::string& address, int port);
    void stop();

    bool running() const;
    bool connected() const;

    void send_imu(const sensors::ImuPayload& payload, std::int64_t device_time_ns);
    void send_tof(const sensors::TofPayload& payload, std::int64_t device_time_ns);
    void send_stereo(std::uint32_t width, std::uint32_t height,
        const std::uint8_t* jpeg, std::size_t size, std::int64_t device_time_ns);

    SensorStreamStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::pilot
