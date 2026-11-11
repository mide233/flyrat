// SPDX-License-Identifier: MIT
//
// TCP server that streams the on-vehicle SLAM pose to the host pose bridge.
// Only the newest pose is kept: stale poses are useless to a viewer, and the
// estimator may run faster than the tunnel.
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "flyrat/sensors/protocol.hpp"

namespace flyrat::pilot {

struct PoseStreamConfig {
    std::string listen_address = "127.0.0.1";
    int listen_port = 5001;
};

class PoseStream {
public:
    explicit PoseStream(PoseStreamConfig config);
    ~PoseStream();

    PoseStream(const PoseStream&) = delete;
    PoseStream& operator=(const PoseStream&) = delete;

    void start();
    void stop();

    // device_time_ns should be a Unix-epoch timestamp so the host can use it
    // directly for the ROS message header.
    void publish(const sensors::PosePayload& pose, std::int64_t device_time_ns);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::pilot
