// SPDX-License-Identifier: MIT
//
// Real-machine / SITL acceptance tool for MspInteract.
//
//   msp_interact_bench --device /dev/ttyS3 --baud 921600
//       --seconds 30 --rc-hz 200 --imu-hz 200 --enable-rc
//
// It sends neutral RC frames (all channels 1500) and never arms. Props must be
// removed before running against a powered flight controller.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/hardware/tcp_transport.hpp"
#include "flyrat/msp_interact.hpp"

namespace {

struct Options {
    std::string device;
    int baud = 921600;
    std::string host = "127.0.0.1";
    int port = 5761;
    int seconds = 10;
    int rc_hz = 0;
    int imu_hz = 200;
    bool enable_rc = false;
};

void usage()
{
    std::printf(
        "usage: msp_interact_bench [--device /dev/ttyS3 --baud 921600] "
        "[--host 127.0.0.1 --port 5761] [--seconds 30] [--imu-hz 200] "
        "[--rc-hz 200] [--enable-rc]\n");
}

double percentile(std::vector<double> values, double fraction)
{
    if (values.empty()) {
        return -1.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(
        std::min<double>(values.size() - 1, fraction * (values.size() - 1)));
    return values[index];
}

void report(const std::string& name, std::vector<double> values)
{
    if (values.empty()) {
        std::printf("%s: no samples\n", name.c_str());
        return;
    }
    std::printf("%s: n=%zu p50=%.2f p95=%.2f p99=%.2f max=%.2f ms\n", name.c_str(),
        values.size(), percentile(values, 0.50), percentile(values, 0.95),
        percentile(values, 0.99), percentile(values, 1.0));
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--device") {
            options.device = next("--device");
        } else if (arg == "--baud") {
            options.baud = std::atoi(next("--baud").c_str());
        } else if (arg == "--host") {
            options.host = next("--host");
        } else if (arg == "--port") {
            options.port = std::atoi(next("--port").c_str());
        } else if (arg == "--seconds") {
            options.seconds = std::atoi(next("--seconds").c_str());
        } else if (arg == "--rc-hz") {
            options.rc_hz = std::atoi(next("--rc-hz").c_str());
        } else if (arg == "--imu-hz") {
            options.imu_hz = std::atoi(next("--imu-hz").c_str());
        } else if (arg == "--enable-rc") {
            options.enable_rc = true;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (options.seconds <= 0 || options.imu_hz <= 0) {
        std::fprintf(stderr, "seconds and imu-hz must be positive\n");
        return 2;
    }

    std::unique_ptr<flyrat::hardware::Transport> transport;
    if (!options.device.empty()) {
        transport = flyrat::hardware::make_serial_transport(options.device, options.baud);
        std::printf("transport=serial device=%s baud=%d\n", options.device.c_str(), options.baud);
    } else {
        transport = flyrat::hardware::make_tcp_transport(options.host, options.port);
        std::printf("transport=tcp host=%s port=%d\n", options.host.c_str(), options.port);
    }

    flyrat::MspConfig config;
    config.poll.imu_hz = options.imu_hz;
    config.poll.attitude_hz = 50;
    config.poll.status_hz = 10;
    config.poll.altitude_hz = 10;
    config.poll.rc_hz = 0;
    config.poll.analog_hz = 1;
    config.poll.motor_hz = 0;
    config.poll.gps_hz = 0;
    config.poll_timeout_ms = 20;
    config.request_timeout_ms = 500;
    config.reconnect_backoff_ms = 1000;

    flyrat::MspInteract msp(config, std::move(transport));

    std::mutex samples_mu;
    std::vector<double> imu_latencies;
    std::atomic<std::uint64_t> imu_count { 0 };
    std::atomic<std::uint64_t> rc_count { 0 };

    msp.set_imu_callback([&](const flyrat::Sample<flyrat::ImuSample>&) {
        ++imu_count;
        const double latency = msp.stats().last_imu_latency_ms;
        if (latency >= 0.0) {
            std::lock_guard<std::mutex> lock(samples_mu);
            imu_latencies.push_back(latency);
        }
    });
    msp.set_link_callback([](bool up, const std::string& why) {
        std::printf("link %s: %s\n", up ? "up" : "down", why.c_str());
    });

    try {
        msp.start();
    } catch (const flyrat::MspError& error) {
        std::fprintf(stderr, "start failed: %s\n", error.what());
        return 1;
    }

    if (const auto version = msp.api_version()) {
        std::printf("api=%u.%u protocol=%u\n", version->api_major, version->api_minor,
            version->protocol);
    }

    if (options.enable_rc && options.rc_hz > 0) {
        std::printf("RC enabled at %d Hz, all channels 1500 (props OFF!)\n", options.rc_hz);
    }

    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(options.seconds);
    auto next_rc = std::chrono::steady_clock::now();
    std::vector<double> rc_latencies;
    std::uint64_t last_ack = 0;
    std::vector<std::uint16_t> channels(16, 1500);

    while (std::chrono::steady_clock::now() < deadline) {
        if (options.enable_rc && options.rc_hz > 0) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_rc) {
                ++rc_count;
                if (msp.send_rc(channels)) {
                    const auto wait_until = now + std::chrono::milliseconds(50);
                    while (std::chrono::steady_clock::now() < wait_until) {
                        const auto stats = msp.stats();
                        if (stats.rc_ack > last_ack) {
                            last_ack = stats.rc_ack;
                            if (stats.last_rc_ack_ms >= 0.0) {
                                rc_latencies.push_back(stats.last_rc_ack_ms);
                            }
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::microseconds(200));
                    }
                }
                next_rc += std::chrono::microseconds(1000000 / options.rc_hz);
                if (next_rc < now) {
                    next_rc = now + std::chrono::microseconds(1000000 / options.rc_hz);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    const auto stats = msp.stats();
    std::vector<double> imu_copy;
    {
        std::lock_guard<std::mutex> lock(samples_mu);
        imu_copy = imu_latencies;
    }

    std::printf("elapsed=%ds imu=%llu rc_sent=%llu rc_ack=%llu timeouts=%llu "
                "rc_timeouts=%llu checksum=%llu malformed=%llu reconnects=%llu\n",
        options.seconds, static_cast<unsigned long long>(imu_count.load()),
        static_cast<unsigned long long>(stats.rc_sent),
        static_cast<unsigned long long>(stats.rc_ack),
        static_cast<unsigned long long>(stats.timeouts),
        static_cast<unsigned long long>(stats.rc_timeouts),
        static_cast<unsigned long long>(stats.checksum_errors),
        static_cast<unsigned long long>(stats.malformed_frames),
        static_cast<unsigned long long>(stats.reconnects));
    std::printf("effective imu rate=%.1f Hz (target %d Hz)\n",
        imu_count.load() / static_cast<double>(options.seconds), options.imu_hz);
    report("imu latency", std::move(imu_copy));
    report("rc ack latency", std::move(rc_latencies));

    msp.stop();
    return imu_count.load() > 0 ? 0 : 1;
}
