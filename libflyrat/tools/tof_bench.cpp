// SPDX-License-Identifier: MIT
//
// Four-sensor VL53L5CX acceptance tool.
//
//   tof_bench --device /dev/i2c-2 --seconds 10 --resolution 4 --hz 60
//
// It initialises the array (address assignment included), prints per-sensor
// frame rates, read errors and the first zone of the latest frame. Props must
// be removed if the flight controller is also powered.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <string>
#include <thread>

#include "flyrat/hardware/tof.hpp"

namespace {

struct Options {
    std::string device = "/dev/i2c-2";
    int seconds = 10;
    int resolution = 4;
    int hz = 60;
    int sensors = 4;
    int poll_ms = 2;
    bool interrupts = false;
    std::array<int, flyrat::hardware::kTofMaxSensors> lpn { 108, 114, 101, 98 };
    std::array<int, flyrat::hardware::kTofMaxSensors> ints { 107, 104, 100, 97 };
    std::array<std::uint16_t, flyrat::hardware::kTofMaxSensors> addresses { 0x60, 0x62, 0x64, 0x66 };
    std::array<std::uint16_t, flyrat::hardware::kTofMaxSensors> initial { 0x52, 0x52, 0x52, 0x52 };
};

void usage()
{
    std::printf("usage: tof_bench [--device /dev/i2c-2] [--seconds 10] "
                "[--resolution 4|8] [--hz 60] [--sensors 4] [--poll-ms 2] [--interrupts] "
                "[--lpn 98,97,103,114] [--int 99,100,101,102] "
                "[--addresses 0x60,0x62,0x64,0x66] "
                "[--initial 0x52,0x52,0x52,0x52]\n");
}

bool parse_int_list(const std::string& text, int* values, int count)
{
    std::size_t start = 0;
    for (int i = 0; i < count; ++i) {
        const std::size_t comma = text.find(',', start);
        const std::string part = text.substr(start, comma - start);
        if (part.empty()) {
            return false;
        }
        char* end = nullptr;
        values[i] = static_cast<int>(std::strtol(part.c_str(), &end, 0));
        if (end == nullptr || *end != '\0') {
            return false;
        }
        if (comma == std::string::npos) {
            return i == count - 1;
        }
        start = comma + 1;
    }
    return false;
}

bool parse_address_list(const std::string& text, std::uint16_t* values, int count)
{
    int parsed[flyrat::hardware::kTofMaxSensors] {};
    if (!parse_int_list(text, parsed, count)) {
        return false;
    }
    for (int i = 0; i < count; ++i) {
        if (parsed[i] < 0x08 || parsed[i] > 0xEE) {
            return false;
        }
        values[i] = static_cast<std::uint16_t>(parsed[i]);
    }
    return true;
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
        } else if (arg == "--seconds") {
            options.seconds = std::atoi(next("--seconds").c_str());
        } else if (arg == "--resolution") {
            options.resolution = std::atoi(next("--resolution").c_str());
        } else if (arg == "--hz") {
            options.hz = std::atoi(next("--hz").c_str());
        } else if (arg == "--sensors") {
            options.sensors = std::atoi(next("--sensors").c_str());
        } else if (arg == "--poll-ms") {
            options.poll_ms = std::atoi(next("--poll-ms").c_str());
        } else if (arg == "--interrupts") {
            options.interrupts = true;
        } else if (arg == "--lpn") {
            if (!parse_int_list(next("--lpn"), options.lpn.data(),
                    flyrat::hardware::kTofMaxSensors)) {
                std::fprintf(stderr, "invalid --lpn list\n");
                return 2;
            }
        } else if (arg == "--int") {
            if (!parse_int_list(next("--int"), options.ints.data(),
                    flyrat::hardware::kTofMaxSensors)) {
                std::fprintf(stderr, "invalid --int list\n");
                return 2;
            }
        } else if (arg == "--addresses") {
            if (!parse_address_list(next("--addresses"), options.addresses.data(),
                    flyrat::hardware::kTofMaxSensors)) {
                std::fprintf(stderr, "invalid --addresses list\n");
                return 2;
            }
        } else if (arg == "--initial") {
            if (!parse_address_list(next("--initial"), options.initial.data(),
                    flyrat::hardware::kTofMaxSensors)) {
                std::fprintf(stderr, "invalid --initial list\n");
                return 2;
            }
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (options.seconds <= 0) {
        std::fprintf(stderr, "seconds must be positive\n");
        return 2;
    }

    flyrat::hardware::TofConfig config;
    config.i2c_device = options.device;
    config.sensor_count = options.sensors;
    config.resolution = options.resolution;
    config.ranging_hz = options.hz;
    config.poll_interval_ms = options.poll_ms;
    config.use_interrupts = options.interrupts;
    config.lpn_gpio = options.lpn;
    config.int_gpio = options.ints;
    config.addresses = options.addresses;
    config.initial_addresses = options.initial;

    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    flyrat::hardware::TofArray tof(config);
    std::atomic<std::uint64_t> callbacks { 0 };
    tof.set_callback([&](const flyrat::hardware::TofFrame&) { ++callbacks; });

    std::printf("opening %s: %d sensors, %dx%d @ %d Hz%s\n", config.i2c_device.c_str(),
        config.sensor_count, config.resolution, config.resolution, config.ranging_hz,
        config.use_interrupts ? ", interrupts" : ", polling");

    const auto started = std::chrono::steady_clock::now();
    try {
        tof.start();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "tof start failed: %s\n", error.what());
        return 1;
    }
    std::printf("initialised in %.0f ms\n",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count());

    const auto measure_start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(options.seconds));

    const auto stats = tof.stats();
    const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - measure_start)
                               .count();
    for (int i = 0; i < config.sensor_count; ++i) {
        std::printf("sensor %d: frames=%llu (%.1f Hz) read_errors=%llu result_block=%u B "
                    "avg_read=%.2f ms\n", i,
            static_cast<unsigned long long>(stats.frames[i]), stats.frames[i] / seconds,
            static_cast<unsigned long long>(stats.read_errors[i]), stats.data_read_size[i],
            stats.frames[i] > 0
                ? stats.read_time_us[i] / 1000.0 / static_cast<double>(stats.frames[i])
                : 0.0);
        if (const auto frame = tof.latest(i)) {
            std::printf("  seq=%llu temp=%dC", static_cast<unsigned long long>(frame->sequence),
                static_cast<int>(frame->silicon_temp_degc));
            for (int zone = 0; zone < std::min(4, config.resolution * config.resolution); ++zone) {
                const auto& value = frame->zones[zone];
                std::printf(" z%d=%dmm/%u/%u", zone, static_cast<int>(value.distance_mm),
                    static_cast<unsigned>(value.target_status),
                    static_cast<unsigned>(value.nb_target));
            }
            std::printf("\n");
        } else {
            std::printf("  no frame\n");
        }
    }
    std::printf("callbacks=%llu\n", static_cast<unsigned long long>(callbacks.load()));

    tof.stop();
    return stats.frames[0] > 0 ? 0 : 1;
}
