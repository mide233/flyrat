// SPDX-License-Identifier: MIT
//
// Four VL53L5CX time-of-flight sensors sharing one I2C bus.
//
// The array is initialised in the only order that works when every sensor
// powers up at 0x52: hold all LPn low, release one sensor, initialise it,
// assign a unique address, then release the next one. The reader thread then
// polls data-ready and copies the trimmed result blocks into TofFrame
// snapshots. Interrupt lines are optional; polling is the default because it
// is robust and the trimmed 4x4@60 Hz traffic still uses less than half of a
// 750 kHz bus.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace flyrat::hardware {

inline constexpr int kTofMaxSensors = 4;
inline constexpr int kTofMaxZones = 64;

struct TofZone {
    std::int16_t distance_mm = 0;
    std::uint8_t target_status = 0;
    std::uint8_t nb_target = 0;

    // Betaflight-style validity for VL53L5CX target status: 5 and 9 are the
    // valid ranging codes, everything else is a measurement artefact.
    bool valid() const { return target_status == 5 || target_status == 9; }
};

struct TofFrame {
    int sensor = 0;
    int resolution = 4;
    std::array<TofZone, kTofMaxZones> zones {};
    std::int8_t silicon_temp_degc = 0;
    std::uint64_t sequence = 0;
    std::chrono::steady_clock::time_point stamp;
};

struct TofConfig {
    std::string i2c_device = "/dev/i2c-2";
    int sensor_count = 4;
    std::array<int, kTofMaxSensors> lpn_gpio { 108, 114, 101, 98 };
    // Optional interrupt lines. Set use_interrupts=false (default) to poll.
    std::array<int, kTofMaxSensors> int_gpio { 107, 104, 100, 97 };
    // 8-bit I2C addresses, one per sensor. All sensors boot at 0x52.
    // The first sensor must also be moved away from 0x52 before the next LPn is
    // released, otherwise the new sensor collides with it.
    std::array<std::uint16_t, kTofMaxSensors> addresses { 0x60, 0x62, 0x64, 0x66 };
    // Address each sensor is expected to answer at before assignment. Normally
    // all 0x52; this is overridable to recover a sensor left at another address.
    std::array<std::uint16_t, kTofMaxSensors> initial_addresses { 0x52, 0x52, 0x52, 0x52 };
    int resolution = 4; // 4 or 8
    int ranging_hz = 60; // 4x4 supports 60 Hz; 8x8 is capped at 15 Hz
    int poll_interval_ms = 2;
    bool use_interrupts = false;
};

struct TofStats {
    std::array<std::uint64_t, kTofMaxSensors> frames {};
    std::array<std::uint64_t, kTofMaxSensors> read_errors {};
    std::array<std::uint32_t, kTofMaxSensors> data_read_size {};
    std::array<std::uint64_t, kTofMaxSensors> read_time_us {};
    std::uint64_t init_errors = 0;
    bool running = false;
};

class TofError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class TofArray {
public:
    explicit TofArray(TofConfig config);
    ~TofArray();

    TofArray(const TofArray&) = delete;
    TofArray& operator=(const TofArray&) = delete;

    // Exports/controls LPn, initialises every sensor, assigns addresses, starts
    // ranging and launches the reader thread. Throws TofError on the first
    // sensor that fails; already-initialised sensors are stopped and all LPn
    // lines are driven low before the exception leaves.
    void start();
    void stop();

    bool running() const;
    std::optional<TofFrame> latest(int sensor) const;
    TofStats stats() const;

    // Runs on the reader thread. It must not block or call stop().
    void set_callback(std::function<void(const TofFrame&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware
