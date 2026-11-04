// SPDX-License-Identifier: MIT
#include "flyrat/hardware/tof.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

extern "C" {
#include "platform.h"
#include "vl53l5cx_api.h"
}

namespace flyrat::hardware {
namespace {

bool write_text(const std::string& path, const std::string& value, std::string& error)
{
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        error = "open " + path + ": " + std::strerror(errno);
        return false;
    }
    const ssize_t n = ::write(fd, value.data(), value.size());
    ::close(fd);
    if (n != static_cast<ssize_t>(value.size())) {
        error = "write " + path + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

bool ensure_exported(int gpio, std::string& error)
{
    if (gpio < 0) {
        error = "invalid GPIO number";
        return false;
    }
    const std::string base = "/sys/class/gpio/gpio" + std::to_string(gpio);
    if (::access(base.c_str(), F_OK) == 0) {
        return true;
    }
    if (!write_text("/sys/class/gpio/export", std::to_string(gpio), error)) {
        // A concurrent export or a stale sysfs entry is harmless.
        if (::access(base.c_str(), F_OK) != 0) {
            return false;
        }
    }
    for (int i = 0; i < 200; ++i) {
        if (::access(base.c_str(), F_OK) == 0) {
            return true;
        }
        ::usleep(1000);
    }
    error = "GPIO " + std::to_string(gpio) + " did not appear in sysfs";
    return false;
}

class GpioOutput {
public:
    void reset(int number) { number_ = number; }

    bool init(std::string& error)
    {
        if (!ensure_exported(number_, error)) {
            return false;
        }
        const std::string base = "/sys/class/gpio/gpio" + std::to_string(number_);
        return write_text(base + "/direction", "out", error) && set(false, error);
    }

    bool set(bool high, std::string& error)
    {
        const std::string base = "/sys/class/gpio/gpio" + std::to_string(number_);
        return write_text(base + "/value", high ? "1" : "0", error);
    }

private:
    int number_ = -1;
};

class GpioInput {
public:
    void reset(int number) { number_ = number; }

    bool init(std::string& error)
    {
        if (!ensure_exported(number_, error)) {
            return false;
        }
        const std::string base = "/sys/class/gpio/gpio" + std::to_string(number_);
        if (!write_text(base + "/direction", "in", error)) {
            return false;
        }
        // A 100 us pulse must not be filtered by debounce.
        if (!write_text(base + "/edge", "falling", error)) {
            return false;
        }
        fd_ = ::open((base + "/value").c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            error = "open " + base + "/value: " + std::strerror(errno);
            return false;
        }
        clear();
        return true;
    }

    int fd() const { return fd_; }

    void clear()
    {
        if (fd_ < 0) {
            return;
        }
        (void)::lseek(fd_, 0, SEEK_SET);
        char buffer[8];
        while (::read(fd_, buffer, sizeof(buffer)) > 0) {
        }
    }

    void close()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~GpioInput() { close(); }

private:
    int number_ = -1;
    int fd_ = -1;
};

const char* status_name(std::uint8_t status)
{
    switch (status) {
    case 0:
        return "ok";
    case 1:
        return "timeout";
    case 2:
        return "corrupted frame";
    case 3:
        return "crc";
    case 4:
        return "xtalk";
    default:
        return "error";
    }
}

} // namespace

struct TofArray::Impl {
    struct Device {
        VL53L5CX_Configuration driver {};
        VL53L5CX_ResultsData results {};
        bool initialized = false;
        bool ranging = false;
        std::uint64_t sequence = 0;
    };

    explicit Impl(TofConfig cfg)
        : config(std::move(cfg))
    {
        for (int i = 0; i < kTofMaxSensors; ++i) {
            lpn[i].reset(config.lpn_gpio[i]);
            interrupt[i].reset(config.int_gpio[i]);
        }
    }

    ~Impl() { stop(); }

    void cleanup()
    {
        for (auto& device : devices) {
            if (device.ranging) {
                (void)vl53l5cx_stop_ranging(&device.driver);
                device.ranging = false;
            }
            if (device.initialized) {
                (void)vl53l5cx_comms_close(&device.driver.platform);
                device.initialized = false;
            }
        }
        for (int i = 0; i < config.sensor_count; ++i) {
            std::string ignored;
            (void)lpn[i].set(false, ignored);
            interrupt[i].close();
        }
    }

    [[noreturn]] void fail(int sensor, const char* step, std::uint8_t status)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++init_errors;
        }
        cleanup();
        throw TofError("ToF sensor " + std::to_string(sensor) + " " + step
            + " failed (" + status_name(status) + ", status "
            + std::to_string(static_cast<unsigned>(status)) + ")");
    }

    void start()
    {
        if (running.load()) {
            return;
        }
        if (config.sensor_count < 1 || config.sensor_count > kTofMaxSensors) {
            throw TofError("sensor_count must be 1..4");
        }
        if (config.resolution != 4 && config.resolution != 8) {
            throw TofError("resolution must be 4 or 8");
        }
        const int max_hz = config.resolution == 4 ? 60 : 15;
        if (config.ranging_hz <= 0 || config.ranging_hz > max_hz) {
            throw TofError("ranging_hz must be 1.." + std::to_string(max_hz));
        }
        for (int i = 0; i < config.sensor_count; ++i) {
            for (int j = i + 1; j < config.sensor_count; ++j) {
                if (config.addresses[i] == config.addresses[j]) {
                    throw TofError("ToF addresses must be unique");
                }
            }
        }

        std::string error;
        for (int i = 0; i < config.sensor_count; ++i) {
            if (!lpn[i].init(error)) {
                cleanup();
                throw TofError("LPn GPIO " + std::to_string(config.lpn_gpio[i]) + ": " + error);
            }
        }
        // All sensors boot at 0x52. Keep every LPn low until it is that
        // sensor's turn, otherwise the bus has four devices at one address.
        for (int i = 0; i < config.sensor_count; ++i) {
            std::string ignored;
            (void)lpn[i].set(false, ignored);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        for (int i = 0; i < config.sensor_count; ++i) {
            if (!lpn[i].set(true, error)) {
                cleanup();
                throw TofError("LPn GPIO " + std::to_string(config.lpn_gpio[i]) + ": " + error);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            auto& device = devices[i];
            std::memset(&device.driver, 0, sizeof(device.driver));
            device.driver.platform.i2c_bus = config.i2c_device.c_str();

            if (vl53l5cx_comms_init(&device.driver.platform) != 0) {
                fail(i, "I2C open", 255);
            }
            device.initialized = true;
            device.driver.platform.address = config.initial_addresses[i];

            std::uint8_t alive = 0;
            std::uint8_t status = vl53l5cx_is_alive(&device.driver, &alive);
            if (status != 0 || alive == 0) {
                fail(i, "is_alive", status);
            }

            status = vl53l5cx_init(&device.driver);
            if (status != 0) {
                fail(i, "init", status);
            }

            if (config.addresses[i] != config.initial_addresses[i]) {
                status = vl53l5cx_set_i2c_address(&device.driver, config.addresses[i]);
                if (status != 0) {
                    fail(i, "set_i2c_address", status);
                }
            }

            status = vl53l5cx_set_resolution(&device.driver,
                config.resolution == 4 ? VL53L5CX_RESOLUTION_4X4 : VL53L5CX_RESOLUTION_8X8);
            if (status != 0) {
                fail(i, "set_resolution", status);
            }

            // A fresh module can come up with a longer integration time (and
            // therefore a lower frame rate). Pin it to 5 ms so 4x4 can reach
            // 60 Hz; 8x8 still runs at its 15 Hz ceiling.
            status = vl53l5cx_set_integration_time_ms(&device.driver, 5);
            if (status != 0) {
                fail(i, "set_integration_time_ms", status);
            }

            status = vl53l5cx_set_ranging_frequency_hz(
                &device.driver, static_cast<std::uint8_t>(config.ranging_hz));
            if (status != 0) {
                fail(i, "set_ranging_frequency_hz", status);
            }

            status = vl53l5cx_start_ranging(&device.driver);
            if (status != 0) {
                fail(i, "start_ranging", status);
            }
            device.ranging = true;
        }

        if (config.use_interrupts) {
            for (int i = 0; i < config.sensor_count; ++i) {
                if (config.int_gpio[i] < 0) {
                    continue;
                }
                if (!interrupt[i].init(error)) {
                    cleanup();
                    throw TofError("INT GPIO " + std::to_string(config.int_gpio[i]) + ": " + error);
                }
            }
        }

        running.store(true);
        reader = std::thread([this]() { reader_loop(); });
    }

    void stop()
    {
        const bool was_running = running.exchange(false);
        if (was_running && reader.joinable()) {
            reader.join();
        } else if (reader.joinable()) {
            reader.join();
        }
        cleanup();
    }

    void reader_loop()
    {
        std::vector<struct pollfd> fds;
        std::vector<int> fds_sensor;
        if (config.use_interrupts) {
            for (int i = 0; i < config.sensor_count; ++i) {
                if (interrupt[i].fd() >= 0) {
                    fds.push_back({ interrupt[i].fd(), POLLPRI, 0 });
                    fds_sensor.push_back(i);
                }
            }
        }

        while (running.load()) {
            if (!fds.empty()) {
                const int rc = ::poll(fds.data(), fds.size(), config.poll_interval_ms);
                if (rc > 0) {
                    for (std::size_t k = 0; k < fds.size(); ++k) {
                        if ((fds[k].revents & (POLLPRI | POLLERR)) != 0) {
                            interrupt[fds_sensor[k]].clear();
                        }
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(config.poll_interval_ms));
            }

            for (int i = 0; i < config.sensor_count; ++i) {
                read_sensor(i);
            }
        }
    }

    void read_sensor(int index)
    {
        auto& device = devices[index];
        const auto read_start = std::chrono::steady_clock::now();
        std::uint8_t ready = 0;
        std::uint8_t status = vl53l5cx_check_data_ready(&device.driver, &ready);
        if (status != 0) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                ++read_errors[index];
            }
            return;
        }
        if (ready == 0) {
            return;
        }

        status = vl53l5cx_get_ranging_data(&device.driver, &device.results);
        if (status != 0) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                ++read_errors[index];
            }
            return;
        }
        const auto read_done = std::chrono::steady_clock::now();

        TofFrame frame;
        frame.sensor = index;
        frame.resolution = config.resolution;
        frame.silicon_temp_degc = device.results.silicon_temp_degc;
        frame.sequence = ++device.sequence;
        frame.stamp = std::chrono::steady_clock::now();
        const int zone_count = config.resolution * config.resolution;
        for (int zone = 0; zone < zone_count; ++zone) {
            frame.zones[zone].distance_mm = device.results.distance_mm[zone];
            frame.zones[zone].target_status = device.results.target_status[zone];
            frame.zones[zone].nb_target = device.results.nb_target_detected[zone];
        }

        std::function<void(const TofFrame&)> callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            latest[index] = frame;
            ++frames[index];
            read_time_us[index] += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(read_done - read_start)
                    .count());
        }
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            callback = callback_;
        }
        if (callback) {
            callback(frame);
        }
    }

    TofConfig config;
    std::array<Device, kTofMaxSensors> devices;
    std::array<GpioOutput, kTofMaxSensors> lpn;
    std::array<GpioInput, kTofMaxSensors> interrupt;
    std::thread reader;
    std::atomic<bool> running { false };

    mutable std::mutex mutex;
    std::array<std::optional<TofFrame>, kTofMaxSensors> latest;
    std::array<std::uint64_t, kTofMaxSensors> frames {};
    std::array<std::uint64_t, kTofMaxSensors> read_errors {};
    std::array<std::uint64_t, kTofMaxSensors> read_time_us {};
    std::uint64_t init_errors = 0;

    std::mutex callback_mutex;
    std::function<void(const TofFrame&)> callback_;
};

TofArray::TofArray(TofConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

TofArray::~TofArray() = default;

void TofArray::start() { impl_->start(); }
void TofArray::stop() { impl_->stop(); }

bool TofArray::running() const { return impl_->running.load(); }

TofStats TofArray::stats() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    TofStats out;
    out.frames = impl_->frames;
    out.read_errors = impl_->read_errors;
    out.read_time_us = impl_->read_time_us;
    for (int i = 0; i < kTofMaxSensors; ++i) {
        out.data_read_size[i] = impl_->devices[i].driver.data_read_size;
    }
    out.init_errors = impl_->init_errors;
    out.running = impl_->running.load();
    return out;
}

std::optional<TofFrame> TofArray::latest(int sensor) const
{
    if (sensor < 0 || sensor >= kTofMaxSensors) {
        return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->latest[sensor];
}

void TofArray::set_callback(std::function<void(const TofFrame&)> callback)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mutex);
    impl_->callback_ = std::move(callback);
}

} // namespace flyrat::hardware
