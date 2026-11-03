// SPDX-License-Identifier: MIT
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "flyrat/msp_interact.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

template <typename Predicate>
bool wait_for(Predicate predicate, int timeout_ms = 2000)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
}

class FakeTransport final : public flyrat::hardware::Transport {
public:
    explicit FakeTransport(int fd)
        : fd_(fd)
    {
        const int flags = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    }

    ~FakeTransport() override
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    bool open(std::string&) override
    {
        open_ = true;
        return true;
    }

    void close() noexcept override { open_ = false; }
    bool is_open() const noexcept override { return open_; }
    int native_handle() const noexcept override { return fd_; }

    std::ptrdiff_t read_some(
        std::uint8_t* data, std::size_t size, std::string& error) override
    {
        if (!open_) {
            error = "closed";
            return -1;
        }
        for (;;) {
            const ssize_t n = ::recv(fd_, data, size, 0);
            if (n > 0) {
                return n;
            }
            if (n == 0) {
                error = "peer closed";
                return -1;
            }
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            error = std::strerror(errno);
            return -1;
        }
    }

    bool write_all(const std::uint8_t* data, std::size_t size, int, std::string& error) override
    {
        if (!open_ || fail_writes) {
            error = "injected write failure";
            return false;
        }
        std::size_t written = 0;
        while (written < size) {
            const ssize_t n = ::send(fd_, data + written, size - written, MSG_NOSIGNAL);
            if (n > 0) {
                written += static_cast<std::size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) {
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                struct pollfd pfd {};
                pfd.fd = fd_;
                pfd.events = POLLOUT;
                ::poll(&pfd, 1, 100);
                continue;
            }
            error = std::strerror(errno);
            return false;
        }
        return true;
    }

    std::atomic<bool> fail_writes { false };

private:
    int fd_;
    std::atomic<bool> open_ { false };
};

class FakeFlightController {
public:
    explicit FakeFlightController(int fd)
        : fd_(fd)
        , thread_([this]() { loop(); })
    {
    }

    ~FakeFlightController() { stop(); }

    void stop()
    {
        if (!running_.exchange(false)) {
            return;
        }
        ::shutdown(fd_, SHUT_RDWR);
        if (thread_.joinable()) {
            thread_.join();
        }
        ::close(fd_);
        fd_ = -1;
    }

    std::atomic<bool> skip_imu { false };
    std::atomic<bool> skip_rc_ack { false };

private:
    static void append_response(std::vector<std::uint8_t>& out, std::uint8_t cmd,
        const std::vector<std::uint8_t>& payload)
    {
        out.push_back('$');
        out.push_back('M');
        out.push_back('>');
        out.push_back(static_cast<std::uint8_t>(payload.size()));
        out.push_back(cmd);
        std::uint8_t checksum = static_cast<std::uint8_t>(payload.size()) ^ cmd;
        for (std::uint8_t byte : payload) {
            out.push_back(byte);
            checksum ^= byte;
        }
        out.push_back(checksum);
    }

    void handle(std::uint8_t cmd)
    {
        if (cmd == 102 && skip_imu) {
            return;
        }
        if (cmd == 200) {
            if (skip_rc_ack) {
                return;
            }
            std::vector<std::uint8_t> out;
            append_response(out, cmd, {});
            ::send(fd_, out.data(), out.size(), MSG_NOSIGNAL);
            return;
        }

        std::vector<std::uint8_t> payload;
        switch (cmd) {
        case 1:
            payload = { 0, 1, 46 };
            break;
        case 102:
            payload.assign(18, 0);
            payload[0] = 100;
            payload[6] = 200;
            payload[12] = 44;
            break;
        case 108:
            payload = { 10, 0, 0xF6, 0xFF, 90, 0 }; // roll 1.0, pitch -1.0, yaw 90
            break;
        case 101: {
            payload.assign(16, 0);
            payload[0] = 0xD2;
            payload[1] = 0x04; // cycletime 1234
            payload[6] = 0x01; // ARM box active
            break;
        }
        case 109: {
            payload.assign(6, 0);
            const std::uint32_t cm = 12345;
            payload[0] = static_cast<std::uint8_t>(cm & 0xFF);
            payload[1] = static_cast<std::uint8_t>((cm >> 8) & 0xFF);
            payload[2] = static_cast<std::uint8_t>((cm >> 16) & 0xFF);
            payload[3] = static_cast<std::uint8_t>((cm >> 24) & 0xFF);
            payload[4] = 50; // 0.5 m/s
            break;
        }
        case 105:
            payload.assign(32, 0);
            for (std::size_t i = 0; i < 16; ++i) {
                const std::uint16_t value = static_cast<std::uint16_t>(1000 + i);
                payload[2 * i] = static_cast<std::uint8_t>(value & 0xFF);
                payload[2 * i + 1] = static_cast<std::uint8_t>(value >> 8);
            }
            break;
        case 104:
            payload = { 1, 0, 2, 0, 3, 0, 4, 0 };
            break;
        case 110:
            payload = { 120, 55, 0, 80, 0, 100, 0, 124, 3 };
            break;
        case 106:
            payload.assign(22, 0);
            payload[0] = 3;
            payload[1] = 9;
            break;
        case 254:
            payload.assign(16, 0);
            break;
        default:
            return;
        }

        std::vector<std::uint8_t> out;
        append_response(out, cmd, payload);
        ::send(fd_, out.data(), out.size(), MSG_NOSIGNAL);
    }

    void loop()
    {
        std::vector<std::uint8_t> buffer;
        std::uint8_t chunk[256];
        while (running_) {
            const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
            if (n <= 0) {
                return;
            }
            buffer.insert(buffer.end(), chunk, chunk + n);
            for (;;) {
                std::size_t start = buffer.size();
                for (std::size_t i = 0; i + 4 < buffer.size(); ++i) {
                    if (buffer[i] == '$' && buffer[i + 1] == 'M' && buffer[i + 2] == '<') {
                        start = i;
                        break;
                    }
                }
                if (start == buffer.size()) {
                    buffer.clear();
                    break;
                }
                if (buffer.size() < 6) {
                    break;
                }
                const std::size_t total = 6U + buffer[start + 3];
                if (buffer.size() - start < total) {
                    break;
                }
                const std::uint8_t cmd = buffer[start + 4];
                buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(start + total));
                handle(cmd);
            }
        }
    }

    int fd_;
    std::atomic<bool> running_ { true };
    std::thread thread_;
};

flyrat::MspConfig basic_config()
{
    flyrat::MspConfig config;
    config.poll.attitude_hz = 50;
    config.poll.imu_hz = 100;
    config.poll.status_hz = 20;
    config.poll.altitude_hz = 10;
    config.poll.rc_hz = 0;
    config.poll.analog_hz = 0;
    config.poll.motor_hz = 0;
    config.poll.gps_hz = 0;
    config.poll.debug_hz = 0;
    config.poll_timeout_ms = 50;
    config.request_timeout_ms = 200;
    config.reconnect_backoff_ms = 50;
    return config;
}

void test_basic_polling_and_rc()
{
    int fds[2] = { -1, -1 };
    check(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0, "socketpair");
    FakeFlightController fc(fds[1]);
    auto transport = std::make_unique<FakeTransport>(fds[0]);

    flyrat::MspInteract msp(basic_config(), std::move(transport));

    std::atomic<int> attitudes { 0 };
    std::atomic<int> imus { 0 };
    std::atomic<int> statuses { 0 };
    std::atomic<double> roll { 0.0 };
    std::atomic<std::uint32_t> cycletime { 0 };

    msp.set_attitude_callback([&](const flyrat::Sample<flyrat::Attitude>& sample) {
        roll = sample.value.roll_deg;
        ++attitudes;
    });
    msp.set_imu_callback([&](const flyrat::Sample<flyrat::ImuSample>&) { ++imus; });
    msp.set_status_callback([&](const flyrat::Sample<flyrat::StatusFrame>& sample) {
        cycletime = sample.value.cycletime;
        ++statuses;
    });

    msp.start();
    check(msp.connected(), "connected after start");
    check(wait_for([&]() { return attitudes > 0 && imus > 0 && statuses > 0; }),
        "telemetry callbacks");
    check(std::abs(roll.load() - 1.0) < 0.01, "attitude roll");
    check(cycletime == 1234, "status cycletime");

    const auto version = msp.api_version();
    check(version.has_value() && version->api_major == 1, "api version snapshot");

    const auto imu = msp.imu();
    check(imu.has_value() && imu->value.acc[0] == 100, "imu snapshot");

    std::vector<std::uint16_t> channels(16, 1500);
    check(msp.send_rc(channels), "send_rc accepted");

    const auto ack = wait_for([&]() { return msp.stats().rc_ack > 0; });
    check(ack, "rc ack");
    check(msp.stats().rc_sent == 1, "rc sent count");

    std::atomic<bool> request_done { false };
    std::atomic<bool> request_ok { false };
    std::atomic<std::size_t> request_size { 0 };
    msp.request(flyrat::MspCommand::kMotor,
        [&](bool ok, const std::vector<std::uint8_t>& payload) {
            request_ok = ok;
            request_size = payload.size();
            request_done = true;
        });
    check(wait_for([&]() { return request_done.load(); }), "one-shot completion");
    check(request_ok && request_size == 8, "one-shot payload");

    msp.stop();
    check(!msp.connected(), "disconnected after stop");
}

void test_timeout_and_reconnect()
{
    int fds[2] = { -1, -1 };
    check(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0, "socketpair reconnect");
    FakeFlightController fc(fds[1]);
    fc.skip_imu = true;
    auto transport = std::make_unique<FakeTransport>(fds[0]);

    flyrat::MspConfig config = basic_config();
    config.poll.attitude_hz = 0;
    config.poll.imu_hz = 100;
    config.poll.status_hz = 0;
    config.poll.altitude_hz = 0;
    config.poll_timeout_ms = 20;
    config.reconnect_backoff_ms = 50;

    flyrat::MspInteract msp(config, std::move(transport));
    std::atomic<int> imus { 0 };
    msp.set_imu_callback([&](const flyrat::Sample<flyrat::ImuSample>&) { ++imus; });

    msp.start();
    check(msp.connected(), "reconnect test connected");
    check(wait_for([&]() { return msp.stats().reconnects >= 1; }, 3000),
        "timeout triggers reconnect");
    check(msp.stats().timeouts >= 3, "timeouts counted");

    fc.skip_imu = false;
    check(wait_for([&]() { return msp.connected(); }, 2000), "reconnects");
    check(wait_for([&]() { return imus > 0; }, 2000), "imu resumes after reconnect");
    check(msp.stats().reconnects >= 1, "reconnect counted");

    msp.stop();
}

} // namespace

int main()
{
    test_basic_polling_and_rc();
    test_timeout_and_reconnect();

    if (g_failures != 0) {
        std::printf("%d msp_interact check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("msp_interact tests passed\n");
    return 0;
}
