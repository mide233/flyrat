// SPDX-License-Identifier: MIT
#include "sensors/sensor_stream.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace flyrat::pilot {
namespace {

using Clock = std::chrono::steady_clock;

std::int64_t to_ns(Clock::time_point stamp)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        stamp.time_since_epoch()).count();
}

bool send_all(int fd, const std::vector<std::uint8_t>& data)
{
    std::size_t offset = 0;
    while (offset < data.size()) {
        const ssize_t sent = ::send(fd, data.data() + offset, data.size() - offset,
            MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (sent == 0) {
            return false;
        }
        offset += static_cast<std::size_t>(sent);
    }
    return true;
}

} // namespace

struct SensorStreamServer::Impl {
    explicit Impl(std::string name)
        : label(std::move(name))
    {
    }

    ~Impl()
    {
        stop();
    }

    void start(const std::string& address, int port)
    {
        if (running) {
            return;
        }
        open_server(address, port);
        running = true;
        writer_thread = std::thread([this] { writer_loop(); });
        accept_thread = std::thread([this] { accept_loop(); });
        time_thread = std::thread([this] { time_loop(); });
    }

    void stop()
    {
        if (!running.exchange(false)) {
            return;
        }
        close_client();
        if (listen_fd >= 0) {
            ::shutdown(listen_fd, SHUT_RDWR);
            ::close(listen_fd);
            listen_fd = -1;
        }
        queue_cv.notify_all();
        if (accept_thread.joinable()) {
            accept_thread.join();
        }
        if (writer_thread.joinable()) {
            writer_thread.join();
        }
        if (time_thread.joinable()) {
            time_thread.join();
        }
        std::lock_guard<std::mutex> lock(queue_mu);
        priority_queue.clear();
        stereo_queue.clear();
    }

    void open_server(const std::string& address, int port)
    {
        listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd < 0) {
            throw std::runtime_error(label + ": socket() failed");
        }
        int one = 1;
        (void)::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in socket_address {};
        socket_address.sin_family = AF_INET;
        socket_address.sin_port = htons(static_cast<std::uint16_t>(port));
        if (::inet_pton(AF_INET, address.c_str(), &socket_address.sin_addr) != 1) {
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error(label + ": invalid listen address");
        }
        if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&socket_address),
                sizeof(socket_address)) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error(label + ": bind failed: " + error);
        }
        if (::listen(listen_fd, 1) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error(label + ": listen failed: " + error);
        }
    }

    void send_priority(sensors::MessageType type, std::vector<std::uint8_t> payload)
    {
        if (!running) {
            return;
        }
        enqueue_priority(sensors::encode_frame(type, ++sequence,
            to_ns(Clock::now()), std::move(payload)));
    }

    void send_frame_priority(sensors::MessageType type, std::vector<std::uint8_t> payload,
        std::int64_t device_time_ns)
    {
        if (!running) {
            return;
        }
        enqueue_priority(sensors::encode_frame(type, ++sequence, device_time_ns,
            std::move(payload)));
    }

    void send_stereo(std::uint32_t width, std::uint32_t height,
        const std::uint8_t* jpeg, std::size_t size, std::int64_t device_time_ns)
    {
        if (!running) {
            return;
        }
        enqueue_stereo(sensors::encode_frame(sensors::MessageType::kStereoJpeg,
            ++sequence, device_time_ns,
            sensors::encode_stereo_jpeg(width, height, jpeg, size)));
    }

    void enqueue_priority(std::vector<std::uint8_t> frame)
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        if (client_fd < 0) {
            ++counters.priority_dropped;
            return;
        }
        if (priority_queue.size() >= 1024) {
            priority_queue.pop_front();
            ++counters.priority_dropped;
        }
        priority_queue.push_back(std::move(frame));
        queue_cv.notify_one();
    }

    void enqueue_stereo(std::vector<std::uint8_t> frame)
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        if (client_fd < 0) {
            ++counters.stereo_dropped;
            return;
        }
        if (stereo_queue.size() >= 4) {
            stereo_queue.pop_front();
            ++counters.stereo_dropped;
        }
        stereo_queue.push_back(std::move(frame));
        queue_cv.notify_one();
    }

    void writer_loop()
    {
        while (running) {
            std::vector<std::uint8_t> frame;
            int fd = -1;
            bool stereo = false;
            {
                std::unique_lock<std::mutex> lock(queue_mu);
                queue_cv.wait_for(lock, std::chrono::milliseconds(200), [this] {
                    return !running || !priority_queue.empty() || !stereo_queue.empty();
                });
                if (!running) {
                    break;
                }
                if (!priority_queue.empty()) {
                    frame = std::move(priority_queue.front());
                    priority_queue.pop_front();
                } else if (!stereo_queue.empty()) {
                    frame = std::move(stereo_queue.front());
                    stereo_queue.pop_front();
                    stereo = true;
                }
                fd = client_fd;
                if (fd < 0) {
                    if (!frame.empty()) {
                        if (stereo) {
                            ++counters.stereo_dropped;
                        } else {
                            ++counters.priority_dropped;
                        }
                    }
                    continue;
                }
            }
            if (frame.empty()) {
                continue;
            }
            if (!send_all(fd, frame)) {
                close_client(fd);
                std::lock_guard<std::mutex> lock(queue_mu);
                if (stereo) {
                    ++counters.stereo_dropped;
                } else {
                    ++counters.priority_dropped;
                }
                continue;
            }
            std::lock_guard<std::mutex> lock(queue_mu);
            if (stereo) {
                ++counters.stereo_sent;
            } else {
                ++counters.priority_sent;
            }
        }
    }

    void accept_loop()
    {
        while (running) {
            pollfd descriptor {};
            descriptor.fd = listen_fd;
            descriptor.events = POLLIN;
            const int ready = ::poll(&descriptor, 1, 500);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            if (ready == 0 || (descriptor.revents & POLLIN) == 0) {
                continue;
            }
            sockaddr_in remote {};
            socklen_t length = sizeof(remote);
            const int fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&remote), &length);
            if (fd < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            int one = 1;
            (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            {
                std::lock_guard<std::mutex> lock(queue_mu);
                if (client_fd >= 0) {
                    ::shutdown(client_fd, SHUT_RDWR);
                    ::close(client_fd);
                }
                client_fd = fd;
                priority_queue.clear();
                stereo_queue.clear();
                counters.connected = true;
            }
            std::cerr << label << ": host connected\n";
        }
    }

    void time_loop()
    {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!running) {
                break;
            }
            if (connected()) {
                send_priority(sensors::MessageType::kTime, {});
            }
        }
    }

    void close_client(int expected_fd = -1)
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        if (client_fd >= 0 && (expected_fd < 0 || client_fd == expected_fd)) {
            ::shutdown(client_fd, SHUT_RDWR);
            ::close(client_fd);
            client_fd = -1;
            counters.connected = false;
        }
    }

    bool connected() const
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        return client_fd >= 0;
    }

    SensorStreamStats stats() const
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        SensorStreamStats copy = counters;
        copy.connected = client_fd >= 0;
        return copy;
    }

    std::string label;
    int listen_fd = -1;
    int client_fd = -1;
    std::atomic<bool> running { false };
    std::atomic<std::uint64_t> sequence { 0 };
    SensorStreamStats counters;

    std::thread accept_thread;
    std::thread writer_thread;
    std::thread time_thread;

    mutable std::mutex queue_mu;
    std::condition_variable queue_cv;
    std::deque<std::vector<std::uint8_t>> priority_queue;
    std::deque<std::vector<std::uint8_t>> stereo_queue;
};

SensorStreamServer::SensorStreamServer(std::string label)
    : impl_(std::make_unique<Impl>(std::move(label)))
{
}

SensorStreamServer::~SensorStreamServer() = default;

void SensorStreamServer::start(const std::string& address, int port)
{
    impl_->start(address, port);
}

void SensorStreamServer::stop()
{
    impl_->stop();
}

bool SensorStreamServer::running() const
{
    return impl_->running;
}

bool SensorStreamServer::connected() const
{
    return impl_->connected();
}

void SensorStreamServer::send_imu(const sensors::ImuPayload& payload,
    std::int64_t device_time_ns)
{
    if (!impl_->connected()) {
        return;
    }
    impl_->send_frame_priority(sensors::MessageType::kImu,
        sensors::encode_imu(payload), device_time_ns);
}

void SensorStreamServer::send_tof(const sensors::TofPayload& payload,
    std::int64_t device_time_ns)
{
    if (!impl_->connected()) {
        return;
    }
    impl_->send_frame_priority(sensors::MessageType::kTof,
        sensors::encode_tof(payload), device_time_ns);
}

void SensorStreamServer::send_stereo(std::uint32_t width, std::uint32_t height,
    const std::uint8_t* jpeg, std::size_t size, std::int64_t device_time_ns)
{
    if (!impl_->connected()) {
        return;
    }
    impl_->send_stereo(width, height, jpeg, size, device_time_ns);
}

SensorStreamStats SensorStreamServer::stats() const
{
    return impl_->stats();
}

} // namespace flyrat::pilot
