// SPDX-License-Identifier: MIT
#include "pose_stream.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace flyrat::pilot {
namespace {

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

struct PoseStream::Impl {
    explicit Impl(PoseStreamConfig cfg)
        : config(std::move(cfg))
    {
    }

    ~Impl()
    {
        stop();
    }

    void start()
    {
        if (running.exchange(true)) {
            return;
        }
        open_server();
        accept_thread = std::thread([this] { accept_loop(); });
        writer_thread = std::thread([this] { writer_loop(); });
    }

    void stop()
    {
        if (!running.exchange(false)) {
            return;
        }
        if (listen_fd >= 0) {
            ::shutdown(listen_fd, SHUT_RDWR);
            ::close(listen_fd);
            listen_fd = -1;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (client_fd >= 0) {
                ::shutdown(client_fd, SHUT_RDWR);
                ::close(client_fd);
                client_fd = -1;
            }
        }
        cv.notify_all();
        if (accept_thread.joinable()) {
            accept_thread.join();
        }
        if (writer_thread.joinable()) {
            writer_thread.join();
        }
    }

    void open_server()
    {
        listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd < 0) {
            throw std::runtime_error("pose stream: socket() failed");
        }
        int one = 1;
        (void)::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in address {};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<std::uint16_t>(config.listen_port));
        if (::inet_pton(AF_INET, config.listen_address.c_str(), &address.sin_addr) != 1) {
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error("pose stream: invalid listen address");
        }
        if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error("pose stream: bind failed: " + error);
        }
        if (::listen(listen_fd, 1) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error("pose stream: listen failed: " + error);
        }
    }

    void publish(const sensors::PosePayload& pose, std::int64_t device_time_ns)
    {
        std::vector<std::uint8_t> frame = sensors::encode_frame(
            sensors::MessageType::kPose, ++sequence, device_time_ns,
            sensors::encode_pose(pose));
        {
            std::lock_guard<std::mutex> lock(mutex);
            latest = std::move(frame);
        }
        cv.notify_one();
    }

    void writer_loop()
    {
        while (running) {
            std::vector<std::uint8_t> frame;
            int fd = -1;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait_for(lock, std::chrono::milliseconds(200), [this] {
                    return !running || latest.has_value();
                });
                if (!running) {
                    break;
                }
                if (!latest) {
                    continue;
                }
                frame = std::move(*latest);
                latest.reset();
                fd = client_fd;
            }
            if (fd < 0) {
                continue;
            }
            if (!send_all(fd, frame)) {
                close_client(fd);
            }
        }
    }

    void accept_loop()
    {
        while (running) {
            pollfd pfd {};
            pfd.fd = listen_fd;
            pfd.events = POLLIN;
            const int ready = ::poll(&pfd, 1, 500);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            if (ready == 0 || (pfd.revents & POLLIN) == 0) {
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
                std::lock_guard<std::mutex> lock(mutex);
                if (client_fd >= 0) {
                    ::shutdown(client_fd, SHUT_RDWR);
                    ::close(client_fd);
                }
                client_fd = fd;
                latest.reset();
            }
            std::cerr << "pose stream: host connected\n";
        }
    }

    void close_client(int expected_fd)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (client_fd >= 0 && client_fd == expected_fd) {
            ::shutdown(client_fd, SHUT_RDWR);
            ::close(client_fd);
            client_fd = -1;
            std::cerr << "pose stream: host disconnected\n";
        }
    }

    PoseStreamConfig config;
    int listen_fd = -1;
    int client_fd = -1;
    std::atomic<bool> running { false };
    std::atomic<std::uint64_t> sequence { 0 };

    std::thread accept_thread;
    std::thread writer_thread;

    std::mutex mutex;
    std::condition_variable cv;
    std::optional<std::vector<std::uint8_t>> latest;
};

PoseStream::PoseStream(PoseStreamConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

PoseStream::~PoseStream() = default;

void PoseStream::start()
{
    impl_->start();
}

void PoseStream::stop()
{
    impl_->stop();
}

void PoseStream::publish(const sensors::PosePayload& pose, std::int64_t device_time_ns)
{
    impl_->publish(pose, device_time_ns);
}

} // namespace flyrat::pilot
