// SPDX-License-Identifier: MIT
#include "sensor_forwarder.hpp"

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

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/msp_interact.hpp"
#include "flyrat/sensors/protocol.hpp"

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

struct SensorForwarder::Impl {
    explicit Impl(SensorForwarderConfig cfg)
        : config(std::move(cfg))
    {
    }

    ~Impl()
    {
        stop();
    }

    void start()
    {
        if (running) {
            return;
        }
        open_server();
        start_sensors();
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
        if (camera) {
            camera->stop();
        }
        if (tof) {
            tof->stop();
        }
        if (msp) {
            msp->stop();
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
        camera_queue.clear();
    }

    void open_server()
    {
        listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd < 0) {
            throw std::runtime_error("sensor forwarder: socket() failed");
        }
        int one = 1;
        (void)::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in address {};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<std::uint16_t>(config.listen_port));
        if (::inet_pton(AF_INET, config.listen_address.c_str(), &address.sin_addr) != 1) {
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error("sensor forwarder: invalid listen address");
        }
        if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error("sensor forwarder: bind failed: " + error);
        }
        if (::listen(listen_fd, 1) < 0) {
            const std::string error = std::strerror(errno);
            ::close(listen_fd);
            listen_fd = -1;
            throw std::runtime_error("sensor forwarder: listen failed: " + error);
        }
    }

    void start_sensors()
    {
        if (config.enable_imu) {
            try {
                MspConfig msp_config;
                msp_config.poll = {};
                msp_config.poll.imu_hz = 200;
                auto transport = std::make_unique<hardware::SerialTransport>(
                    config.msp_device, config.msp_baud);
                msp = std::make_unique<MspInteract>(msp_config, std::move(transport));
                msp->set_imu_callback([this](const Sample<ImuSample>& sample) {
                    on_imu(sample);
                });
                msp->start();
            } catch (const std::exception& error) {
                std::cerr << "sensor forwarder: IMU disabled: " << error.what() << '\n';
                msp.reset();
            }
        }

        if (config.enable_tof) {
            try {
                std::cout << "sensor forwarder: ToF "
                          << config.tof.resolution << "x" << config.tof.resolution
                          << " @ " << config.tof.ranging_hz << " Hz\n";
                tof = std::make_unique<hardware::TofArray>(config.tof);
                tof->set_callback([this](const hardware::TofFrame& frame) {
                    on_tof(frame);
                });
                tof->start();
            } catch (const std::exception& error) {
                std::cerr << "sensor forwarder: ToF disabled: " << error.what() << '\n';
                tof.reset();
            }
        }

        if (config.enable_camera) {
            try {
                camera = std::make_unique<hardware::MonoJpegSource>(config.camera);
                camera->set_callback([this](const hardware::MonoJpegFrame& frame) {
                    on_camera(frame);
                });
                camera->start();
            } catch (const std::exception& error) {
                std::cerr << "sensor forwarder: camera disabled: " << error.what() << '\n';
                camera.reset();
            }
        }
    }

    void on_imu(const Sample<ImuSample>& sample)
    {
        sensors::ImuPayload payload;
        payload.acc = sample.value.acc;
        payload.gyro = sample.value.gyro;
        payload.mag = sample.value.mag;
        enqueue_priority(sensors::encode_frame(sensors::MessageType::kImu,
            ++sequence, to_ns(sample.stamp), sensors::encode_imu(payload)));
    }

    void on_tof(const hardware::TofFrame& frame)
    {
        sensors::TofPayload payload;
        payload.sensor_id = static_cast<std::uint8_t>(frame.sensor);
        payload.resolution = static_cast<std::uint8_t>(frame.resolution);
        payload.temperature_c = frame.silicon_temp_degc;
        for (std::size_t i = 0; i < payload.zones.size(); ++i) {
            payload.zones[i].distance_mm = frame.zones[i].distance_mm;
            payload.zones[i].target_status = frame.zones[i].target_status;
            payload.zones[i].nb_target = frame.zones[i].nb_target;
        }
        enqueue_priority(sensors::encode_frame(sensors::MessageType::kTof,
            ++sequence, to_ns(frame.stamp), sensors::encode_tof(payload)));
    }

    void on_camera(const hardware::MonoJpegFrame& frame)
    {
        enqueue_camera(sensors::encode_frame(sensors::MessageType::kCameraJpeg,
            ++sequence, to_ns(frame.stamp),
            sensors::encode_jpeg(static_cast<std::uint32_t>(frame.width),
                static_cast<std::uint32_t>(frame.height),
                frame.jpeg.data(), frame.jpeg.size())));
    }

    void enqueue_priority(std::vector<std::uint8_t> frame)
    {
        {
            std::lock_guard<std::mutex> lock(queue_mu);
            if (priority_queue.size() >= 1024) {
                priority_queue.pop_front();
            }
            priority_queue.push_back(std::move(frame));
        }
        queue_cv.notify_one();
    }

    void enqueue_camera(std::vector<std::uint8_t> frame)
    {
        {
            std::lock_guard<std::mutex> lock(queue_mu);
            if (camera_queue.size() >= 4) {
                camera_queue.pop_front();
            }
            camera_queue.push_back(std::move(frame));
        }
        queue_cv.notify_one();
    }

    void writer_loop()
    {
        while (running) {
            std::vector<std::uint8_t> frame;
            int fd = -1;
            {
                std::unique_lock<std::mutex> lock(queue_mu);
                queue_cv.wait_for(lock, std::chrono::milliseconds(200), [this] {
                    return !running || !priority_queue.empty() || !camera_queue.empty();
                });
                if (!running) {
                    break;
                }
                if (!priority_queue.empty()) {
                    frame = std::move(priority_queue.front());
                    priority_queue.pop_front();
                } else if (!camera_queue.empty()) {
                    frame = std::move(camera_queue.front());
                    camera_queue.pop_front();
                }
                fd = client_fd;
            }
            if (fd < 0 || frame.empty()) {
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
                std::lock_guard<std::mutex> lock(queue_mu);
                if (client_fd >= 0) {
                    ::shutdown(client_fd, SHUT_RDWR);
                    ::close(client_fd);
                }
                client_fd = fd;
                priority_queue.clear();
                camera_queue.clear();
            }
            std::cerr << "sensor forwarder: host connected\n";
        }
    }

    void time_loop()
    {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!running) {
                break;
            }
            enqueue_priority(sensors::encode_frame(sensors::MessageType::kTime,
                ++sequence, to_ns(Clock::now()), {}));
        }
    }

    void close_client(int expected_fd = -1)
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        if (client_fd >= 0 && (expected_fd < 0 || client_fd == expected_fd)) {
            ::shutdown(client_fd, SHUT_RDWR);
            ::close(client_fd);
            client_fd = -1;
        }
    }

    SensorForwarderConfig config;
    int listen_fd = -1;
    int client_fd = -1;
    std::atomic<bool> running { false };
    std::atomic<std::uint64_t> sequence { 0 };

    std::unique_ptr<MspInteract> msp;
    std::unique_ptr<hardware::TofArray> tof;
    std::unique_ptr<hardware::MonoJpegSource> camera;

    std::thread accept_thread;
    std::thread writer_thread;
    std::thread time_thread;

    std::mutex queue_mu;
    std::condition_variable queue_cv;
    std::deque<std::vector<std::uint8_t>> priority_queue;
    std::deque<std::vector<std::uint8_t>> camera_queue;
};

SensorForwarder::SensorForwarder(SensorForwarderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

SensorForwarder::~SensorForwarder() = default;

void SensorForwarder::start()
{
    impl_->start();
}

void SensorForwarder::stop()
{
    impl_->stop();
}

bool SensorForwarder::running() const
{
    return impl_->running;
}

} // namespace flyrat::pilot
