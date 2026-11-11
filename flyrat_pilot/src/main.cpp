// SPDX-License-Identifier: MIT
//
// On-vehicle SLAM runtime.
//
// The pilot decodes the side-by-side MJPEG stream locally, converts the MSP IMU
// stream with the same calibration as the host sensor receiver, runs SchurVINS
// and streams body poses over TCP. The host bridge publishes them as ROS 2
// odometry, path and TF so Foxglove can display the vehicle pose.
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/hardware/stereo_jpeg_source.hpp"
#include "flyrat/msp_interact.hpp"
#include "flyrat/odometry/odometry.hpp"

#include "slam/imu_converter.hpp"
#include "slam/pose_stream.hpp"

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_stop { false };
std::atomic<bool> g_reset { false };

void handleSignal(int signal)
{
    if (signal == SIGINT || signal == SIGTERM) {
        g_stop = true;
    } else if (signal == SIGUSR1) {
        g_reset = true;
    }
}

struct Options {
    std::string camera_config
        = "/workspaces/flyrat/libflyrat/configs/camera/uvc_640x480.yaml";
    std::string imu_config
        = "/workspaces/flyrat/flyrat_play/ros2_ws/src/flyrat_sensors/config/"
          "imu_intrinsics.yaml";
    std::string msp_device = "/dev/ttyS3";
    int msp_baud = 921600;
    std::string camera_device = "/dev/video10";
    int width = 1280;
    int height = 480;
    int fps = 30;
    std::string pose_address = "127.0.0.1";
    int pose_port = 5001;
    std::string trajectory = "/tmp/flyrat_pilot.tum";
    double imu_lookahead = 0.015;
    double imu_history = 0.1;
    int max_frames = 0;
    bool check_config = false;
};

void printUsage(const char* program)
{
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --camera-config PATH   stereo calibration YAML\n"
        << "  --imu-config PATH      IMU scale/misalignment YAML\n"
        << "  --msp-device DEV       flight controller serial device\n"
        << "  --msp-baud BAUD        MSP baud rate (default 921600)\n"
        << "  --camera-device DEV    V4L2 MJPEG device (default /dev/video10)\n"
        << "  --width N              side-by-side width (default 1280)\n"
        << "  --height N             image height (default 480)\n"
        << "  --fps N                camera frame rate (default 30)\n"
        << "  --pose-address ADDR    pose stream listen address\n"
        << "  --pose-port PORT       pose stream port (default 5001)\n"
        << "  --trajectory PATH      TUM output, empty disables\n"
        << "  --imu-lookahead SEC    IMU samples after each frame (default 0.015)\n"
        << "  --imu-history SEC      IMU history for the first frame (default 0.1)\n"
        << "  --max-frames N         stop after N processed frames (testing)\n"
        << "  --check-config         validate calibration files and exit\n";
}

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            return argv[++i];
        };
        if (arg == "--camera-config") {
            options.camera_config = value("--camera-config");
        } else if (arg == "--imu-config") {
            options.imu_config = value("--imu-config");
        } else if (arg == "--msp-device") {
            options.msp_device = value("--msp-device");
        } else if (arg == "--msp-baud") {
            options.msp_baud = std::stoi(value("--msp-baud"));
        } else if (arg == "--camera-device") {
            options.camera_device = value("--camera-device");
        } else if (arg == "--width") {
            options.width = std::stoi(value("--width"));
        } else if (arg == "--height") {
            options.height = std::stoi(value("--height"));
        } else if (arg == "--fps") {
            options.fps = std::stoi(value("--fps"));
        } else if (arg == "--pose-address") {
            options.pose_address = value("--pose-address");
        } else if (arg == "--pose-port") {
            options.pose_port = std::stoi(value("--pose-port"));
        } else if (arg == "--trajectory") {
            options.trajectory = value("--trajectory");
        } else if (arg == "--imu-lookahead") {
            options.imu_lookahead = std::stod(value("--imu-lookahead"));
        } else if (arg == "--imu-history") {
            options.imu_history = std::stod(value("--imu-history"));
        } else if (arg == "--max-frames") {
            options.max_frames = std::stoi(value("--max-frames"));
        } else if (arg == "--check-config") {
            options.check_config = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    if (options.width <= 0 || options.height <= 0 || (options.width % 2) != 0) {
        throw std::runtime_error("width must be positive and even");
    }
    if (options.fps <= 0 || options.pose_port <= 0 || options.pose_port > 65535) {
        throw std::runtime_error("invalid fps or pose port");
    }
    if (options.imu_lookahead < 0.0 || options.imu_lookahead > 0.1
        || options.imu_history <= 0.0) {
        throw std::runtime_error("invalid IMU lookahead/history");
    }
    return options;
}

// Maps CLOCK_MONOTONIC timestamps (MSP and V4L2) to Unix-epoch seconds. The
// offset is sampled once: both clocks come from the same kernel oscillator, so
// drift over a flight is far below the camera/IMU tolerances.
class ClockMapper {
public:
    ClockMapper()
    {
        const auto now = Clock::now().time_since_epoch();
        const auto wall = std::chrono::system_clock::now().time_since_epoch();
        offset_s_ = std::chrono::duration<double>(wall - now).count();
    }

    double toEpoch(Clock::time_point stamp) const
    {
        return std::chrono::duration<double>(stamp.time_since_epoch()).count() + offset_s_;
    }

private:
    double offset_s_ = 0.0;
};

class ImuBuffer {
public:
    void push(const flyrat::odometry::ImuSample& sample)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            samples_.push_back(sample);
            while (!samples_.empty()
                && samples_.front().timestamp < sample.timestamp - 5.0) {
                samples_.pop_front();
            }
        }
        cv_.notify_all();
    }

    bool waitFor(double timestamp, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&] {
            return !samples_.empty() && samples_.back().timestamp > timestamp;
        });
    }

    std::vector<flyrat::odometry::ImuSample> collect(double begin, double end) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<flyrat::odometry::ImuSample> result;
        for (const auto& sample : samples_) {
            if (sample.timestamp > begin && sample.timestamp <= end) {
                result.push_back(sample);
            }
        }
        return result;
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return samples_.size();
    }

    double latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return samples_.empty() ? -1.0 : samples_.back().timestamp;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<flyrat::odometry::ImuSample> samples_;
};

struct FramePacket {
    cv::Mat left;
    cv::Mat right;
    double stamp_s = 0.0;
    std::uint64_t sequence = 0;
};

class FrameBuffer {
public:
    void push(FramePacket packet)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(packet);
            ++received_;
        }
        cv_.notify_all();
    }

    std::optional<FramePacket> takeLatest()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_) {
            return std::nullopt;
        }
        auto packet = std::move(*latest_);
        latest_.reset();
        return packet;
    }

    std::uint64_t received() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return received_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<FramePacket> latest_;
    std::uint64_t received_ = 0;
};

cv::Matx44d loadBodyCamera(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("cannot open camera calibration: " + path);
    }
    cv::Mat matrix;
    fs["imu"]["T_body_camera"] >> matrix;
    if (matrix.rows != 4 || matrix.cols != 4) {
        throw std::runtime_error("imu.T_body_camera must be a 4x4 matrix");
    }
    cv::Matx44d result;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            result(row, col) = matrix.at<double>(row, col);
        }
    }
    return result;
}

const char* stateName(flyrat::odometry::State state)
{
    switch (state) {
    case flyrat::odometry::State::kNotInitialized:
        return "not_initialized";
    case flyrat::odometry::State::kTracking:
        return "tracking";
    case flyrat::odometry::State::kRecentlyLost:
        return "recently_lost";
    case flyrat::odometry::State::kLost:
        return "lost";
    }
    return "unknown";
}

std::uint8_t stateCode(flyrat::odometry::State state)
{
    return static_cast<std::uint8_t>(state);
}

Eigen::Matrix4d toEigen(const cv::Matx44d& matrix)
{
    Eigen::Matrix4d result;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            result(row, col) = matrix(row, col);
        }
    }
    return result;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    std::signal(SIGUSR1, handleSignal);

    Options options;
    try {
        options = parseOptions(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "flyrat_pilot: " << error.what() << "\n";
        printUsage(argv[0]);
        return 2;
    }

    try {
        const ClockMapper clock;
        const auto intrinsics = flyrat::pilot::loadImuIntrinsics(options.imu_config);
        const Eigen::Matrix4d T_body_camera = toEigen(loadBodyCamera(options.camera_config));

        flyrat::odometry::Options odometry_options;
        odometry_options.mode = flyrat::odometry::Mode::kImuStereo;
        odometry_options.camera_config_path = options.camera_config;
        flyrat::odometry::Odometry odometry(odometry_options);

        if (options.check_config) {
            std::cout << "camera_config=" << options.camera_config << "\n"
                      << "imu_config=" << options.imu_config << "\n"
                      << "imu_intrinsics_enable=" << intrinsics.enable << "\n"
                      << "imu_time_offset=" << intrinsics.time_offset_s << "\n"
                      << "T_body_camera=\n"
                      << T_body_camera << "\n";
            return 0;
        }

        flyrat::pilot::PoseStreamConfig stream_config;
        stream_config.listen_address = options.pose_address;
        stream_config.listen_port = options.pose_port;
        flyrat::pilot::PoseStream pose_stream(stream_config);

        ImuBuffer imu_buffer;
        flyrat::MspConfig msp_config;
        msp_config.poll.imu_hz = 200;
        msp_config.poll.attitude_hz = 0;
        msp_config.poll.status_hz = 0;
        msp_config.poll.altitude_hz = 0;
        msp_config.poll.rc_hz = 0;
        msp_config.poll.analog_hz = 0;
        msp_config.poll.motor_hz = 0;
        msp_config.poll.gps_hz = 0;
        msp_config.poll.debug_hz = 0;

        flyrat::MspInteract msp(msp_config,
            std::make_unique<flyrat::hardware::SerialTransport>(
                options.msp_device, options.msp_baud));
        msp.set_imu_callback([&](const flyrat::Sample<flyrat::ImuSample>& sample) {
            flyrat::sensors::ImuPayload raw;
            for (std::size_t i = 0; i < 3; ++i) {
                raw.acc[i] = sample.value.acc[i];
                raw.gyro[i] = sample.value.gyro[i];
                raw.mag[i] = sample.value.mag[i];
            }
            const double stamp_s = clock.toEpoch(sample.stamp);
            imu_buffer.push(
                flyrat::pilot::convertImuSample(intrinsics, raw, stamp_s));
        });
        msp.set_link_callback([](bool connected, const std::string& why) {
            std::cerr << "msp: " << (connected ? "connected" : "disconnected")
                      << (why.empty() ? "" : " (" + why + ")") << "\n";
        });

        FrameBuffer frame_buffer;
        std::atomic<std::uint64_t> decode_errors { 0 };
        flyrat::hardware::StereoJpegConfig camera_config;
        camera_config.device = options.camera_device;
        camera_config.width = options.width;
        camera_config.height = options.height;
        camera_config.fps = options.fps;
        flyrat::hardware::StereoJpegSource camera(camera_config);
        camera.set_callback([&](const flyrat::hardware::StereoJpegFrame& frame) {
            cv::Mat encoded(1, static_cast<int>(frame.jpeg.size()), CV_8UC1,
                const_cast<std::uint8_t*>(frame.jpeg.data()));
            cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
            if (decoded.empty() || decoded.cols != options.width
                || decoded.rows != options.height) {
                ++decode_errors;
                return;
            }
            const int half = decoded.cols / 2;
            FramePacket packet;
            packet.left = decoded.colRange(0, half);
            packet.right = decoded.colRange(half, decoded.cols);
            packet.stamp_s = clock.toEpoch(frame.stamp);
            packet.sequence = frame.sequence;
            frame_buffer.push(std::move(packet));
        });

        msp.start();
        pose_stream.start();
        camera.start();

        std::cerr << "flyrat_pilot: SLAM started; pose stream on "
                  << options.pose_address << ":" << options.pose_port
                  << ", camera " << options.width << "x" << options.height
                  << "@" << options.fps << " fps\n";

        double last_frame_s = -1.0;
        double last_pose_s = -1.0;
        std::uint64_t processed = 0;
        std::uint64_t published = 0;
        std::uint64_t invalid = 0;
        std::uint64_t skipped = 0;
        auto last_report = Clock::now();

        while (!g_stop) {
            if (g_reset.exchange(false)) {
                odometry.reset();
                last_frame_s = -1.0;
                last_pose_s = -1.0;
                std::cerr << "flyrat_pilot: odometry reset\n";
            }

            auto frame = frame_buffer.takeLatest();
            if (!frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            if (frame->stamp_s <= last_frame_s + 1e-6) {
                continue;
            }
            const double now_s = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                                     .count();
            if (now_s - frame->stamp_s > 0.5) {
                ++skipped;
                continue;
            }

            const double end_s = frame->stamp_s + options.imu_lookahead;
            if (!imu_buffer.waitFor(end_s, std::chrono::milliseconds(100))) {
                ++skipped;
                last_frame_s = frame->stamp_s;
                continue;
            }
            const double begin_s = last_frame_s >= 0.0
                ? last_frame_s
                : frame->stamp_s - options.imu_history;
            auto imu = imu_buffer.collect(begin_s, end_s);
            if (imu.empty()) {
                ++skipped;
                last_frame_s = frame->stamp_s;
                continue;
            }
            const auto result = odometry.track(
                flyrat::odometry::ImageView { frame->left.cols, frame->left.rows,
                    static_cast<int>(frame->left.step),
                    frame->left.ptr<std::uint8_t>() },
                flyrat::odometry::ImageView { frame->right.cols, frame->right.rows,
                    static_cast<int>(frame->right.step),
                    frame->right.ptr<std::uint8_t>() },
                frame->stamp_s, imu);
            last_frame_s = frame->stamp_s;
            ++processed;

            flyrat::sensors::PosePayload payload;
            payload.state = stateCode(result.state);
            if (result.pose.valid) {
                Eigen::Matrix4d T_world_camera = Eigen::Matrix4d::Identity();
                T_world_camera.block<3, 3>(0, 0)
                    = result.pose.orientation.toRotationMatrix();
                T_world_camera.block<3, 1>(0, 3) = result.pose.position;
                const Eigen::Matrix4d T_world_body
                    = T_world_camera * T_body_camera.inverse();
                Eigen::Quaterniond orientation(T_world_body.block<3, 3>(0, 0));
                orientation.normalize();
                payload.position = { T_world_body(0, 3), T_world_body(1, 3),
                    T_world_body(2, 3) };
                payload.orientation = { orientation.x(), orientation.y(),
                    orientation.z(), orientation.w() };
                payload.valid = 1;
                last_pose_s = result.timestamp;
                ++published;
            } else {
                ++invalid;
            }

            pose_stream.publish(payload,
                static_cast<std::int64_t>(std::llround(
                    (result.pose.valid ? result.timestamp : frame->stamp_s) * 1e9)));

            const auto now = Clock::now();
            if (now - last_report >= std::chrono::seconds(2)) {
                last_report = now;
                std::cerr << "flyrat_pilot: state=" << stateName(result.state)
                          << " frames=" << processed
                          << " poses=" << published
                          << " invalid=" << invalid
                          << " skipped=" << skipped
                          << " imu=" << imu_buffer.size()
                          << " camera_drops=" << decode_errors.load()
                          << " t=" << last_pose_s << "\n";
            }

            if (options.max_frames > 0
                && processed >= static_cast<std::uint64_t>(options.max_frames)) {
                break;
            }
        }

        camera.stop();
        msp.stop();
        pose_stream.stop();
        if (!options.trajectory.empty()) {
            odometry.saveTrajectory(options.trajectory);
            std::cerr << "flyrat_pilot: saved trajectory to " << options.trajectory
                      << "\n";
        }
        std::cerr << "flyrat_pilot: stopped after " << processed << " frames\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "flyrat_pilot: " << error.what() << "\n";
        return 1;
    }
}
