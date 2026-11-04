// SPDX-License-Identifier: GPL-3.0-or-later
#include "flyrat/slam/stereo_slam.hpp"

#include <cstdio>
#include <mutex>
#include <utility>

#include <System.h>
#include <Tracking.h>

namespace flyrat::slam {

namespace {

State map_state(int state)
{
    switch (state) {
    case ORB_SLAM3::Tracking::OK:
    case ORB_SLAM3::Tracking::OK_KLT:
        return State::kTracking;
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
        return State::kRecentlyLost;
    case ORB_SLAM3::Tracking::LOST:
        return State::kLost;
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
    default:
        return State::kNotInitialized;
    }
}

} // namespace

struct StereoSlam::Impl {
    Options options;
    std::unique_ptr<ORB_SLAM3::System> system;
    std::mutex mutex;
    bool shut_down = false;

    Result track(const cv::Mat& left, const cv::Mat& right, double timestamp_s,
        const std::vector<ImuSample>* imu)
    {
        if (shut_down) {
            throw SlamError("track() called after shutdown()");
        }
        if (left.empty() || right.empty()) {
            throw SlamError("track() received an empty stereo image");
        }
        if (left.channels() != 1 && left.channels() != 3 && left.channels() != 4) {
            throw SlamError("left image must have 1, 3 or 4 channels");
        }
        if (right.channels() != 1 && right.channels() != 3 && right.channels() != 4) {
            throw SlamError("right image must have 1, 3 or 4 channels");
        }
        if (left.size() != right.size()) {
            throw SlamError("left and right image sizes differ");
        }

        std::vector<ORB_SLAM3::IMU::Point> points;
        if (imu != nullptr && options.mode == Mode::kImuStereo) {
            points.reserve(imu->size());
            for (const ImuSample& sample : *imu) {
                points.emplace_back(
                    static_cast<float>(sample.accel.x()),
                    static_cast<float>(sample.accel.y()),
                    static_cast<float>(sample.accel.z()),
                    static_cast<float>(sample.gyro.x()),
                    static_cast<float>(sample.gyro.y()),
                    static_cast<float>(sample.gyro.z()),
                    sample.timestamp);
            }
        }

        const Sophus::SE3f Tcw = system->TrackStereo(left, right, timestamp_s, points);

        Result result;
        result.timestamp = timestamp_s;
        result.state = map_state(system->GetTrackingState());
        const bool tracked = result.state == State::kTracking;
        result.pose.valid = tracked;

        if (tracked) {
            const Sophus::SE3f Twc = Tcw.inverse();
            const Eigen::Matrix3f R = Twc.rotationMatrix();
            const Eigen::Vector3f t = Twc.translation();
            if (!R.allFinite() || !t.allFinite()) {
                result.pose.valid = false;
            } else {
                result.pose.position = Eigen::Vector3d(
                    static_cast<double>(t.x()),
                    static_cast<double>(t.y()),
                    static_cast<double>(t.z()));
                const Eigen::Matrix3d Rd = R.cast<double>();
                result.pose.orientation = Eigen::Quaterniond(Rd).normalized();
            }
        }
        return result;
    }
};

StereoSlam::StereoSlam(const Options& options)
    : impl_(std::make_unique<Impl>())
{
    if (options.vocabulary_path.empty()) {
        throw SlamError("SLAM vocabulary path is empty");
    }
    if (options.settings_path.empty()) {
        throw SlamError("SLAM settings path is empty");
    }

    // The vocabulary loader returns false without saying why; checking the file
    // up front gives a useful message and avoids a 139 MB load attempt on a
    // typo. The settings file is a YAML document, not a FileStorage archive, so
    // only existence is checked here.
    {
        FILE* file = std::fopen(options.vocabulary_path.c_str(), "rb");
        if (file == nullptr) {
            throw SlamError("cannot open SLAM vocabulary: " + options.vocabulary_path);
        }
        std::fclose(file);
    }
    {
        FILE* file = std::fopen(options.settings_path.c_str(), "rb");
        if (file == nullptr) {
            throw SlamError("cannot open SLAM settings: " + options.settings_path);
        }
        std::fclose(file);
    }

    impl_->options = options;
    const ORB_SLAM3::System::eSensor sensor = options.mode == Mode::kImuStereo
        ? ORB_SLAM3::System::IMU_STEREO
        : ORB_SLAM3::System::STEREO;
    try {
        impl_->system = std::make_unique<ORB_SLAM3::System>(
            options.vocabulary_path, options.settings_path, sensor, false, 0, "flyrat");
    } catch (const std::exception& error) {
        throw SlamError(std::string("failed to initialise ORB-SLAM3: ") + error.what());
    }
}

StereoSlam::~StereoSlam()
{
    try {
        shutdown();
    } catch (...) {
        // Destructors must not throw.
    }
}

Result StereoSlam::track(const cv::Mat& left, const cv::Mat& right, double timestamp_s)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->track(left, right, timestamp_s, nullptr);
}

Result StereoSlam::track(const cv::Mat& left, const cv::Mat& right, double timestamp_s,
    const std::vector<ImuSample>& imu)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->track(left, right, timestamp_s, &imu);
}

void StereoSlam::reset()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->shut_down) {
        return;
    }
    impl_->system->Reset();
}

void StereoSlam::shutdown()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->shut_down) {
        return;
    }
    impl_->shut_down = true;
    if (impl_->system) {
        impl_->system->Shutdown();
    }
}

void StereoSlam::saveTrajectory(const std::string& path)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->shut_down) {
        throw SlamError("saveTrajectory() requires shutdown() first");
    }
    impl_->system->SaveTrajectoryTUM(path);
}

} // namespace flyrat::slam
