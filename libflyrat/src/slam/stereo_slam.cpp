// SPDX-License-Identifier: GPL-3.0-or-later

#include "flyrat/slam/stereo_slam.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <utility>

#include <Eigen/StdVector>
#include <opencv2/imgproc.hpp>

#include "estimator/estimator.h"
#include "estimator/parameters.h"

namespace flyrat::slam {

namespace {

std::atomic<bool> g_estimator_in_use { false };

SlamError make_error(const std::string& prefix, const std::exception& error)
{
    return SlamError(prefix + ": " + error.what());
}

cv::Mat to_gray(const cv::Mat& image)
{
    if (image.empty()) {
        return {};
    }
    if (image.type() == CV_8UC1) {
        // VINS keeps prev_img as a shallow reference for optical flow, so the
        // adapter owns a copy and callers may safely reuse their buffers.
        return image.clone();
    }
    cv::Mat gray;
    if (image.type() == CV_8UC3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.type() == CV_8UC4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        throw SlamError("SLAM images must be 8-bit mono, BGR or BGRA");
    }
    return gray;
}

} // namespace

class StereoSlam::Impl {
public:
    explicit Impl(Options options)
        : config_path_(std::move(options.config_path))
    {
        if (config_path_.empty()) {
            throw SlamError("VINS config path is empty");
        }
        if (g_estimator_in_use.exchange(true)) {
            throw SlamError(
                "only one StereoSlam instance may be alive per process "
                "(VINS-Fusion uses global parameters)");
        }

        try {
            readParameters(config_path_);
            if (!USE_IMU) {
                throw SlamError("VINS config must enable IMU (imu: 1)");
            }
            if (!STEREO || NUM_OF_CAM != 2) {
                throw SlamError("VINS config must use two cameras (num_of_cam: 2)");
            }
            // The adapter feeds measurements synchronously. Running VINS's
            // background consumer thread would make shutdown and reset
            // non-deterministic.
            MULTIPLE_THREAD = 0;
            estimator_ = std::make_unique<Estimator>();
            estimator_->setParameter();
        } catch (const SlamError&) {
            g_estimator_in_use.store(false);
            throw;
        } catch (const std::exception& error) {
            g_estimator_in_use.store(false);
            throw make_error("failed to initialize VINS-Fusion", error);
        }
    }

    ~Impl()
    {
        shutdown();
    }

    Result track(const cv::Mat& left, const cv::Mat& right, double timestamp,
        const std::vector<ImuSample>& imu)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!estimator_) {
            throw SlamError("StereoSlam has been shut down");
        }
        if (!std::isfinite(timestamp)) {
            throw SlamError("SLAM timestamp is not finite");
        }
        if (last_frame_timestamp_ >= 0.0 && timestamp <= last_frame_timestamp_) {
            throw SlamError("SLAM timestamps must advance");
        }
        if (imu.empty()) {
            throw SlamError("at least one IMU sample is required per frame");
        }

        const cv::Mat gray_left = to_gray(left);
        const cv::Mat gray_right = to_gray(right);
        if (gray_left.empty() || gray_right.empty()) {
            throw SlamError("SLAM received an empty stereo image");
        }
        if (gray_left.size() != gray_right.size()) {
            throw SlamError("left/right SLAM image sizes do not match");
        }
        if (gray_left.cols != COL || gray_left.rows != ROW) {
            throw SlamError("SLAM image size does not match VINS config");
        }

        std::vector<ImuSample> samples = imu;
        std::sort(samples.begin(), samples.end(),
            [](const ImuSample& a, const ImuSample& b) {
                return a.timestamp < b.timestamp;
            });
        for (const ImuSample& sample : samples) {
            if (!std::isfinite(sample.timestamp) ||
                !sample.accel.allFinite() || !sample.gyro.allFinite()) {
                throw SlamError("SLAM received a non-finite IMU sample");
            }
            estimator_->inputIMU(sample.timestamp, sample.accel, sample.gyro);
        }
        estimator_->inputImage(timestamp, gray_left, gray_right);
        last_frame_timestamp_ = timestamp;

        return make_result(timestamp);
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!estimator_) {
            throw SlamError("StereoSlam has been shut down");
        }
        estimator_->clearState();
        estimator_->setParameter();
        was_tracking_ = false;
        last_frame_timestamp_ = -1.0;
        trajectory_.clear();
    }

    void shutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (estimator_) {
            estimator_.reset();
            g_estimator_in_use.store(false);
        }
    }

    void saveTrajectory(const std::string& path) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path.empty()) {
            throw SlamError("trajectory path is empty");
        }
        std::ofstream output(path);
        if (!output) {
            throw SlamError("cannot open trajectory file: " + path);
        }
        output << std::setprecision(12);
        for (const TrajectorySample& sample : trajectory_) {
            const Eigen::Quaterniond q(sample.pose.block<3, 3>(0, 0));
            output << sample.timestamp << ' '
                   << sample.pose(0, 3) << ' '
                   << sample.pose(1, 3) << ' '
                   << sample.pose(2, 3) << ' '
                   << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
                   << '\n';
        }
    }

private:
    struct TrajectorySample {
        double timestamp = 0.0;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    };

    Result make_result(double timestamp)
    {
        Result result;
        result.timestamp = timestamp;

        if (estimator_->solver_flag == Estimator::NON_LINEAR &&
            estimator_->frame_count >= 0 && estimator_->frame_count <= WINDOW_SIZE) {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            estimator_->getPoseInWorldFrame(T);
            const Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);
            const Eigen::Vector3d translation = T.block<3, 1>(0, 3);
            if (rotation.allFinite() && translation.allFinite() &&
                (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-3) {
                Eigen::Quaterniond q(rotation);
                q.normalize();
                result.state = State::kTracking;
                result.pose.position = translation;
                result.pose.orientation = q;
                result.pose.valid = true;
                was_tracking_ = true;

                TrajectorySample sample;
                sample.timestamp = timestamp;
                sample.pose = T;
                trajectory_.push_back(sample);
                return result;
            }
        }

        if (was_tracking_ && estimator_->solver_flag == Estimator::INITIAL) {
            result.state = State::kLost;
            was_tracking_ = false;
        } else {
            result.state = State::kNotInitialized;
        }
        return result;
    }

    std::string config_path_;
    std::unique_ptr<Estimator> estimator_;
    bool was_tracking_ = false;
    double last_frame_timestamp_ = -1.0;
    std::vector<TrajectorySample, Eigen::aligned_allocator<TrajectorySample>> trajectory_;
    mutable std::mutex mutex_;
};

StereoSlam::StereoSlam(Options options)
    : impl_(std::make_unique<Impl>(std::move(options)))
{
}

StereoSlam::~StereoSlam() = default;

Result StereoSlam::track(const cv::Mat& left, const cv::Mat& right,
    double timestamp, const std::vector<ImuSample>& imu)
{
    return impl_->track(left, right, timestamp, imu);
}

void StereoSlam::reset()
{
    impl_->reset();
}

void StereoSlam::shutdown()
{
    impl_->shutdown();
}

void StereoSlam::saveTrajectory(const std::string& path) const
{
    impl_->saveTrajectory(path);
}

} // namespace flyrat::slam
