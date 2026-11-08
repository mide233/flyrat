// SPDX-License-Identifier: MIT
#include "flyrat/tio/tio_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <torch/script.h>

namespace flyrat::tio {

namespace {

constexpr int kChannels = 6;

Eigen::Quaterniond expSo3(const Eigen::Vector3d& omega)
{
    const double angle = omega.norm();
    if (angle < 1e-12) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega / angle));
}

} // namespace

struct TioOdometry::Impl {
    explicit Impl(Options value)
        : options(std::move(value))
    {
    }

    Options options;
    torch::jit::script::Module module;
    mutable std::mutex mutex;
    bool shutdown = false;
    bool started = false;

    std::vector<odometry::ImuSample> imu_buffer;
    std::deque<std::array<float, 4 * 64>> tof_window;
    std::deque<std::array<float, 4 * 64>> weight_window;
    std::deque<std::array<float, 16 * kChannels>> imu_window;

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double last_tof_stamp = 0.0;
    bool have_last_tof = false;

    struct TrajectorySample {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };
    std::vector<TrajectorySample> trajectory;

    std::array<float, 16 * kChannels> makeImuWindow_locked(double timestamp)
    {
        std::array<float, 16 * kChannels> window {};
        const int samples = options.imu_samples;
        if (samples <= 0) {
            return window;
        }
        const double period = 1.0 / std::max(options.tof_rate_hz, 1e-6);
        const double start = have_last_tof ? last_tof_stamp : timestamp - period;
        if (imu_buffer.empty()) {
            return window;
        }

        const auto sampleAt = [&](double time) {
            if (time <= imu_buffer.front().timestamp) {
                return imu_buffer.front();
            }
            if (time >= imu_buffer.back().timestamp) {
                return imu_buffer.back();
            }
            auto next = std::lower_bound(imu_buffer.begin(), imu_buffer.end(), time,
                [](const odometry::ImuSample& sample, double value) {
                    return sample.timestamp < value;
                });
            if (next == imu_buffer.begin()) {
                return *next;
            }
            const auto previous = std::prev(next);
            const double span = next->timestamp - previous->timestamp;
            const double alpha = span > 1e-9 ? (time - previous->timestamp) / span : 0.0;
            odometry::ImuSample out = *previous;
            out.timestamp = time;
            out.accel = previous->accel + alpha * (next->accel - previous->accel);
            out.gyro = previous->gyro + alpha * (next->gyro - previous->gyro);
            return out;
        };

        const double span = std::max(timestamp - start, 1e-6);
        for (int index = 0; index < samples; ++index) {
            const double alpha = samples > 1
                ? static_cast<double>(index) / static_cast<double>(samples - 1)
                : 1.0;
            const odometry::ImuSample sample = sampleAt(start + alpha * span);
            float* row = window.data() + index * kChannels;
            for (int axis = 0; axis < 3; ++axis) {
                row[axis] = static_cast<float>(sample.accel[axis]);
                row[3 + axis] = static_cast<float>(sample.gyro[axis]);
            }
        }
        return window;
    }

    torch::Tensor tensorFromWindow(
        const std::deque<std::array<float, 4 * 64>>& window, int views, int grid) const
    {
        const int frames = static_cast<int>(window.size());
        const int cells = views * grid * grid;
        auto tensor = torch::zeros(
            { 1, options.window_frames, views, grid, grid },
            torch::TensorOptions().dtype(torch::kFloat32));
        float* destination = tensor.data_ptr<float>();
        const int offset = (options.window_frames - frames) * cells;
        for (int frame = 0; frame < frames; ++frame) {
            std::memcpy(destination + offset + frame * cells, window[frame].data(),
                static_cast<std::size_t>(cells) * sizeof(float));
        }
        return tensor;
    }

    torch::Tensor shiftedTensor(const torch::Tensor& input) const
    {
        auto output = torch::zeros_like(input);
        output.index_put_({ 0, 0 }, input.index({ 0, 0 }));
        if (input.size(1) > 1) {
            output.index_put_({ 0, torch::indexing::Slice(1, torch::indexing::None) },
                input.index({ 0, torch::indexing::Slice(0, -1) }));
        }
        return output;
    }

    torch::Tensor imuTensor() const
    {
        const int frames = static_cast<int>(imu_window.size());
        const int values = options.imu_samples * kChannels;
        auto tensor = torch::zeros(
            { 1, options.window_frames, options.imu_samples, kChannels },
            torch::TensorOptions().dtype(torch::kFloat32));
        float* destination = tensor.data_ptr<float>();
        const int offset = (options.window_frames - frames) * values;
        for (int frame = 0; frame < frames; ++frame) {
            std::memcpy(destination + offset + frame * values, imu_window[frame].data(),
                static_cast<std::size_t>(values) * sizeof(float));
        }
        return tensor;
    }
};

TioOdometry::TioOdometry(Options options)
    : impl_(std::make_unique<Impl>(std::move(options)))
{
    if (impl_->options.model_path.empty()) {
        throw odometry::OdometryError("TIO model_path is empty");
    }
    if (impl_->options.views != 4 ||
        impl_->options.grid_size * impl_->options.grid_size != 64) {
        throw odometry::OdometryError("TIO backend requires four 8x8 views");
    }
    if (impl_->options.imu_samples != 16) {
        throw odometry::OdometryError("TIO backend requires 16 IMU samples per ToF frame");
    }
    if (impl_->options.warmup_frames < 1 ||
        impl_->options.warmup_frames > impl_->options.window_frames) {
        throw odometry::OdometryError(
            "TIO warmup_frames must be in [1, window_frames]");
    }
    try {
        impl_->module = torch::jit::load(impl_->options.model_path);
        impl_->module.eval();
    } catch (const std::exception& error) {
        throw odometry::OdometryError(
            "cannot load TIO TorchScript model " + impl_->options.model_path + ": " + error.what());
    }
}

TioOdometry::~TioOdometry() = default;

void TioOdometry::addImu(const odometry::ImuSample& sample)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->shutdown) {
        return;
    }
    // The interpolation below assumes a sorted buffer.  Drop malformed or
    // out-of-order samples instead of silently corrupting the IMU window.
    if (!std::isfinite(sample.timestamp) || !sample.accel.allFinite() ||
        !sample.gyro.allFinite()) {
        return;
    }
    if (!impl_->imu_buffer.empty() &&
        sample.timestamp < impl_->imu_buffer.back().timestamp) {
        return;
    }
    impl_->imu_buffer.push_back(sample);
    const double cutoff = sample.timestamp - impl_->options.imu_buffer_seconds;
    while (!impl_->imu_buffer.empty() && impl_->imu_buffer.front().timestamp < cutoff) {
        impl_->imu_buffer.erase(impl_->imu_buffer.begin());
    }
}

odometry::Result TioOdometry::track(const std::vector<TofGrid>& views, double timestamp)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->shutdown) {
        throw odometry::OdometryError("TIO odometry has been shut down");
    }
    if (views.size() != static_cast<std::size_t>(impl_->options.views)) {
        throw odometry::OdometryError("TIO track expects four ToF views");
    }
    if (!std::isfinite(timestamp)) {
        throw odometry::OdometryError("TIO timestamp is not finite");
    }
    if (impl_->have_last_tof && timestamp <= impl_->last_tof_stamp) {
        throw odometry::OdometryError("TIO timestamps must be strictly increasing");
    }

    std::array<float, 4 * 64> range {};
    std::array<float, 4 * 64> weight_input {};
    for (int view = 0; view < 4; ++view) {
        std::memcpy(range.data() + view * 64, views[view].range.data(), 64 * sizeof(float));
        std::memcpy(weight_input.data() + view * 64, views[view].weight.data(), 64 * sizeof(float));
    }
    impl_->tof_window.push_back(range);
    impl_->weight_window.push_back(weight_input);
    impl_->imu_window.push_back(impl_->makeImuWindow_locked(timestamp));
    while (static_cast<int>(impl_->tof_window.size()) > impl_->options.window_frames) {
        impl_->tof_window.pop_front();
        impl_->weight_window.pop_front();
        impl_->imu_window.pop_front();
    }
    impl_->last_tof_stamp = timestamp;
    impl_->have_last_tof = true;

    odometry::Result result;
    result.timestamp = timestamp;
    result.state = impl_->started ? odometry::State::kTracking
                                  : odometry::State::kNotInitialized;
    if (static_cast<int>(impl_->tof_window.size()) < impl_->options.warmup_frames) {
        return result;
    }

    const auto tof = impl_->tensorFromWindow(impl_->tof_window, 4, 8);
    const auto previous_tof = impl_->shiftedTensor(tof);
    const auto weight = impl_->tensorFromWindow(impl_->weight_window, 4, 8);
    const auto previous_weight = impl_->shiftedTensor(weight);
    const auto imu = impl_->imuTensor();

    torch::Tensor delta_p;
    torch::Tensor delta_phi;
    try {
        torch::NoGradGuard no_grad;
        const auto output = impl_->module.forward({ tof, imu, weight, previous_tof, previous_weight });
        const auto tuple = output.toTuple();
        delta_p = tuple->elements()[0].toTensor();
        delta_phi = tuple->elements()[1].toTensor();
    } catch (const std::exception& error) {
        throw odometry::OdometryError(std::string("TIO inference failed: ") + error.what());
    }

    const int last = static_cast<int>(delta_p.size(1)) - 1;
    if (last < 0) {
        throw odometry::OdometryError("TIO model returned an empty sequence");
    }
    Eigen::Vector3d dp;
    Eigen::Vector3d dphi;
    for (int axis = 0; axis < 3; ++axis) {
        dp[axis] = delta_p[0][last][axis].item<float>();
        dphi[axis] = delta_phi[0][last][axis].item<float>();
    }
    if (!dp.allFinite() || !dphi.allFinite()) {
        result.state = odometry::State::kLost;
        return result;
    }

    if (!impl_->started) {
        impl_->started = true;
        impl_->position.setZero();
        impl_->rotation.setIdentity();
    } else {
        const Eigen::Matrix3d previous_rotation = impl_->rotation;
        impl_->position += previous_rotation * dp;
        impl_->rotation = impl_->rotation * expSo3(dphi).toRotationMatrix();
    }

    result.state = odometry::State::kTracking;
    result.pose.position = impl_->position;
    result.pose.orientation = Eigen::Quaterniond(impl_->rotation).normalized();
    result.pose.valid = true;
    impl_->trajectory.push_back({ timestamp, result.pose.position, result.pose.orientation });
    return result;
}

void TioOdometry::reset()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->started = false;
    impl_->have_last_tof = false;
    impl_->position.setZero();
    impl_->rotation.setIdentity();
    impl_->imu_buffer.clear();
    impl_->tof_window.clear();
    impl_->weight_window.clear();
    impl_->imu_window.clear();
    impl_->trajectory.clear();
}

void TioOdometry::shutdown()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->shutdown = true;
}

void TioOdometry::saveTrajectory(const std::string& path) const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::ofstream stream(path);
    if (!stream) {
        throw odometry::OdometryError("cannot open trajectory file: " + path);
    }
    for (const auto& sample : impl_->trajectory) {
        stream << sample.timestamp << ' ' << sample.position.x() << ' '
               << sample.position.y() << ' ' << sample.position.z() << ' '
               << sample.orientation.x() << ' ' << sample.orientation.y() << ' '
               << sample.orientation.z() << ' ' << sample.orientation.w() << '\n';
    }
}

} // namespace flyrat::tio
