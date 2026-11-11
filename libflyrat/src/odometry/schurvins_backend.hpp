// SPDX-License-Identifier: MIT
//
// SchurVINS stereo-inertial backend. This header is private to
// flyrat_odometry; the public interface is flyrat/odometry/odometry.hpp.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "flyrat/odometry/odometry.hpp"

namespace flyrat::odometry {

// Owns the SVO/SchurVINS frontend and adapts it to the flyrat odometry
// contract. Only built when FLYRAT_WITH_SCHURVINS is enabled.
class SchurvinsBackend {
public:
    explicit SchurvinsBackend(const std::string& camera_config_path,
        const std::string& trace_dir = {});
    ~SchurvinsBackend();

    SchurvinsBackend(const SchurvinsBackend&) = delete;
    SchurvinsBackend& operator=(const SchurvinsBackend&) = delete;

    Result track(
        const ImageView& left, const ImageView& right, double timestamp,
        const std::vector<ImuSample>& imu);
    void reset();
    void saveTrajectory(const std::string& path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::odometry
