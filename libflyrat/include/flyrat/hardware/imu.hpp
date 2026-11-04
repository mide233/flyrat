// SPDX-License-Identifier: MIT
#pragma once

#include <chrono>
#include <optional>

#include "flyrat/msp_interact.hpp"

namespace flyrat::hardware {

// IMU facade for the production code. The MSP link polls MSP_RAW_IMU; this
// class never performs I/O itself, it only reads the latest sample and can
// reject stale data.
class Imu {
public:
    explicit Imu(const MspInteract& msp)
        : msp_(msp)
    {
    }

    std::optional<Sample<ImuSample>> read() const { return msp_.imu(); }

    std::optional<Sample<ImuSample>> read(std::chrono::milliseconds max_age) const
    {
        auto sample = msp_.imu();
        if (!sample) {
            return std::nullopt;
        }
        if (std::chrono::steady_clock::now() - sample->stamp > max_age) {
            return std::nullopt;
        }
        return sample;
    }

private:
    const MspInteract& msp_;
};

} // namespace flyrat::hardware
