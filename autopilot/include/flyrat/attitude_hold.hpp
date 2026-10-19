// SPDX-License-Identifier: MIT
//
// Minimal example autopilot: attitude hold with rate damping.
//
// The outer-loop algorithms we actually care about will live alongside this
// one; it exists to exercise the interface end to end and to give the ROS 2
// bring-up something safe to fly.
#pragma once

#include "flyrat/autopilot.hpp"

namespace flyrat {

class AttitudeHold : public Autopilot {
public:
    struct Gains {
        // Stick deflection per radian of attitude error.
        double roll_kp = 1.0;
        double pitch_kp = 1.0;
        // Stick deflection per rad/s of yaw-rate error.
        double yaw_rate_kp = 0.5;
        // Deflection limits.
        double max_roll = 1.0;
        double max_pitch = 1.0;
        double max_yaw = 1.0;
    };

    AttitudeHold() = default;
    explicit AttitudeHold(const Gains& gains)
        : gains_(gains) {}

    void reset() override {}

    Command update(const State& state, const Setpoint& setpoint, double dt) override;

    const Gains& gains() const { return gains_; }
    Gains& gains() { return gains_; }

private:
    Gains gains_{};
};

} // namespace flyrat
