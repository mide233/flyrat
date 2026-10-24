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
        //
        // The flight controller is configured with a linear 400 deg/s rate
        // curve (rc_rate = 200, expo = 0, srate = 0), i.e. full stick is
        // 6.98 rad/s. One stick unit per radian would therefore command
        // 400 deg/s of rate per radian of error, which is far too hot; half a
        // stick per radian gives a well damped 200 deg/s.
        double roll_kp = 0.5;
        double pitch_kp = 0.5;
        // Stick deflection per rad/s of yaw-rate error.
        double yaw_rate_kp = 0.25;
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
