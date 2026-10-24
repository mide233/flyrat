// SPDX-License-Identifier: MIT
//
// Online identification of the thrust-to-acceleration gain.
//
// The flight controller maps a normalised throttle to an acceleration that
// depends on the battery, the propeller efficiency and the vehicle mass. The
// MPC needs the inverse of that gain to turn a thrust acceleration into a
// throttle command, so it is estimated online with a recursive least squares
// fit of   a_z = thrust_to_accel * throttle   with a forgetting factor.
//
// The caller is expected to feed the throttle that was sent one actuator delay
// ago (35-45 ms works well), so the estimate is not biased by the lag.
#pragma once

namespace flyrat {

class ThrustModel {
public:
    explicit ThrustModel(double hover_fraction = 0.705, double gravity = 9.80665)
        : gravity_(gravity)
        , thrust_to_accel_(hover_fraction > 1e-3 ? gravity / hover_fraction : gravity) { }

    double thrust_to_accel() const { return thrust_to_accel_; }
    double hover_fraction() const { return gravity_ / thrust_to_accel_; }

    void reset(double hover_fraction) {
        if (hover_fraction > 1e-3) {
            thrust_to_accel_ = gravity_ / hover_fraction;
            covariance_ = 100.0;
        }
    }

    // `throttle` is the normalised command in [0, 1], `accel_z` the measured
    // vertical acceleration in the world frame (m/s^2, gravity included).
    void update(double throttle, double accel_z) {
        if (throttle < 0.02 || throttle > 1.0)
            return;
        const double gamma = 1.0 / (kForgetting + throttle * covariance_ * throttle);
        const double gain = gamma * covariance_ * throttle;
        thrust_to_accel_ += gain * (accel_z - throttle * thrust_to_accel_);
        covariance_ = (1.0 - gain * throttle) * covariance_ / kForgetting;

        // Keep the hover throttle inside a sane band; a wild estimate is worse
        // than a stale one.
        const double hover = gravity_ / thrust_to_accel_;
        if (hover > 0.8)
            thrust_to_accel_ = gravity_ / 0.8;
        else if (hover < 0.1)
            thrust_to_accel_ = gravity_ / 0.1;
    }

private:
    static constexpr double kForgetting = 0.998;

    double gravity_ = 9.80665;
    double thrust_to_accel_ = 9.80665 / 0.705;
    double covariance_ = 100.0;
};

} // namespace flyrat
