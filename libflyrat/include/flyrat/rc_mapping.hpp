// SPDX-License-Identifier: MIT
//
// Stick <-> physical command conversion for the flight controller.
//
// The SITL (and the real machine) is configured with
//
//     rates_type = BETAFLIGHT, rc_expo = 0, super_rate = 0, rc_rate = 200
//
// which makes Betaflight's rate curve exactly linear:
//
//     rate [deg/s] = 200 * (rc_rate / 100) * stick
//
// so 400 deg/s at full stick. That is what lets a controller emit body rates
// in rad/s and convert them to receiver pulses without fitting any curve.
#pragma once

#include <algorithm>
#include <cmath>

namespace flyrat {

struct RcMapping {
    // Full stick rate, matching the rc_rate programmed into the FC.
    double full_stick_rate_dps = 400.0;
    double min_pulse = 1000.0;
    double mid_pulse = 1500.0;
    double max_pulse = 2000.0;

    // Body rate in rad/s -> normalised stick in [-1, 1].
    double stick_from_rate(double rate_rad_s) const {
        constexpr double kRadToDeg = 57.29577951308232;
        const double dps = rate_rad_s * kRadToDeg;
        return std::clamp(dps / full_stick_rate_dps, -1.0, 1.0);
    }

    double pulse_from_stick(double stick) const {
        const double half = 0.5 * (max_pulse - min_pulse);
        return std::clamp(mid_pulse + half * stick, min_pulse, max_pulse);
    }

    // Normalised thrust in [0, 1] -> throttle pulse.
    double pulse_from_throttle(double throttle) const {
        return min_pulse + (max_pulse - min_pulse) * std::clamp(throttle, 0.0, 1.0);
    }
};

} // namespace flyrat
