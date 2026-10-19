// SPDX-License-Identifier: MIT
#include "flyrat/attitude_hold.hpp"

#include <algorithm>
#include <cmath>

namespace flyrat {
namespace {

double clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }

} // namespace

Command AttitudeHold::update(const State& state, const Setpoint& setpoint, double dt) {
    (void)dt; // P-only for now; integrators land here later.

    const double roll_error = setpoint.roll - state.roll;
    const double pitch_error = setpoint.pitch - state.pitch;
    const double yaw_rate_error = setpoint.yaw_rate - state.yaw_rate;

    Command command;
    command.roll = clamp(gains_.roll_kp * roll_error, -gains_.max_roll, gains_.max_roll);
    command.pitch = clamp(gains_.pitch_kp * pitch_error, -gains_.max_pitch, gains_.max_pitch);
    command.yaw = clamp(gains_.yaw_rate_kp * yaw_rate_error, -gains_.max_yaw, gains_.max_yaw);
    command.throttle = clamp(setpoint.throttle, 0.0, 1.0);
    command.valid = true;
    return command;
}

} // namespace flyrat
