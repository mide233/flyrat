// SPDX-License-Identifier: MIT
//
// Estimated vehicle state fed to the autopilot.
//
// Everything the autopilot is allowed to see comes from the flight
// controller's own estimator (Betaflight MSP telemetry). The autopilot is a
// plain C++ library: it must not depend on ROS, Gazebo or any transport.
#pragma once

namespace flyrat {

struct State {
    // Attitude, radians. Roll/pitch are body-to-level errors, yaw is a free
    // heading in [-pi, pi] (NED/ENU convention is the caller's job).
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // Body angular rates, rad/s.
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;

    // Barometric/estimated altitude above the takeoff point, metres.
    double altitude = 0.0;

    // Climb rate, metres per second.
    double climb_rate = 0.0;

    // Supply voltage, volts.
    double battery_voltage = 0.0;

    // True once the flight controller reports the ARM flag.
    bool armed = false;

    // Monotonic age of this sample, seconds. Lets the autopilot fail safe when
    // telemetry stalls.
    double age = 0.0;
};

} // namespace flyrat
