// SPDX-License-Identifier: MIT
//
// Guidance setpoint handed to the autopilot each cycle.
#pragma once

namespace flyrat {

struct Setpoint {
  // Desired attitude, radians (roll/pitch) and yaw rate (rad/s).
  double roll = 0.0;
  double pitch = 0.0;
  double yaw_rate = 0.0;

  // Desired collective throttle in [0, 1]. Altitude controllers fill this in.
  double throttle = 0.0;
};

}  // namespace flyrat
