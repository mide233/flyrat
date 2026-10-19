// SPDX-License-Identifier: MIT
//
// Normalized command produced by the autopilot.
#pragma once

namespace flyrat {

struct Command {
    // Stick deflections in [-1, 1]; 0 is centred.
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // Throttle in [0, 1]; 0 is idle, 1 is full.
    double throttle = 0.0;

    bool valid = true;
};

} // namespace flyrat
