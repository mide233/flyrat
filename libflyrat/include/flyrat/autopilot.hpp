// SPDX-License-Identifier: MIT
//
// Autopilot interface. A controller receives the latest estimated state and a
// guidance setpoint and returns a normalized command. Implementations must be
// deterministic and free of ROS/transport dependencies so they can be unit
// tested and, later, reused on the real vehicle.
#pragma once

#include "flyrat/command.hpp"
#include "flyrat/setpoint.hpp"
#include "flyrat/state.hpp"

namespace flyrat {

class Autopilot {
public:
    virtual ~Autopilot() = default;

    // Clear integrators/state; called before (re)arming.
    virtual void reset() = 0;

    // Produce one command. `dt` is the cycle time in seconds.
    virtual Command update(const State& state, const Setpoint& setpoint, double dt) = 0;
};

} // namespace flyrat
