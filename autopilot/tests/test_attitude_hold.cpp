// SPDX-License-Identifier: MIT
//
// Tiny dependency-free test runner for the autopilot core. Keeping this free of
// gtest/ROS is what lets the library build anywhere.
#include <cmath>
#include <cstdio>
#include <string>

#include "flyrat/attitude_hold.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what) {
  if (ok) {
    std::printf("  ok   %s\n", what.c_str());
  } else {
    std::printf("  FAIL %s\n", what.c_str());
    ++g_failures;
  }
}

bool near(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps;
}

}  // namespace

int main() {
  using flyrat::AttitudeHold;
  using flyrat::Command;
  using flyrat::Setpoint;
  using flyrat::State;

  std::printf("AttitudeHold\n");

  // Level vehicle, level setpoint: no attitude correction, throttle passes
  // straight through.
  {
    AttitudeHold ap;
    State state;
    Setpoint sp;
    sp.throttle = 0.6;
    const Command cmd = ap.update(state, sp, 0.01);
    check(near(cmd.roll, 0.0), "level roll -> no roll command");
    check(near(cmd.pitch, 0.0), "level pitch -> no pitch command");
    check(near(cmd.yaw, 0.0), "no yaw rate error -> no yaw command");
    check(near(cmd.throttle, 0.6), "throttle passes through");
    check(cmd.valid, "command is valid");
  }

  // Positive roll error drives positive roll command (sign convention).
  {
    AttitudeHold ap;
    State state;
    state.roll = -0.2;  // rolled left, want level
    Setpoint sp;
    const Command cmd = ap.update(state, sp, 0.01);
    check(cmd.roll > 0.0, "rolled left -> right roll command");
    check(near(cmd.roll, 0.2), "roll command equals gain * error");
  }

  // Command saturates at the configured deflection limit.
  {
    AttitudeHold::Gains gains;
    gains.roll_kp = 100.0;
    gains.max_roll = 1.0;
    AttitudeHold ap(gains);
    State state;
    state.roll = -1.0;
    Setpoint sp;
    const Command cmd = ap.update(state, sp, 0.01);
    check(near(cmd.roll, 1.0), "roll command saturates at max_roll");
  }

  // Yaw rate tracking.
  {
    AttitudeHold ap;
    State state;
    state.yaw_rate = 0.0;
    Setpoint sp;
    sp.yaw_rate = 1.0;
    const Command cmd = ap.update(state, sp, 0.01);
    check(near(cmd.yaw, 0.5), "yaw command = yaw_rate_kp * rate error");
  }

  // Throttle is clamped to [0, 1].
  {
    AttitudeHold ap;
    State state;
    Setpoint sp;
    sp.throttle = 5.0;
    check(near(ap.update(state, sp, 0.01).throttle, 1.0),
          "throttle clamped to 1");
    sp.throttle = -5.0;
    check(near(ap.update(state, sp, 0.01).throttle, 0.0),
          "throttle clamped to 0");
  }

  if (g_failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAILED (%d)\n", g_failures);
  return 1;
}
