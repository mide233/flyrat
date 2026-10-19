/// @file main.cpp
/// @brief Placeholder entry point for the real-machine program.
///
/// The runtime loop is intentionally not implemented yet. When it is written,
/// it should own:
///   - the MSP link (`flyrat::MspClient`, TCP for SITL or serial on the vehicle),
///   - the RC channel stream (`flyrat::RcReceiver` / the vehicle's own receiver),
///   - a 50 Hz command sender plus an independent telemetry/fail-safe loop,
///   - the autopilot implementation from `flyrat::Autopilot`.
///
/// The CMake target already links `flyrat::flyrat`, so no build wiring is
/// needed to start using the library.

#include <cstdio>

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    std::puts("[flyrat_pilot] placeholder: runtime is not implemented yet");
    return 0;
}
