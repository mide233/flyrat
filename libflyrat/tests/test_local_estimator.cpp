// SPDX-License-Identifier: MIT
//
// Local tangent-plane conversion for the GPS-fed position estimate. The test
// pins the sign conventions the autopilot depends on: x = east, y = north.
#include <cmath>
#include <cstdio>
#include <string>

#include "flyrat/local_estimator.hpp"

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

bool near(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }

flyrat::GpsFrame fix(double lat, double lon, double alt, double speed, double course) {
    flyrat::GpsFrame g;
    g.fix = true;
    g.satellites = 12;
    g.lat_deg = lat;
    g.lon_deg = lon;
    g.altitude_m = alt;
    g.ground_speed_mps = speed;
    g.ground_course_deg = course;
    return g;
}

} // namespace

int main() {
    using flyrat::LocalEstimator;

    std::printf("LocalEstimator\n");

    // The first fix defines the origin.
    {
        LocalEstimator est;
        check(est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 250.0, 0.0), "first fix valid");
        check(near(est.position().east_m, 0.0), "origin east");
        check(near(est.position().north_m, 0.0), "origin north");
        check(near(est.position().up_m, 0.0), "origin up (baro reference)");
    }

    // A pure latitude increase is due north: +0.0001 deg ~ 11.13 m.
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0001, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        check(near(est.position().north_m, 11.132), "north is +y");
        check(near(est.position().east_m, 0.0, 1e-6), "north does not leak into east");
    }

    // A pure longitude increase at 40 deg N is east, scaled by cos(lat).
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0, -79.4999, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        check(near(est.position().east_m, 8.527), "east is +x");
        check(near(est.position().north_m, 0.0, 1e-6), "east does not leak into north");
    }

    // Barometric altitude is the vertical channel; the GPS altitude field only
    // has one metre resolution.
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0, -79.5, 101.0, 0.0, 0.0), true, 101.5, 0.75);
        check(near(est.position().up_m, 1.5), "up uses baro");
        check(near(est.position().vel_up_mps, 0.75), "climb rate uses baro vario");
    }

    // Course 90 deg (east) with 2 m/s ground speed.
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0, -79.5, 100.0, 2.0, 90.0), true, 100.0, 0.0);
        check(near(est.position().vel_east_mps, 2.0, 1e-6), "course 90 -> +east");
        check(near(est.position().vel_north_mps, 0.0, 1e-6), "course 90 -> no north");
    }

    // Course 0 deg (north).
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0, -79.5, 100.0, 3.0, 0.0), true, 100.0, 0.0);
        check(near(est.position().vel_north_mps, 3.0, 1e-6), "course 0 -> +north");
        check(near(est.position().vel_east_mps, 0.0, 1e-6), "course 0 -> no east");
    }

    // Hovering: the GPS course is meaningless, so velocity must be zeroed.
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0, -79.5, 100.0, 0.1, 217.0), true, 100.0, 0.0);
        check(near(est.position().vel_east_mps, 0.0, 1e-9), "deadband kills east jitter");
        check(near(est.position().vel_north_mps, 0.0, 1e-9), "deadband kills north jitter");
    }

    // Losing the fix must not silently keep publishing a stale position.
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        flyrat::GpsFrame lost;
        lost.fix = false;
        check(!est.update(lost, true, 100.0, 0.0), "no fix -> invalid");
        check(!est.position().valid, "no fix -> valid flag cleared");
        check(est.origin_set(), "origin survives a dropout");
    }

    // reset() re-anchors on the next fix.
    {
        LocalEstimator est;
        est.update(fix(40.0, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.update(fix(40.0001, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        est.reset();
        check(!est.origin_set(), "reset clears the origin");
        est.update(fix(40.0001, -79.5, 100.0, 0.0, 0.0), true, 100.0, 0.0);
        check(near(est.position().north_m, 0.0), "reset re-anchors");
    }

    if (g_failures == 0) {
        std::printf("all checks passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
