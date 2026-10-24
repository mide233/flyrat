// SPDX-License-Identifier: MIT
#include "flyrat/local_estimator.hpp"

#include <cmath>

namespace flyrat {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
// Mean Earth radius (WGS84 semi-major axis). Good to ~0.5% for the small
// tangent-plane offsets we care about.
constexpr double kEarthRadiusM = 6378137.0;

} // namespace

void LocalEstimator::reset() {
    origin_set_ = false;
    position_ = LocalPosition{};
}

bool LocalEstimator::update(
    const GpsFrame& gps, bool baro_valid, double baro_altitude_m, double baro_vario_mps) {
    if (!gps.fix) {
        position_.valid = false;
        return false;
    }

    if (!origin_set_) {
        lat0_deg_ = gps.lat_deg;
        lon0_deg_ = gps.lon_deg;
        use_baro_ = baro_valid;
        alt0_m_ = baro_valid ? baro_altitude_m : gps.altitude_m;
        origin_set_ = true;
    }

    const double lat0_rad = lat0_deg_ * kDegToRad;
    position_.north_m = (gps.lat_deg - lat0_deg_) * kDegToRad * kEarthRadiusM;
    position_.east_m =
        (gps.lon_deg - lon0_deg_) * kDegToRad * kEarthRadiusM * std::cos(lat0_rad);

    // Vertical: the barometer is centimetre resolution and smooth, the GPS
    // altitude field is quantised to one metre (MSP backwards compatibility).
    if (use_baro_ && baro_valid) {
        position_.up_m = baro_altitude_m - alt0_m_;
        position_.vel_up_mps = baro_vario_mps;
    } else {
        position_.up_m = gps.altitude_m - alt0_m_;
        position_.vel_up_mps = 0.0;
    }

    // Ground speed and course are the GPS Doppler solution. The course is only
    // meaningful while moving: at a hover it jumps around the whole compass,
    // which would inject a large fake velocity, so gate it.
    if (gps.ground_speed_mps > kSpeedDeadbandMps) {
        const double course = gps.ground_course_deg * kDegToRad;
        position_.vel_east_mps = gps.ground_speed_mps * std::sin(course);
        position_.vel_north_mps = gps.ground_speed_mps * std::cos(course);
    } else {
        position_.vel_east_mps = 0.0;
        position_.vel_north_mps = 0.0;
    }

    position_.valid = true;
    return true;
}

} // namespace flyrat
