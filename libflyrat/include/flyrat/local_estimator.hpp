// SPDX-License-Identifier: MIT
//
// Local position estimate built from the flight controller's GPS solution.
//
// The autopilot works in a local tangent plane anchored at the first valid
// fix: x = east, y = north, z = up, all in metres, with the origin at
// (lat0, lon0) and the vertical zero at the barometric altitude captured at
// the same moment. That is the same contract a VSLAM front end can satisfy on
// the real machine (origin at startup), so the control code does not care
// which one is feeding it.
//
// The conversion is the flat-earth approximation, which is accurate to a few
// centimetres over the few hundred metres a multirotor can realistically fly
// in one battery.
#pragma once

#include "flyrat/msp_client.hpp"

namespace flyrat {

struct LocalPosition {
    double east_m = 0.0;
    double north_m = 0.0;
    double up_m = 0.0;

    double vel_east_mps = 0.0;
    double vel_north_mps = 0.0;
    double vel_up_mps = 0.0;

    bool valid = false;
};

class LocalEstimator {
public:
    // Feed one GPS sample plus the barometric altitude of the same instant.
    // Returns true when a local position is available. Vertical velocity comes
    // from the barometric vario (GPS altitude has one metre resolution).
    bool update(const GpsFrame& gps, bool baro_valid, double baro_altitude_m,
        double baro_vario_mps);

    // Forget the origin; the next valid fix becomes the new one.
    void reset();

    const LocalPosition& position() const { return position_; }
    bool origin_set() const { return origin_set_; }

    // Below this ground speed the GPS course is noise, so velocity is zeroed.
    static constexpr double kSpeedDeadbandMps = 0.25;

private:
    bool origin_set_ = false;
    double lat0_deg_ = 0.0;
    double lon0_deg_ = 0.0;
    double alt0_m_ = 0.0;
    bool use_baro_ = false;

    LocalPosition position_;
};

} // namespace flyrat
