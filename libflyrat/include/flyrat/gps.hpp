// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

namespace flyrat {

// GPS fix decoded from MSP_RAW_GPS. Kept in the core library because the local
// estimator consumes it without needing the MSP client.
struct GpsFrame {
    std::uint8_t fix = 0;
    std::uint8_t satellites = 0;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double altitude_m = 0.0;
    double ground_speed_mps = 0.0;
    double ground_course_deg = 0.0;
    double pdop = 0.0;
    double hdop = 0.0;
    double vdop = 0.0;
};

} // namespace flyrat
