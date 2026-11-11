// SPDX-License-Identifier: MIT
//
// Compile-time/runtime smoke test for the hardware public headers. It does not
// touch I2C or GPIO.
#include <cstdio>
#include <string>

#include "flyrat/hardware/imu.hpp"
#include "flyrat/hardware/stereo_camera.hpp"
#include "flyrat/hardware/tof.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

} // namespace

int main()
{
    flyrat::hardware::TofZone zone;
    zone.target_status = 5;
    check(zone.valid(), "status 5 is valid");
    zone.target_status = 9;
    check(zone.valid(), "status 9 is valid");
    zone.target_status = 0;
    check(!zone.valid(), "status 0 is invalid");

    flyrat::hardware::TofFrame frame;
    frame.sensor = 2;
    frame.resolution = 4;
    frame.zones[0].distance_mm = 1234;
    check(frame.sensor == 2 && frame.resolution == 4 && frame.zones[0].distance_mm == 1234,
        "frame fields");

    flyrat::hardware::TofConfig config;
    check(config.sensor_count == 4, "four sensors by default");
    check(flyrat::hardware::kTofAddresses[0] == 0x60 &&
              flyrat::hardware::kTofAddresses[3] == 0x66,
        "hardcoded ToF addresses");
    check(flyrat::hardware::kTofDefaultAddress == 0x52, "ToF default address");

    flyrat::hardware::StereoConfig stereo;
    check(stereo.width == 1280 && stereo.height == 480, "stereo defaults");
    check(stereo.width / 2 == 640, "stereo eye width");
    flyrat::hardware::StereoFrame stereo_frame;
    stereo_frame.width = 640;
    stereo_frame.height = 480;
    stereo_frame.format = flyrat::hardware::StereoPixelFormat::kGray8;
    stereo_frame.left.resize(640 * 480);
    stereo_frame.right.resize(640 * 480);
    check(stereo_frame.left.size() == stereo_frame.right.size(), "stereo eye buffers");
    check(stereo_frame.format == flyrat::hardware::StereoPixelFormat::kGray8,
        "stereo GRAY8 frame format");

    if (g_failures != 0) {
        std::printf("%d hardware header check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("hardware header tests passed\n");
    return 0;
}
