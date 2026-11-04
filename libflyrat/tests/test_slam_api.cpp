// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <string>

#include "flyrat/slam/stereo_slam.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

} // namespace

int main()
{
    using namespace flyrat::slam;

    check(static_cast<int>(State::kTracking) != static_cast<int>(State::kLost),
        "state values are distinct");

    Options options;
    check(options.mode == Mode::kStereo, "stereo mode by default");

    bool threw = false;
    try {
        StereoSlam slam(options);
    } catch (const SlamError&) {
        threw = true;
    }
    check(threw, "empty paths are rejected");

    options.vocabulary_path = "/definitely/not/a/vocabulary";
    options.settings_path = "/definitely/not/a/settings";
    threw = false;
    try {
        StereoSlam slam(options);
    } catch (const SlamError&) {
        threw = true;
    }
    check(threw, "missing vocabulary is rejected");

    options.vocabulary_path = "/dev/null";
    options.settings_path = "/definitely/not/a/settings";
    threw = false;
    try {
        StereoSlam slam(options);
    } catch (const SlamError&) {
        threw = true;
    }
    check(threw, "missing settings is rejected");

#ifdef FLYRAT_TEST_ORB_VOCAB
    // Optional smoke test: with a real vocabulary, construct and tear down the
    // system. This exercises the ORB-SLAM3 settings parser and vocabulary load.
    options.vocabulary_path = FLYRAT_TEST_ORB_VOCAB;
    options.settings_path = FLYRAT_TEST_ORB_SETTINGS;
    try {
        StereoSlam slam(options);
        slam.shutdown();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: system construction: %s\n", error.what());
        ++failures;
    }
#endif

    if (failures == 0) {
        std::printf("SLAM API tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
