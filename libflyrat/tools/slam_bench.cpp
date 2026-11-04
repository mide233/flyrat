// SPDX-License-Identifier: GPL-3.0-or-later
//
// Headless stereo SLAM benchmark.
//
//   slam_bench --vocab ORBvoc.txt --settings uvc_640x480.yaml
//       --left dir/left --right dir/right --fps 30
//
//   slam_bench --vocab ... --settings ... --video recording.avi
//
//   slam_bench --vocab ... --settings ... --sbs /dev/video0
//       --width 1280 --height 480 --fps 60 --seconds 20
//
// The settings file must describe the per-eye camera (for the side-by-side
// modes the frames are split in half before tracking).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <sys/resource.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "flyrat/slam/stereo_slam.hpp"

namespace {

struct Options {
    std::string vocab;
    std::string settings;
    std::string left_dir;
    std::string right_dir;
    std::string video;
    std::string sbs;
    std::string save_trajectory;
    flyrat::slam::Mode mode = flyrat::slam::Mode::kStereo;
    double fps = 30.0;
    int width = 1280;
    int height = 480;
    int max_frames = 0;
    double seconds = 0.0;
    bool realtime = false;
};

void usage()
{
    std::printf(
        "usage: slam_bench --vocab ORBvoc.txt --settings camera.yaml [input] [options]\n"
        "\n"
        "input (choose one):\n"
        "  --left DIR --right DIR   image sequence, sorted by filename\n"
        "  --video FILE             side-by-side video (left | right)\n"
        "  --sbs DEVICE             live side-by-side UVC device\n"
        "\n"
        "options:\n"
        "  --fps N                  timestamps per second (default 30; live capture\n"
        "                           uses the wall clock instead)\n"
        "  --width N --height N     combined capture size (default 1280x480)\n"
        "  --max-frames N           stop after N frames\n"
        "  --seconds N              stop after N seconds (live capture)\n"
        "  --mode stereo|imu-stereo select the ORB-SLAM3 sensor mode\n"
        "  --save-traj FILE         write a TUM trajectory after tracking\n"
        "  --help                   show this message\n");
}

std::vector<std::string> list_images(const std::string& dir)
{
    std::string cmd = "find '" + dir + "' -maxdepth 1 -type f "
                      "\\( -name '*.png' -o -name '*.jpg' -o -name '*.jpeg' -o -name '*.pgm' \\) "
                      "| sort";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe == nullptr) {
        return {};
    }
    std::vector<std::string> files;
    char line[1024];
    while (std::fgets(line, sizeof(line), pipe) != nullptr) {
        std::string path(line);
        while (!path.empty() && (path.back() == '\n' || path.back() == '\r')) {
            path.pop_back();
        }
        if (!path.empty()) {
            files.push_back(path);
        }
    }
    pclose(pipe);
    return files;
}

const char* state_name(flyrat::slam::State state)
{
    switch (state) {
    case flyrat::slam::State::kTracking:
        return "TRACKING";
    case flyrat::slam::State::kRecentlyLost:
        return "RECENTLY_LOST";
    case flyrat::slam::State::kLost:
        return "LOST";
    case flyrat::slam::State::kNotInitialized:
    default:
        return "NOT_INITIALIZED";
    }
}

struct Stats {
    std::size_t frames = 0;
    std::size_t tracked = 0;
    double total_ms = 0.0;
    double max_ms = 0.0;

    void add(bool ok, double ms)
    {
        ++frames;
        if (ok) {
            ++tracked;
        }
        total_ms += ms;
        max_ms = std::max(max_ms, ms);
    }
};

bool split(const cv::Mat& frame, cv::Mat& left, cv::Mat& right);
bool step_limit(const Options& options, std::size_t index);

void report(const Stats& stats)
{
    if (stats.frames == 0) {
        std::printf("no frames processed\n");
        return;
    }
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    std::printf("tracked=%zu/%zu avg=%.2f ms (%.1f fps) max=%.2f ms peak-rss=%.1f MB\n",
        stats.tracked, stats.frames, stats.total_ms / stats.frames,
        1000.0 * stats.frames / stats.total_ms, stats.max_ms,
        static_cast<double>(usage.ru_maxrss) / 1024.0);
}

bool run_sequence(const Options& options, flyrat::slam::StereoSlam& slam)
{
    const auto left = list_images(options.left_dir);
    const auto right = list_images(options.right_dir);
    if (left.empty() || right.empty()) {
        std::fprintf(stderr, "no images in %s or %s\n",
            options.left_dir.c_str(), options.right_dir.c_str());
        return false;
    }
    std::size_t count = std::min(left.size(), right.size());
    if (options.max_frames > 0) {
        count = std::min(count, static_cast<std::size_t>(options.max_frames));
    }
    std::printf("sequence: %zu frame pairs (fps %.1f)\n", count, options.fps);

    Stats stats;
    for (std::size_t i = 0; i < count; ++i) {
        const cv::Mat l = cv::imread(left[i], cv::IMREAD_GRAYSCALE);
        const cv::Mat r = cv::imread(right[i], cv::IMREAD_GRAYSCALE);
        if (l.empty() || r.empty()) {
            std::fprintf(stderr, "failed to read frame %zu\n", i);
            return false;
        }
        const double timestamp = static_cast<double>(i) / options.fps;
        const auto begin = std::chrono::steady_clock::now();
        const auto result = slam.track(l, r, timestamp);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        stats.add(result.pose.valid, ms);
        if (i % 10 == 0 || i + 1 == count) {
            std::printf("frame %zu/%zu %-15s t=(%.3f %.3f %.3f) %.2f ms\n",
                i + 1, count, state_name(result.state), result.pose.position.x(),
                result.pose.position.y(), result.pose.position.z(), ms);
        }
    }
    report(stats);
    return true;
}

bool run_video(const Options& options, flyrat::slam::StereoSlam& slam)
{
    cv::VideoCapture capture(options.video);
    if (!capture.isOpened()) {
        std::fprintf(stderr, "cannot open video %s\n", options.video.c_str());
        return false;
    }
    const double fps = options.fps > 0.0 ? options.fps : capture.get(cv::CAP_PROP_FPS);
    std::printf("video: %s (fps %.1f)\n", options.video.c_str(), fps);

    Stats stats;
    cv::Mat frame;
    std::size_t index = 0;
    while (capture.read(frame) && !frame.empty()) {
        if (step_limit(options, index)) {
            break;
        }
        cv::Mat left;
        cv::Mat right;
        if (!split(frame, left, right)) {
            return false;
        }
        const double timestamp = static_cast<double>(index) / (fps > 0.0 ? fps : 30.0);
        const auto begin = std::chrono::steady_clock::now();
        const auto result = slam.track(left, right, timestamp);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        stats.add(result.pose.valid, ms);
        if (index % 10 == 0) {
            std::printf("frame %zu %-15s t=(%.3f %.3f %.3f) %.2f ms\n", index,
                state_name(result.state), result.pose.position.x(),
                result.pose.position.y(), result.pose.position.z(), ms);
        }
        ++index;
    }
    report(stats);
    return index > 0;
}

bool split(const cv::Mat& frame, cv::Mat& left, cv::Mat& right)
{
    if (frame.cols < 2 || frame.cols % 2 != 0) {
        std::fprintf(stderr, "side-by-side frame must have an even width, got %d\n",
            frame.cols);
        return false;
    }
    const int half = frame.cols / 2;
    left = frame(cv::Rect(0, 0, half, frame.rows));
    right = frame(cv::Rect(half, 0, half, frame.rows));
    return true;
}

bool step_limit(const Options& options, std::size_t index)
{
    return options.max_frames > 0
        && index >= static_cast<std::size_t>(options.max_frames);
}

bool run_live(const Options& options, flyrat::slam::StereoSlam& slam)
{
    cv::VideoCapture capture(options.sbs, cv::CAP_V4L2);
    if (!capture.isOpened()) {
        std::fprintf(stderr, "cannot open camera %s\n", options.sbs.c_str());
        return false;
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    capture.set(cv::CAP_PROP_FPS, options.fps);
    std::printf("camera: %s %dx%d @ %.1f\n", options.sbs.c_str(),
        static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT)),
        capture.get(cv::CAP_PROP_FPS));

    Stats stats;
    cv::Mat frame;
    const auto start = std::chrono::steady_clock::now();
    std::size_t index = 0;
    while (capture.read(frame) && !frame.empty()) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if ((options.seconds > 0.0 && elapsed > options.seconds)
            || (options.max_frames > 0
                && index >= static_cast<std::size_t>(options.max_frames))) {
            break;
        }
        cv::Mat left;
        cv::Mat right;
        if (!split(frame, left, right)) {
            return false;
        }
        const auto begin = std::chrono::steady_clock::now();
        const auto result = slam.track(left, right, elapsed);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        stats.add(result.pose.valid, ms);
        if (index % 10 == 0) {
            std::printf("frame %zu %-15s t=(%.3f %.3f %.3f) %.2f ms\n", index,
                state_name(result.state), result.pose.position.x(),
                result.pose.position.y(), result.pose.position.z(), ms);
        }
        ++index;
    }
    report(stats);
    return index > 0;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--vocab") {
            options.vocab = next("--vocab");
        } else if (arg == "--settings") {
            options.settings = next("--settings");
        } else if (arg == "--left") {
            options.left_dir = next("--left");
        } else if (arg == "--right") {
            options.right_dir = next("--right");
        } else if (arg == "--video") {
            options.video = next("--video");
        } else if (arg == "--sbs") {
            options.sbs = next("--sbs");
        } else if (arg == "--save-traj") {
            options.save_trajectory = next("--save-traj");
        } else if (arg == "--fps") {
            options.fps = std::atof(next("--fps").c_str());
        } else if (arg == "--width") {
            options.width = std::atoi(next("--width").c_str());
        } else if (arg == "--height") {
            options.height = std::atoi(next("--height").c_str());
        } else if (arg == "--max-frames") {
            options.max_frames = std::atoi(next("--max-frames").c_str());
        } else if (arg == "--seconds") {
            options.seconds = std::atof(next("--seconds").c_str());
        } else if (arg == "--mode") {
            const std::string mode = next("--mode");
            if (mode == "stereo") {
                options.mode = flyrat::slam::Mode::kStereo;
            } else if (mode == "imu-stereo") {
                options.mode = flyrat::slam::Mode::kImuStereo;
            } else {
                std::fprintf(stderr, "unknown --mode %s\n", mode.c_str());
                return 2;
            }
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (options.vocab.empty() || options.settings.empty()) {
        usage();
        return 2;
    }
    const int inputs = !options.left_dir.empty() + !options.video.empty() + !options.sbs.empty();
    if (inputs != 1 || (!options.left_dir.empty() && options.right_dir.empty())) {
        std::fprintf(stderr, "choose exactly one input\n");
        usage();
        return 2;
    }

    flyrat::slam::Options slam_options;
    slam_options.mode = options.mode;
    slam_options.vocabulary_path = options.vocab;
    slam_options.settings_path = options.settings;

    try {
        flyrat::slam::StereoSlam slam(slam_options);
        bool ok = false;
        if (!options.left_dir.empty()) {
            ok = run_sequence(options, slam);
        } else if (!options.video.empty()) {
            ok = run_video(options, slam);
        } else {
            ok = run_live(options, slam);
        }
        slam.shutdown();
        if (!options.save_trajectory.empty()) {
            slam.saveTrajectory(options.save_trajectory);
            std::printf("trajectory: %s\n", options.save_trajectory.c_str());
        }
        return ok ? 0 : 1;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "slam_bench: %s\n", error.what());
        return 1;
    }
}
