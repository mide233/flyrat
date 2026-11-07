// SPDX-License-Identifier: GPL-3.0-or-later
//
// Headless VINS-Fusion stereo-inertial benchmark.
//
//   slam_bench --config libflyrat/configs/vins_fusion/uvc_640x480_imu.yaml
//       --imu imu.csv --left dir/left --right dir/right --fps 30
//
//   slam_bench --config ... --imu imu.csv --video recording.avi
//
// The IMU CSV is headerless: timestamp,ax,ay,az,gx,gy,gz (seconds, m/s^2,
// rad/s), one sample per line. Frame timestamps start at the first IMU sample
// and advance by 1/fps.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/resource.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "flyrat/slam/stereo_slam.hpp"

namespace {

struct Options {
    std::string config;
    std::string imu_path;
    std::string left_dir;
    std::string right_dir;
    std::string video;
    std::string save_trajectory;
    double fps = 30.0;
    int max_frames = 0;
    double seconds = 0.0;
    bool realtime = false;
};

void usage()
{
    std::printf(
        "usage: slam_bench --config VINS.yaml --imu imu.csv [input] [options]\n"
        "\n"
        "input (choose one):\n"
        "  --left DIR --right DIR   image sequence, sorted by filename\n"
        "  --video FILE             side-by-side video (left | right)\n"
        "\n"
        "options:\n"
        "  --fps N                  frame timestamps per second (default 30)\n"
        "  --max-frames N           stop after N frames\n"
        "  --seconds N              stop after N seconds of replay\n"
        "  --realtime               sleep to match the requested fps\n"
        "  --save-traj FILE         write a TUM trajectory after tracking\n"
        "  --help                   show this message\n"
        "\n"
        "IMU CSV columns: timestamp,ax,ay,az,gx,gy,gz\n");
}

std::vector<std::string> list_images(const std::string& dir)
{
    std::vector<std::string> files;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator(dir, error)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string extension = entry.path().extension().string();
        if (extension == ".png" || extension == ".jpg" || extension == ".jpeg"
            || extension == ".pgm") {
            files.push_back(entry.path().string());
        }
    }
    if (error) {
        return {};
    }
    std::sort(files.begin(), files.end());
    return files;
}

bool read_imu_csv(const std::string& path,
    std::vector<flyrat::slam::ImuSample>& samples)
{
    std::ifstream input(path);
    if (!input) {
        std::fprintf(stderr, "cannot open IMU CSV %s\n", path.c_str());
        return false;
    }

    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream stream(line);
        flyrat::slam::ImuSample sample;
        if (!(stream >> sample.timestamp
                >> sample.accel.x() >> sample.accel.y() >> sample.accel.z()
                >> sample.gyro.x() >> sample.gyro.y() >> sample.gyro.z())) {
            std::fprintf(stderr, "bad IMU CSV line %zu in %s\n", line_number,
                path.c_str());
            return false;
        }
        samples.push_back(sample);
    }
    if (samples.empty()) {
        std::fprintf(stderr, "IMU CSV %s is empty\n", path.c_str());
        return false;
    }
    std::sort(samples.begin(), samples.end(),
        [](const flyrat::slam::ImuSample& a, const flyrat::slam::ImuSample& b) {
            return a.timestamp < b.timestamp;
        });
    return true;
}

const char* state_name(flyrat::slam::State state)
{
    switch (state) {
    case flyrat::slam::State::kTracking:
        return "TRACKING";
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
    std::size_t lost = 0;
    std::size_t not_initialized = 0;
    std::size_t skipped = 0;
    double total_ms = 0.0;
    double max_ms = 0.0;

    void add(const flyrat::slam::Result& result, double ms)
    {
        ++frames;
        total_ms += ms;
        max_ms = std::max(max_ms, ms);
        switch (result.state) {
        case flyrat::slam::State::kTracking:
            ++tracked;
            break;
        case flyrat::slam::State::kLost:
            ++lost;
            break;
        case flyrat::slam::State::kNotInitialized:
        default:
            ++not_initialized;
            break;
        }
    }
};

void report(const Stats& stats)
{
    if (stats.frames == 0) {
        std::printf("no frames processed (skipped=%zu)\n", stats.skipped);
        return;
    }
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    std::printf(
        "frames=%zu tracking=%zu lost=%zu not-init=%zu skipped=%zu "
        "avg=%.2f ms (%.1f fps) max=%.2f ms peak-rss=%.1f MB\n",
        stats.frames, stats.tracked, stats.lost, stats.not_initialized,
        stats.skipped, stats.total_ms / stats.frames,
        1000.0 * stats.frames / stats.total_ms, stats.max_ms,
        static_cast<double>(usage.ru_maxrss) / 1024.0);
}

bool split(const cv::Mat& frame, cv::Mat& left, cv::Mat& right)
{
    if (frame.cols < 2 || frame.cols % 2 != 0) {
        std::fprintf(stderr,
            "side-by-side frame must have an even width, got %d\n", frame.cols);
        return false;
    }
    const int half = frame.cols / 2;
    left = frame(cv::Rect(0, 0, half, frame.rows));
    right = frame(cv::Rect(half, 0, half, frame.rows));
    return true;
}

bool feed_frame(flyrat::slam::StereoSlam& slam,
    std::vector<flyrat::slam::ImuSample>& imu_samples, std::size_t& imu_cursor,
    const cv::Mat& left, const cv::Mat& right, double timestamp, Stats& stats,
    std::size_t index)
{
    std::vector<flyrat::slam::ImuSample> batch;
    while (imu_cursor < imu_samples.size()
        && imu_samples[imu_cursor].timestamp <= timestamp) {
        batch.push_back(imu_samples[imu_cursor]);
        ++imu_cursor;
    }
    if (batch.empty()) {
        ++stats.skipped;
        return true;
    }

    const auto begin = std::chrono::steady_clock::now();
    const flyrat::slam::Result result = slam.track(left, right, timestamp, batch);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    stats.add(result, ms);
    if (index % 10 == 0 || result.state != flyrat::slam::State::kNotInitialized) {
        std::printf("frame %zu %-15s t=(%.3f %.3f %.3f) %.2f ms\n", index,
            state_name(result.state), result.pose.position.x(),
            result.pose.position.y(), result.pose.position.z(), ms);
    }
    return true;
}

bool run_sequence(const Options& options, flyrat::slam::StereoSlam& slam,
    std::vector<flyrat::slam::ImuSample>& imu_samples)
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
    if (options.seconds > 0.0) {
        count = std::min(count,
            static_cast<std::size_t>(options.seconds * options.fps));
    }
    std::printf("sequence: %zu frame pairs (fps %.1f)\n", count, options.fps);

    Stats stats;
    std::size_t cursor = 0;
    const double first_timestamp = imu_samples.front().timestamp;
    for (std::size_t i = 0; i < count; ++i) {
        const cv::Mat l = cv::imread(left[i], cv::IMREAD_GRAYSCALE);
        const cv::Mat r = cv::imread(right[i], cv::IMREAD_GRAYSCALE);
        if (l.empty() || r.empty()) {
            std::fprintf(stderr, "failed to read frame %zu\n", i);
            return false;
        }
        const double timestamp = first_timestamp + static_cast<double>(i) / options.fps;
        if (!feed_frame(slam, imu_samples, cursor, l, r, timestamp, stats, i + 1)) {
            return false;
        }
        if (options.realtime) {
            std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / options.fps));
        }
    }
    report(stats);
    return true;
}

bool run_video(const Options& options, flyrat::slam::StereoSlam& slam,
    std::vector<flyrat::slam::ImuSample>& imu_samples)
{
    cv::VideoCapture capture(options.video);
    if (!capture.isOpened()) {
        std::fprintf(stderr, "cannot open video %s\n", options.video.c_str());
        return false;
    }
    const double fps = options.fps > 0.0 ? options.fps : 30.0;
    std::printf("video: %s (fps %.1f)\n", options.video.c_str(), fps);

    Stats stats;
    std::size_t cursor = 0;
    std::size_t index = 0;
    const double first_timestamp = imu_samples.front().timestamp;
    cv::Mat frame;
    while (capture.read(frame) && !frame.empty()) {
        if (options.max_frames > 0
            && index >= static_cast<std::size_t>(options.max_frames)) {
            break;
        }
        if (options.seconds > 0.0
            && static_cast<double>(index) / fps >= options.seconds) {
            break;
        }
        cv::Mat left;
        cv::Mat right;
        if (!split(frame, left, right)) {
            return false;
        }
        const double timestamp = first_timestamp + static_cast<double>(index) / fps;
        if (!feed_frame(slam, imu_samples, cursor, left, right, timestamp,
                stats, index)) {
            return false;
        }
        ++index;
        if (options.realtime) {
            std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / fps));
        }
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
        if (arg == "--config") {
            options.config = next("--config");
        } else if (arg == "--imu") {
            options.imu_path = next("--imu");
        } else if (arg == "--left") {
            options.left_dir = next("--left");
        } else if (arg == "--right") {
            options.right_dir = next("--right");
        } else if (arg == "--video") {
            options.video = next("--video");
        } else if (arg == "--save-traj") {
            options.save_trajectory = next("--save-traj");
        } else if (arg == "--fps") {
            options.fps = std::atof(next("--fps").c_str());
        } else if (arg == "--max-frames") {
            options.max_frames = std::atoi(next("--max-frames").c_str());
        } else if (arg == "--seconds") {
            options.seconds = std::atof(next("--seconds").c_str());
        } else if (arg == "--realtime") {
            options.realtime = true;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (options.config.empty() || options.imu_path.empty()) {
        usage();
        return 2;
    }
    const int inputs = !options.left_dir.empty() + !options.video.empty();
    if (inputs != 1 || (!options.left_dir.empty() && options.right_dir.empty())) {
        std::fprintf(stderr, "choose exactly one input\n");
        usage();
        return 2;
    }
    if (options.fps <= 0.0) {
        std::fprintf(stderr, "--fps must be positive\n");
        return 2;
    }

    std::vector<flyrat::slam::ImuSample> imu_samples;
    if (!read_imu_csv(options.imu_path, imu_samples)) {
        return 1;
    }

    flyrat::slam::Options slam_options;
    slam_options.config_path = options.config;

    try {
        flyrat::slam::StereoSlam slam(slam_options);
        bool ok = false;
        if (!options.left_dir.empty()) {
            ok = run_sequence(options, slam, imu_samples);
        } else {
            ok = run_video(options, slam, imu_samples);
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
