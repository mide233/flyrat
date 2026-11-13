// SPDX-License-Identifier: MIT
//
// Raw sensor bag format used by flyrat_raw_recorder.
//
// This container is deliberately separate from the calibrated sensor bag in
// sensor_bag.hpp. It stores MSP int16 IMU counts, monotonic timestamps and the
// original MJPEG frames, so a new calibration can be estimated without undoing
// the calibration that is currently active in the live pipeline.
#pragma once

#include <array>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace flyrat::pilot {

struct RawBagMetadata {
    std::string tool = "flyrat_raw_recorder";
    int format_version = 1;
    std::string git_commit = "unknown";
    std::int64_t created_unix_ns = 0;

    bool imu_enabled = true;
    std::string msp_device;
    int msp_baud = 921600;
    int imu_rate_hz = 200;
    double acc_lsb_per_g = 2048.0;
    double gyro_lsb_per_dps = 16.4;
    std::string accel_axis = "none";
    std::string gyro_axis = "none";

    bool camera_enabled = true;
    std::string camera_device;
    int width = 0;
    int height = 0;
    std::string camera_format = "MJPEG";
    std::string camera_layout = "side_by_side";
    int capture_fps_requested = 0;
    double capture_fps_negotiated = 0.0;
    int delivery_fps = 0;

    bool preview_enabled = false;
    std::string preview_address = "127.0.0.1";
    int preview_port = 5000;

    std::int64_t start_steady_ns = 0;
    std::int64_t start_system_ns = 0;
};

std::string raw_bag_metadata_json(const RawBagMetadata& metadata);

struct RawBagStats {
    std::uint64_t imu_records = 0;
    std::uint64_t image_records = 0;
    std::uint64_t event_records = 0;
    std::uint64_t dropped_imu = 0;
    std::uint64_t dropped_images = 0;
    std::uint64_t write_errors = 0;
    std::uint64_t bytes_written = 0;
    std::size_t queued_imu = 0;
    std::size_t queued_images = 0;
    std::size_t queued_bytes = 0;
};

// Asynchronous writer. The capture callbacks must never block on disk I/O, so
// records are queued and a single worker thread owns the file.
class RawBagWriter {
public:
    static constexpr std::uint8_t kImu = 1;
    static constexpr std::uint8_t kImage = 2;
    static constexpr std::uint8_t kEvent = 3;
    static constexpr std::uint8_t kFooter = 4;

    RawBagWriter() = default;
    ~RawBagWriter();

    RawBagWriter(const RawBagWriter&) = delete;
    RawBagWriter& operator=(const RawBagWriter&) = delete;

    // Creates `directory` and `directory/raw_sensors.bin`, writes the header and
    // starts the writer thread.
    void open(const std::string& directory, const RawBagMetadata& metadata);

    void writeImu(const std::array<std::int16_t, 9>& sample, std::int64_t t_ns);
    void writeImage(const void* data, std::size_t size, std::uint64_t sequence, std::int64_t t_ns);
    void writeEvent(const std::string& json, std::int64_t t_ns);

    RawBagStats stats() const;
    // Stops accepting records, drains the queue, writes the footer and closes
    // the file. Safe to call more than once.
    RawBagStats close();

    const std::string& path() const { return path_; }

private:
    struct Record {
        std::uint8_t type = 0;
        std::uint64_t sequence = 0;
        std::int64_t t_ns = 0;
        std::vector<std::uint8_t> payload;
    };

    void push(Record record);
    void dropOldest(std::uint8_t type);
    void worker();
    bool writeRecord(const Record& record);
    bool writeFooter();

    std::string path_;
    std::FILE* file_ = nullptr;
    std::thread thread_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<Record> queue_;
    bool stopping_ = false;
    std::size_t queued_imu_ = 0;
    std::size_t queued_images_ = 0;
    std::size_t queued_bytes_ = 0;
    std::uint64_t sequence_ = 0;
    RawBagStats stats_;
};

struct RawBagRecord {
    std::uint8_t type = 0;
    std::uint64_t sequence = 0;
    std::int64_t t_ns = 0;
    std::vector<std::uint8_t> payload;
};

class RawBagReader {
public:
    RawBagReader() = default;
    ~RawBagReader();

    RawBagReader(const RawBagReader&) = delete;
    RawBagReader& operator=(const RawBagReader&) = delete;

    bool open(const std::string& directory);
    void close();

    const std::string& metadata_json() const { return metadata_json_; }
    const std::string& footer_json() const { return footer_json_; }
    const std::string& path() const { return path_; }

    // Returns false at end of file or on a malformed record.
    bool next(RawBagRecord& record);

private:
    std::FILE* file_ = nullptr;
    std::string path_;
    std::string metadata_json_;
    std::string footer_json_;
};

// Writes and reads back a small bag under /tmp. Returns 0 on success.
int run_raw_bag_self_test();

} // namespace flyrat::pilot
