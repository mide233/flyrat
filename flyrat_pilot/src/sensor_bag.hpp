// SPDX-License-Identifier: MIT
//
// On-vehicle sensor bag: camera JPEG frames and calibrated IMU samples.
//
// The format is a small append-only binary stream so the pilot can record on
// the RK3566 without pulling in rosbag or a compression library. A bag is a
// single file, sensor_bag.bin, inside the directory passed to --record.
//
// Header (24 bytes, little endian):
//   char     magic[8]   "FLYRATB1"
//   uint32   width
//   uint32   height
//   uint32   fps
//   uint32   reserved
//
// Every record starts with a 24-byte header:
//   uint32   type       1 = camera JPEG, 2 = IMU
//   uint32   size       payload bytes
//   uint64   sequence   camera frame sequence, 0 for IMU
//   double   stamp_s    original sensor timestamp (Unix seconds)
//
// Camera payload is the complete MJPEG frame. IMU payload is six little-endian
// doubles: accel xyz, gyro xyz, in the calibrated body frame.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "flyrat/odometry/odometry.hpp"

namespace flyrat::pilot {

struct BagStats {
    std::uint64_t frames = 0;
    std::uint64_t imu_samples = 0;
    std::uint64_t dropped = 0;
    std::uint64_t write_errors = 0;
    std::uint64_t bytes = 0;
};

struct BagInfo {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t fps = 0;
    std::uint64_t frames = 0;
    std::uint64_t imu_samples = 0;
    double duration_s = 0.0;
    double first_stamp_s = 0.0;
    double last_frame_stamp_s = 0.0;
};

class BagWriter {
public:
    BagWriter() = default;
    ~BagWriter();

    BagWriter(const BagWriter&) = delete;
    BagWriter& operator=(const BagWriter&) = delete;

    // Creates `directory` and truncates any previous bag in it.
    bool open(const std::string& directory, int width, int height, int fps);
    bool enabled() const { return file_ != nullptr; }

    // Copies the JPEG before returning, so the caller can reuse its buffer.
    void writeFrame(std::uint64_t sequence, double stamp_s, const void* data,
        std::size_t size);
    void writeImu(const odometry::ImuSample& sample);

    // Drains the queue, flushes and closes the file. Safe to call twice.
    BagStats close();

    const std::string& path() const { return path_; }

private:
    struct Record {
        std::uint32_t type = 0;
        std::uint64_t sequence = 0;
        double stamp_s = 0.0;
        std::vector<std::uint8_t> payload;
    };

    void push(Record record);
    void worker();
    bool writeRecord(const Record& record);

    std::FILE* file_ = nullptr;
    std::string path_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Record> queue_;
    bool stopping_ = false;
    BagStats stats_;
};

struct BagRecord {
    std::uint32_t type = 0;
    std::uint64_t sequence = 0;
    double stamp_s = 0.0;
    std::vector<std::uint8_t> payload;
};

class BagReader {
public:
    enum class Type : std::uint32_t {
        kFrame = 1,
        kImu = 2,
    };

    BagReader() = default;
    ~BagReader();

    BagReader(const BagReader&) = delete;
    BagReader& operator=(const BagReader&) = delete;

    // Reads the header and scans the whole file for stats, then rewinds.
    bool open(const std::string& directory);
    bool isOpen() const { return file_ != nullptr; }
    const BagInfo& info() const { return info_; }
    const std::string& path() const { return path_; }
    const std::string& lastError() const { return last_error_; }

    // Returns false at end of file or on a malformed record.
    bool next(BagRecord& record);

private:
    void fail(const std::string& message);

    std::FILE* file_ = nullptr;
    std::string path_;
    std::string last_error_;
    long data_offset_ = 0;
    BagInfo info_;
};

} // namespace flyrat::pilot
