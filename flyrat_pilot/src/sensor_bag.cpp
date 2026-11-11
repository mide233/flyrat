// SPDX-License-Identifier: MIT
#include "sensor_bag.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>

namespace flyrat::pilot {
namespace {

constexpr char kMagic[8] = { 'F', 'L', 'Y', 'R', 'A', 'T', 'B', '1' };
constexpr std::size_t kFileHeaderSize = 24;
constexpr std::size_t kRecordHeaderSize = 24;
constexpr std::size_t kMaxPayload = 16U * 1024U * 1024U;
constexpr std::uint32_t kTypeFrame = 1;
constexpr std::uint32_t kTypeImu = 2;
constexpr std::size_t kQueueLimit = 64;

constexpr int kImuDoubles = 6;

template <typename T>
void put(std::vector<std::uint8_t>& out, const T& value)
{
    static_assert(std::is_trivially_copyable<T>::value, "plain type required");
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(T));
}

template <typename T>
bool get(const std::uint8_t* data, T& value)
{
    static_assert(std::is_trivially_copyable<T>::value, "plain type required");
    std::memcpy(&value, data, sizeof(T));
    return true;
}

} // namespace

BagWriter::~BagWriter()
{
    close();
}

bool BagWriter::open(const std::string& directory, int width, int height, int fps)
{
    close();
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        std::cerr << "flyrat_pilot: cannot create bag directory " << directory
                  << ": " << error.message() << "\n";
        return false;
    }
    path_ = directory + "/sensor_bag.bin";
    file_ = std::fopen(path_.c_str(), "wb");
    if (file_ == nullptr) {
        std::cerr << "flyrat_pilot: cannot open " << path_ << "\n";
        return false;
    }

    std::vector<std::uint8_t> header;
    header.insert(header.end(), kMagic, kMagic + sizeof(kMagic));
    put(header, static_cast<std::uint32_t>(width));
    put(header, static_cast<std::uint32_t>(height));
    put(header, static_cast<std::uint32_t>(fps));
    put(header, static_cast<std::uint32_t>(0));
    if (std::fwrite(header.data(), 1, header.size(), file_) != header.size()) {
        std::cerr << "flyrat_pilot: cannot write bag header\n";
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }

    stats_ = BagStats {};
    stats_.bytes = header.size();
    stopping_ = false;
    thread_ = std::thread([this] { worker(); });
    return true;
}

void BagWriter::writeFrame(std::uint64_t sequence, double stamp_s,
    const void* data, std::size_t size)
{
    if (!enabled() || data == nullptr || size == 0) {
        return;
    }
    Record record;
    record.type = kTypeFrame;
    record.sequence = sequence;
    record.stamp_s = stamp_s;
    record.payload.resize(size);
    std::memcpy(record.payload.data(), data, size);
    push(std::move(record));
}

void BagWriter::writeImu(const odometry::ImuSample& sample)
{
    if (!enabled()) {
        return;
    }
    Record record;
    record.type = kTypeImu;
    record.stamp_s = sample.timestamp;
    record.payload.reserve(kImuDoubles * sizeof(double));
    for (int i = 0; i < 3; ++i) {
        put(record.payload, sample.accel[i]);
    }
    for (int i = 0; i < 3; ++i) {
        put(record.payload, sample.gyro[i]);
    }
    push(std::move(record));
}

void BagWriter::push(Record record)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= kQueueLimit) {
            ++stats_.dropped;
            return;
        }
        queue_.push_back(std::move(record));
    }
    cv_.notify_one();
}

void BagWriter::worker()
{
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (queue_.empty()) {
            if (stopping_) {
                break;
            }
            continue;
        }
        Record record = std::move(queue_.front());
        queue_.pop_front();
        lock.unlock();
        writeRecord(record);
        lock.lock();
    }
}

bool BagWriter::writeRecord(const Record& record)
{
    std::vector<std::uint8_t> header;
    header.reserve(kRecordHeaderSize);
    put(header, record.type);
    put(header, static_cast<std::uint32_t>(record.payload.size()));
    put(header, record.sequence);
    put(header, record.stamp_s);

    std::lock_guard<std::mutex> lock(mutex_);
    const bool ok = std::fwrite(header.data(), 1, header.size(), file_)
            == header.size()
        && std::fwrite(record.payload.data(), 1, record.payload.size(), file_)
            == record.payload.size();
    if (!ok) {
        ++stats_.write_errors;
        return false;
    }
    stats_.bytes += header.size() + record.payload.size();
    if (record.type == kTypeFrame) {
        ++stats_.frames;
    } else if (record.type == kTypeImu) {
        ++stats_.imu_samples;
    }
    return true;
}

BagStats BagWriter::close()
{
    if (thread_.joinable()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }
    if (file_ != nullptr) {
        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
    }
    return stats_;
}

BagReader::~BagReader()
{
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

void BagReader::fail(const std::string& message)
{
    if (last_error_.empty()) {
        last_error_ = path_ + ": " + message;
    }
}

bool BagReader::open(const std::string& directory)
{
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }
    last_error_.clear();
    path_ = directory + "/sensor_bag.bin";
    file_ = std::fopen(path_.c_str(), "rb");
    if (file_ == nullptr) {
        fail("cannot open");
        return false;
    }

    std::uint8_t header[kFileHeaderSize];
    if (std::fread(header, 1, sizeof(header), file_) != sizeof(header)
        || std::memcmp(header, kMagic, sizeof(kMagic)) != 0) {
        fail("not a FlyRat sensor bag");
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }
    get(header + 8, info_.width);
    get(header + 12, info_.height);
    get(header + 16, info_.fps);

    data_offset_ = std::ftell(file_);
    if (data_offset_ < 0) {
        fail("cannot seek");
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }

    // Scan once for the summary, then rewind for playback.
    double first = 0.0;
    double last = 0.0;
    bool have_stamp = false;
    BagRecord record;
    while (next(record)) {
        if (record.type == kTypeFrame) {
            ++info_.frames;
            info_.last_frame_stamp_s = record.stamp_s;
        } else if (record.type == kTypeImu) {
            ++info_.imu_samples;
        }
        if (!have_stamp) {
            first = record.stamp_s;
            have_stamp = true;
        }
        last = record.stamp_s;
    }
    if (!last_error_.empty()) {
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }
    info_.first_stamp_s = first;
    info_.duration_s = std::max(0.0, last - first);
    if (std::fseek(file_, data_offset_, SEEK_SET) != 0) {
        fail("cannot rewind");
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }
    return true;
}

bool BagReader::next(BagRecord& record)
{
    if (file_ == nullptr || !last_error_.empty()) {
        return false;
    }
    std::uint8_t header[kRecordHeaderSize];
    const std::size_t got = std::fread(header, 1, sizeof(header), file_);
    if (got == 0) {
        return false;
    }
    if (got != sizeof(header)) {
        fail("truncated record header");
        return false;
    }

    get(header, record.type);
    std::uint32_t size = 0;
    get(header + 4, size);
    get(header + 8, record.sequence);
    get(header + 16, record.stamp_s);
    if (size > kMaxPayload) {
        fail("record payload too large");
        return false;
    }

    record.payload.resize(size);
    if (size > 0
        && std::fread(record.payload.data(), 1, size, file_) != size) {
        fail("truncated record payload");
        return false;
    }
    return true;
}

} // namespace flyrat::pilot
