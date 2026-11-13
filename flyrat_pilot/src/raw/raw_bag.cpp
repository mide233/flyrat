// SPDX-License-Identifier: MIT

#include "raw/raw_bag.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>

#include <unistd.h>

namespace flyrat::pilot {

namespace {

constexpr char kMagic[8] = {'F', 'L', 'Y', 'R', 'A', 'T', 'R', 'W'};
constexpr std::uint16_t kFormatVersion = 1;
constexpr std::size_t kRecordHeaderSize = 24;
constexpr std::size_t kMaxImuQueue = 8192;
constexpr std::size_t kMaxImageQueue = 8;
constexpr std::size_t kMaxQueuedBytes = 128ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaxHeaderBytes = 1024ULL * 1024ULL;
constexpr std::uint32_t kMaxRecordBytes = 256ULL * 1024ULL * 1024ULL;

std::string escape_json(const std::string& value) {
    std::ostringstream out;
    for (const char raw : value) {
        const unsigned char c = static_cast<unsigned char>(raw);
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
                    << std::dec;
            } else {
                out << raw;
            }
        }
    }
    return out.str();
}

void append_u16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF));
    }
}

void append_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF));
    }
}

std::uint16_t read_u16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) | static_cast<std::uint16_t>(data[1]) << 8;
}

std::uint32_t read_u32(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) | static_cast<std::uint32_t>(data[1]) << 8
         | static_cast<std::uint32_t>(data[2]) << 16 | static_cast<std::uint32_t>(data[3]) << 24;
}

std::uint64_t read_u64(const std::uint8_t* data) {
    std::uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | data[i];
    }
    return value;
}

std::int64_t read_i64(const std::uint8_t* data) {
    return static_cast<std::int64_t>(read_u64(data));
}

bool read_exact(std::FILE* file, void* data, std::size_t size) {
    return std::fread(data, 1, size, file) == size;
}

bool write_all(std::FILE* file, const void* data, std::size_t size) {
    return std::fwrite(data, 1, size, file) == size;
}

std::int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string stats_json(const RawBagStats& stats) {
    std::ostringstream out;
    out << "{\"imu_records\":" << stats.imu_records << ",\"image_records\":" << stats.image_records
        << ",\"event_records\":" << stats.event_records << ",\"dropped_imu\":" << stats.dropped_imu
        << ",\"dropped_images\":" << stats.dropped_images
        << ",\"write_errors\":" << stats.write_errors
        << ",\"bytes_written\":" << stats.bytes_written << '}';
    return out.str();
}

} // namespace

std::string raw_bag_metadata_json(const RawBagMetadata& metadata) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{"
        << "\"tool\":\"" << escape_json(metadata.tool) << "\""
        << ",\"format_version\":" << metadata.format_version << ",\"git_commit\":\""
        << escape_json(metadata.git_commit) << "\""
        << ",\"created_unix_ns\":" << metadata.created_unix_ns << ",\"imu\":{"
        << "\"enabled\":" << (metadata.imu_enabled ? "true" : "false") << ",\"device\":\""
        << escape_json(metadata.msp_device) << "\""
        << ",\"baud\":" << metadata.msp_baud << ",\"rate_hz\":" << metadata.imu_rate_hz
        << ",\"acc_lsb_per_g\":" << metadata.acc_lsb_per_g
        << ",\"gyro_lsb_per_dps\":" << metadata.gyro_lsb_per_dps << ",\"accel_axis\":\""
        << escape_json(metadata.accel_axis) << "\""
        << ",\"gyro_axis\":\"" << escape_json(metadata.gyro_axis) << "\""
        << "}"
        << ",\"camera\":{"
        << "\"enabled\":" << (metadata.camera_enabled ? "true" : "false") << ",\"device\":\""
        << escape_json(metadata.camera_device) << "\""
        << ",\"width\":" << metadata.width << ",\"height\":" << metadata.height << ",\"format\":\""
        << escape_json(metadata.camera_format) << "\""
        << ",\"layout\":\"" << escape_json(metadata.camera_layout) << "\""
        << ",\"capture_fps_requested\":" << metadata.capture_fps_requested
        << ",\"capture_fps_negotiated\":" << metadata.capture_fps_negotiated
        << ",\"delivery_fps\":" << metadata.delivery_fps << "}"
        << ",\"clocks\":{"
        << "\"source\":\"steady_clock\""
        << ",\"start_steady_ns\":" << metadata.start_steady_ns
        << ",\"start_system_ns\":" << metadata.start_system_ns << "}"
        << "}";
    return out.str();
}

RawBagWriter::~RawBagWriter() {
    try {
        close();
    } catch (...) {}
}

void RawBagWriter::open(const std::string& directory, const RawBagMetadata& metadata) {
    if (file_ != nullptr) {
        throw std::runtime_error("raw bag is already open");
    }

    const std::filesystem::path dir(directory);
    std::error_code error;
    std::filesystem::create_directories(dir, error);
    if (error) {
        throw std::runtime_error("cannot create " + dir.string() + ": " + error.message());
    }

    const auto path = dir / "raw_sensors.bin";
    path_ = path.string();
    file_ = std::fopen(path_.c_str(), "wb");
    if (file_ == nullptr) {
        throw std::runtime_error("cannot open " + path_ + ": " + std::strerror(errno));
    }

    const std::string header = raw_bag_metadata_json(metadata);
    std::vector<std::uint8_t> prefix;
    prefix.insert(prefix.end(), kMagic, kMagic + sizeof(kMagic));
    append_u16(prefix, kFormatVersion);
    append_u16(prefix, 0);
    append_u32(prefix, static_cast<std::uint32_t>(header.size()));
    stats_ = {};
    sequence_ = 0;
    queued_imu_ = 0;
    queued_images_ = 0;
    queued_bytes_ = 0;
    const bool ok = write_all(file_, prefix.data(), prefix.size())
                 && write_all(file_, header.data(), header.size());
    if (!ok) {
        std::fclose(file_);
        file_ = nullptr;
        throw std::runtime_error("cannot write raw bag header to " + path_);
    }
    stats_.bytes_written += prefix.size() + header.size();

    stopping_ = false;
    thread_ = std::thread(&RawBagWriter::worker, this);
}

void RawBagWriter::writeImu(const std::array<std::int16_t, 9>& sample, std::int64_t t_ns) {
    Record record;
    record.type = kImu;
    record.sequence = ++sequence_;
    record.t_ns = t_ns;
    record.payload.resize(sample.size() * 2);
    for (std::size_t i = 0; i < sample.size(); ++i) {
        const auto value = static_cast<std::uint16_t>(sample[i]);
        record.payload[2 * i] = static_cast<std::uint8_t>(value & 0xFF);
        record.payload[2 * i + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    }
    push(std::move(record));
}

void RawBagWriter::writeImage(
    const void* data, std::size_t size, std::uint64_t sequence, std::int64_t t_ns) {
    Record record;
    record.type = kImage;
    record.sequence = sequence;
    record.t_ns = t_ns;
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    record.payload.assign(bytes, bytes + size);
    push(std::move(record));
}

void RawBagWriter::writeEvent(const std::string& json, std::int64_t t_ns) {
    Record record;
    record.type = kEvent;
    record.sequence = ++sequence_;
    record.t_ns = t_ns;
    record.payload.assign(json.begin(), json.end());
    push(std::move(record));
}

void RawBagWriter::push(Record record) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) {
            if (record.type == kImu) {
                ++stats_.dropped_imu;
            } else if (record.type == kImage) {
                ++stats_.dropped_images;
            }
            return;
        }

        const std::size_t size = record.payload.size();
        if (record.type == kImu) {
            while (queued_imu_ >= kMaxImuQueue) {
                const auto before = queue_.size();
                dropOldest(kImu);
                if (queue_.size() == before) {
                    break;
                }
            }
        } else if (record.type == kImage) {
            if (size > kMaxQueuedBytes) {
                ++stats_.dropped_images;
                return;
            }
            while (queued_images_ >= kMaxImageQueue || queued_bytes_ + size > kMaxQueuedBytes) {
                const auto before = queue_.size();
                dropOldest(kImage);
                if (queue_.size() == before) {
                    break;
                }
            }
        }

        if (record.type != kEvent && queued_bytes_ + size > kMaxQueuedBytes) {
            if (record.type == kImu) {
                ++stats_.dropped_imu;
            } else {
                ++stats_.dropped_images;
            }
            return;
        }

        if (record.type == kImu) {
            ++queued_imu_;
        } else if (record.type == kImage) {
            ++queued_images_;
        }
        queued_bytes_ += size;
        queue_.push_back(std::move(record));
    }
    condition_.notify_one();
}

void RawBagWriter::dropOldest(std::uint8_t type) {
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
        if (it->type != type) {
            continue;
        }
        queued_bytes_ -= it->payload.size();
        if (type == kImu) {
            --queued_imu_;
            ++stats_.dropped_imu;
        } else if (type == kImage) {
            --queued_images_;
            ++stats_.dropped_images;
        }
        queue_.erase(it);
        return;
    }
}

void RawBagWriter::worker() {
    while (true) {
        Record record;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (queue_.empty() && stopping_) {
                break;
            }
            record = std::move(queue_.front());
            queue_.pop_front();
            queued_bytes_ -= record.payload.size();
            if (record.type == kImu) {
                --queued_imu_;
            } else if (record.type == kImage) {
                --queued_images_;
            }
        }

        const bool ok = writeRecord(record);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!ok) {
                ++stats_.write_errors;
            } else {
                if (record.type == kImu) {
                    ++stats_.imu_records;
                } else if (record.type == kImage) {
                    ++stats_.image_records;
                } else if (record.type == kEvent) {
                    ++stats_.event_records;
                }
                stats_.bytes_written += kRecordHeaderSize + record.payload.size();
            }
        }
    }

    writeFooter();
}

bool RawBagWriter::writeRecord(const Record& record) {
    if (file_ == nullptr) {
        return false;
    }

    std::vector<std::uint8_t> header;
    header.reserve(kRecordHeaderSize);
    header.push_back(record.type);
    header.push_back(0);
    append_u16(header, 0);
    append_u32(header, static_cast<std::uint32_t>(record.payload.size()));
    append_u64(header, record.sequence);
    append_u64(header, static_cast<std::uint64_t>(record.t_ns));

    if (!write_all(file_, header.data(), header.size())) {
        return false;
    }
    if (!record.payload.empty()
        && !write_all(file_, record.payload.data(), record.payload.size())) {
        return false;
    }
    return true;
}

bool RawBagWriter::writeFooter() {
    if (file_ == nullptr) {
        return true;
    }

    RawBagStats snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = stats_;
        snapshot.queued_imu = queued_imu_;
        snapshot.queued_images = queued_images_;
        snapshot.queued_bytes = queued_bytes_;
    }
    const std::string json = "{\"footer\":" + stats_json(snapshot) + "}";
    Record record;
    record.type = kFooter;
    record.sequence = ++sequence_;
    record.t_ns = steady_now_ns();
    record.payload.assign(json.begin(), json.end());
    return writeRecord(record);
}

RawBagStats RawBagWriter::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    RawBagStats out = stats_;
    out.queued_imu = queued_imu_;
    out.queued_images = queued_images_;
    out.queued_bytes = queued_bytes_;
    return out;
}

RawBagStats RawBagWriter::close() {
    if (file_ == nullptr && !thread_.joinable()) {
        return stats();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    condition_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }

    if (file_ != nullptr) {
        std::fflush(file_);
        ::fsync(::fileno(file_));
        std::fclose(file_);
        file_ = nullptr;
    }
    return stats();
}

RawBagReader::~RawBagReader() { close(); }

bool RawBagReader::open(const std::string& directory) {
    close();
    const std::filesystem::path path = std::filesystem::path(directory) / "raw_sensors.bin";
    path_ = path.string();
    file_ = std::fopen(path_.c_str(), "rb");
    if (file_ == nullptr) {
        return false;
    }

    std::uint8_t prefix[16]{};
    if (!read_exact(file_, prefix, sizeof(prefix))
        || std::memcmp(prefix, kMagic, sizeof(kMagic)) != 0) {
        close();
        return false;
    }
    const std::uint16_t version = read_u16(prefix + 8);
    const std::uint32_t header_size = read_u32(prefix + 12);
    if (version != kFormatVersion || header_size == 0 || header_size > kMaxHeaderBytes) {
        close();
        return false;
    }
    std::string header(header_size, '\0');
    if (!read_exact(file_, header.data(), header.size())) {
        close();
        return false;
    }
    metadata_json_ = std::move(header);
    return true;
}

void RawBagReader::close() {
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }
    metadata_json_.clear();
    footer_json_.clear();
}

bool RawBagReader::next(RawBagRecord& record) {
    if (file_ == nullptr) {
        return false;
    }

    std::uint8_t header[kRecordHeaderSize]{};
    const std::size_t got = std::fread(header, 1, sizeof(header), file_);
    if (got == 0) {
        return false;
    }
    if (got != sizeof(header)) {
        return false;
    }

    record.type = header[0];
    record.sequence = read_u64(header + 8);
    record.t_ns = read_i64(header + 16);
    const std::uint32_t size = read_u32(header + 4);
    if (size > kMaxRecordBytes) {
        return false;
    }

    record.payload.resize(size);
    if (size > 0 && !read_exact(file_, record.payload.data(), size)) {
        return false;
    }

    if (record.type == RawBagWriter::kFooter) {
        footer_json_.assign(record.payload.begin(), record.payload.end());
    }
    return true;
}

int run_raw_bag_self_test() {
    const auto directory = std::filesystem::temp_directory_path()
                         / ("flyrat_raw_selftest_" + std::to_string(::getpid()));
    std::error_code error;
    std::filesystem::remove_all(directory, error);

    try {
        RawBagMetadata metadata;
        metadata.imu_enabled = true;
        metadata.msp_device = "/dev/null";
        metadata.imu_rate_hz = 200;
        metadata.camera_enabled = true;
        metadata.camera_device = "/dev/null";
        metadata.width = 1280;
        metadata.height = 480;
        metadata.capture_fps_requested = 60;
        metadata.capture_fps_negotiated = 60.0;
        metadata.delivery_fps = 20;
        metadata.start_steady_ns = steady_now_ns();
        metadata.start_system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count();

        const std::int64_t base_ns = metadata.start_steady_ns - 6000000000LL;
        RawBagWriter writer;
        writer.open(directory.string(), metadata);
        for (int i = 0; i < 1000; ++i) {
            std::array<std::int16_t, 9> sample{};
            for (std::size_t j = 0; j < sample.size(); ++j) {
                sample[j] = static_cast<std::int16_t>(i + static_cast<int>(j));
            }
            writer.writeImu(sample, base_ns + i * 5000000LL);
        }
        for (int i = 0; i < 20; ++i) {
            const std::string jpeg = "jpeg-frame-" + std::to_string(i);
            writer.writeImage(
                jpeg.data(), jpeg.size(), static_cast<std::uint64_t>(i),
                base_ns + 5000000000LL + i * 50000000LL);
            // Give the writer thread a chance to drain the small image queue so
            // the self-test measures the format, not the bounded-queue policy.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        writer.writeEvent("{\"event\":\"self-test\"}", steady_now_ns());
        const RawBagStats written = writer.close();
        if (written.imu_records != 1000 || written.image_records != 20 || written.event_records != 1
            || written.write_errors != 0) {
            std::fprintf(
                stderr,
                "raw bag self-test: writer counts imu=%llu image=%llu event=%llu errors=%llu\n",
                static_cast<unsigned long long>(written.imu_records),
                static_cast<unsigned long long>(written.image_records),
                static_cast<unsigned long long>(written.event_records),
                static_cast<unsigned long long>(written.write_errors));
            return 1;
        }

        RawBagReader reader;
        if (!reader.open(directory.string())) {
            std::fprintf(stderr, "raw bag self-test: cannot reopen bag\n");
            return 1;
        }
        std::uint64_t imu = 0;
        std::uint64_t images = 0;
        std::uint64_t events = 0;
        std::int64_t last_stamp = -1;
        RawBagRecord record;
        while (reader.next(record)) {
            if (record.t_ns < last_stamp) {
                std::fprintf(stderr, "raw bag self-test: timestamps are not monotonic\n");
                return 1;
            }
            last_stamp = record.t_ns;
            if (record.type == RawBagWriter::kImu) {
                if (record.payload.size() != 18) {
                    std::fprintf(stderr, "raw bag self-test: bad IMU payload size\n");
                    return 1;
                }
                ++imu;
            } else if (record.type == RawBagWriter::kImage) {
                ++images;
            } else if (record.type == RawBagWriter::kEvent) {
                ++events;
            }
        }
        if (imu != 1000 || images != 20 || events != 1) {
            std::fprintf(
                stderr, "raw bag self-test: reader counts imu=%llu image=%llu event=%llu\n",
                static_cast<unsigned long long>(imu), static_cast<unsigned long long>(images),
                static_cast<unsigned long long>(events));
            return 1;
        }
        reader.close();
        std::filesystem::remove_all(directory, error);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "raw bag self-test failed: %s\n", error.what());
        return 1;
    }

    std::printf("raw bag self-test: ok (1000 IMU, 20 images)\n");
    return 0;
}

} // namespace flyrat::pilot
