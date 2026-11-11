// SPDX-License-Identifier: MIT
#include "flyrat/hardware/stereo_camera.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace flyrat::hardware {
namespace {

int xioctl(int fd, unsigned long request, void* arg)
{
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

std::string ioctl_error(const char* what)
{
    return std::string(what) + ": " + std::strerror(errno);
}

std::chrono::steady_clock::time_point to_time_point(const timeval& ts)
{
    return std::chrono::steady_clock::time_point(
        std::chrono::seconds(ts.tv_sec) + std::chrono::microseconds(ts.tv_usec));
}

} // namespace

struct StereoCamera::Impl {
    struct Buffer {
        void* start = nullptr;
        std::size_t length = 0;
    };

    explicit Impl(StereoConfig cfg)
        : config(std::move(cfg))
    {
    }

    StereoConfig config;
    int fd = -1;
    std::vector<Buffer> buffers;
    std::uint32_t source_stride = 0;
    std::atomic<int> actual_width { 0 };
    std::atomic<int> actual_height { 0 };

    std::atomic<bool> running { false };
    std::thread thread;
    mutable std::mutex mutex;
    std::optional<StereoFrame> latest;
    std::function<void(const StereoFrame&)> callback;

    std::atomic<std::uint64_t> frames { 0 };
    std::atomic<std::uint64_t> dropped { 0 };
    std::atomic<std::uint64_t> errors { 0 };
    std::atomic<std::int64_t> first_stamp_us { -1 };
    std::atomic<std::int64_t> last_stamp_us { 0 };
    bool have_sequence = false;
    std::uint32_t last_sequence = 0;
    std::string last_error;
    std::chrono::steady_clock::time_point started_at;

    void open_device()
    {
        fd = ::open(config.device.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            throw StereoError("open " + config.device + ": " + std::strerror(errno));
        }

        v4l2_capability cap {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            throw StereoError(ioctl_error("VIDIOC_QUERYCAP"));
        }
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
            !(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw StereoError(config.device + " is not a streaming video capture device");
        }

        v4l2_format format {};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = static_cast<std::uint32_t>(config.width);
        format.fmt.pix.height = static_cast<std::uint32_t>(config.height);
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            throw StereoError(ioctl_error("VIDIOC_S_FMT"));
        }
        if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
            throw StereoError("device rejected YUYV");
        }

        actual_width = static_cast<int>(format.fmt.pix.width);
        actual_height = static_cast<int>(format.fmt.pix.height);
        if (actual_width <= 0 || actual_height <= 0 || (actual_width % 2) != 0) {
            throw StereoError("driver returned an invalid side-by-side mode");
        }
        if (config.grayscale && (actual_width % 4) != 0) {
            throw StereoError("grayscale output requires a width divisible by 4");
        }
        source_stride = format.fmt.pix.bytesperline;
        if (source_stride < static_cast<std::uint32_t>(actual_width) * 2U) {
            source_stride = static_cast<std::uint32_t>(actual_width) * 2U;
        }

        v4l2_streamparm parm {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(config.fps);
        // Some UVC bridges ignore or reject this; the negotiated mode still
        // determines the real rate, so a failure here is not fatal.
        (void)xioctl(fd, VIDIOC_S_PARM, &parm);

        v4l2_requestbuffers request {};
        request.count = static_cast<std::uint32_t>(std::max(config.buffer_count, 2));
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0) {
            throw StereoError(ioctl_error("VIDIOC_REQBUFS"));
        }
        if (request.count < 2) {
            throw StereoError("device provided fewer than two capture buffers");
        }

        buffers.resize(request.count);
        for (std::uint32_t i = 0; i < request.count; ++i) {
            v4l2_buffer buffer {};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;
            if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
                throw StereoError(ioctl_error("VIDIOC_QUERYBUF"));
            }
            void* start = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                MAP_SHARED, fd, static_cast<off_t>(buffer.m.offset));
            if (start == MAP_FAILED) {
                throw StereoError("mmap capture buffer: " + std::string(std::strerror(errno)));
            }
            buffers[i].start = start;
            buffers[i].length = buffer.length;
            if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
                throw StereoError(ioctl_error("VIDIOC_QBUF"));
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            throw StereoError(ioctl_error("VIDIOC_STREAMON"));
        }
    }

    void close_device()
    {
        if (fd >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            (void)xioctl(fd, VIDIOC_STREAMOFF, &type);
        }
        for (const Buffer& buffer : buffers) {
            if (buffer.start != nullptr && buffer.start != MAP_FAILED) {
                munmap(buffer.start, buffer.length);
            }
        }
        buffers.clear();
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
        actual_width = 0;
        actual_height = 0;
        source_stride = 0;
    }

    void fail(const std::string& message)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            last_error = message;
        }
        ++errors;
        running = false;
    }

    StereoFrame make_frame(const Buffer& buffer, const v4l2_buffer& meta) const
    {
        const std::uint8_t* source = static_cast<const std::uint8_t*>(buffer.start);
        const std::size_t needed = static_cast<std::size_t>(source_stride) *
            static_cast<std::size_t>(actual_height);
        if (meta.bytesused < needed) {
            throw StereoError("short frame from device");
        }

        const int eye_width = actual_width / 2;
        const int eye_yuyv_bytes = eye_width * 2;
        StereoFrame frame;
        frame.width = eye_width;
        frame.height = actual_height;
        frame.format = config.grayscale ? StereoPixelFormat::kGray8
                                        : StereoPixelFormat::kYuyv;
        frame.stride = config.grayscale ? eye_width : eye_yuyv_bytes;

        if (config.grayscale) {
            frame.left.resize(static_cast<std::size_t>(eye_width) * actual_height);
            frame.right.resize(static_cast<std::size_t>(eye_width) * actual_height);
            for (int y = 0; y < actual_height; ++y) {
                const std::uint8_t* row = source + static_cast<std::size_t>(y) * source_stride;
                std::uint8_t* left = frame.left.data() + static_cast<std::size_t>(y) * eye_width;
                std::uint8_t* right = frame.right.data() + static_cast<std::size_t>(y) * eye_width;
                for (int x = 0; x < eye_width; x += 2) {
                    left[x] = row[x * 2];
                    left[x + 1] = row[x * 2 + 2];
                    right[x] = row[eye_yuyv_bytes + x * 2];
                    right[x + 1] = row[eye_yuyv_bytes + x * 2 + 2];
                }
            }
        } else {
            frame.left.resize(static_cast<std::size_t>(eye_yuyv_bytes) * actual_height);
            frame.right.resize(static_cast<std::size_t>(eye_yuyv_bytes) * actual_height);
            for (int y = 0; y < actual_height; ++y) {
                const std::uint8_t* row = source + static_cast<std::size_t>(y) * source_stride;
                std::uint8_t* left = frame.left.data() +
                    static_cast<std::size_t>(y) * eye_yuyv_bytes;
                std::uint8_t* right = frame.right.data() +
                    static_cast<std::size_t>(y) * eye_yuyv_bytes;
                std::memcpy(left, row, eye_yuyv_bytes);
                std::memcpy(right, row + eye_yuyv_bytes, eye_yuyv_bytes);
            }
        }
        return frame;
    }

    void publish(StereoFrame frame, std::uint64_t sequence,
        std::chrono::steady_clock::time_point stamp)
    {
        if (have_sequence && sequence > last_sequence + 1) {
            dropped += sequence - last_sequence - 1;
        }
        last_sequence = static_cast<std::uint32_t>(sequence);
        have_sequence = true;
        frame.sequence = sequence;
        frame.stamp = stamp;

        const auto stamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            frame.stamp.time_since_epoch()).count();
        std::int64_t expected = -1;
        first_stamp_us.compare_exchange_strong(expected, stamp_us);
        last_stamp_us = stamp_us;

        std::function<void(const StereoFrame&)> cb;
        {
            std::lock_guard<std::mutex> lock(mutex);
            cb = callback;
        }
        ++frames;
        if (cb) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                latest = frame;
            }
            try {
                cb(frame);
            } catch (...) {
                ++errors;
            }
        } else {
            std::lock_guard<std::mutex> lock(mutex);
            latest = std::move(frame);
        }
    }

    void capture_v4l2_loop()
    {
        while (running) {
            pollfd pfd {};
            pfd.fd = fd;
            pfd.events = POLLIN;
            const int ready = poll(&pfd, 1, 500);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                fail(ioctl_error("poll"));
                break;
            }
            if (ready == 0) {
                continue;
            }
            if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                fail("video device disconnected");
                break;
            }

            v4l2_buffer meta {};
            meta.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            meta.memory = V4L2_MEMORY_MMAP;
            if (xioctl(fd, VIDIOC_DQBUF, &meta) < 0) {
                if (errno == EAGAIN) {
                    continue;
                }
                fail(ioctl_error("VIDIOC_DQBUF"));
                break;
            }
            if (meta.index >= buffers.size()) {
                fail("driver returned an invalid buffer index");
                break;
            }

            try {
                publish(make_frame(buffers[meta.index], meta), meta.sequence,
                    to_time_point(meta.timestamp));
            } catch (const std::exception&) {
                ++errors;
            }

            if (xioctl(fd, VIDIOC_QBUF, &meta) < 0) {
                fail(ioctl_error("VIDIOC_QBUF"));
                break;
            }
        }
    }

};

StereoCamera::StereoCamera(StereoConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

StereoCamera::~StereoCamera()
{
    stop();
}

void StereoCamera::start()
{
    if (impl_->running) {
        throw StereoError("stereo camera is already running");
    }
    if (impl_->config.device.empty()) {
        throw StereoError("stereo device path is empty");
    }
    if (impl_->config.width <= 0 || impl_->config.height <= 0 ||
        impl_->config.fps <= 0) {
        throw StereoError("stereo width, height and fps must be positive");
    }

    impl_->frames = 0;
    impl_->dropped = 0;
    impl_->errors = 0;
    impl_->first_stamp_us = -1;
    impl_->last_stamp_us = 0;
    impl_->have_sequence = false;
    impl_->last_sequence = 0;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->last_error.clear();
        impl_->latest.reset();
    }

    try {
        impl_->open_device();
    } catch (...) {
        impl_->close_device();
        throw;
    }

    impl_->started_at = std::chrono::steady_clock::now();
    impl_->running = true;
    try {
        impl_->thread = std::thread([this] { impl_->capture_v4l2_loop(); });
    } catch (...) {
        impl_->running = false;
        impl_->close_device();
        throw;
    }
}

void StereoCamera::stop()
{
    impl_->running = false;
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
    impl_->close_device();
}

bool StereoCamera::running() const
{
    return impl_->running;
}

std::optional<StereoFrame> StereoCamera::latest() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->latest;
}

std::optional<StereoFrame> StereoCamera::takeLatest()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->latest) {
        return std::nullopt;
    }
    auto frame = std::move(*impl_->latest);
    impl_->latest.reset();
    return frame;
}

StereoStats StereoCamera::stats() const
{
    StereoStats result;
    result.frames = impl_->frames;
    result.dropped = impl_->dropped;
    result.errors = impl_->errors;
    result.width = impl_->actual_width / 2;
    result.height = impl_->actual_height;
    result.running = impl_->running;
    const std::int64_t first = impl_->first_stamp_us;
    const std::int64_t last = impl_->last_stamp_us;
    if (first >= 0 && last > first && result.frames > 1) {
        // Measure between V4L2 timestamps so the first-frame latency and the
        // time spent inside start() do not depress the reported frame rate.
        result.fps = static_cast<double>(result.frames - 1) * 1000000.0 /
            static_cast<double>(last - first);
    } else if (impl_->started_at.time_since_epoch().count() != 0) {
        const double seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - impl_->started_at).count();
        if (seconds > 0.0) {
            result.fps = static_cast<double>(result.frames) / seconds;
        }
    }
    return result;
}

void StereoCamera::set_callback(std::function<void(const StereoFrame&)> callback)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->callback = std::move(callback);
}

} // namespace flyrat::hardware
