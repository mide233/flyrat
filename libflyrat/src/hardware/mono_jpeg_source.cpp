// SPDX-License-Identifier: MIT
#include "flyrat/hardware/mono_jpeg_source.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
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

constexpr std::size_t kMaxJpegBytes = 8U * 1024U * 1024U;

} // namespace

struct MonoJpegSource::Impl {
    struct Buffer {
        void* start = nullptr;
        std::size_t length = 0;
    };

    explicit Impl(MonoJpegConfig cfg)
        : config(std::move(cfg))
    {
    }

    MonoJpegConfig config;
    int fd = -1;
    std::vector<Buffer> buffers;
    std::atomic<int> actual_width { 0 };
    std::atomic<int> actual_height { 0 };

    std::atomic<bool> running { false };
    std::thread thread;
    mutable std::mutex mutex;
    std::optional<MonoJpegFrame> latest;
    std::function<void(const MonoJpegFrame&)> callback;

    std::atomic<std::uint64_t> frames { 0 };
    std::atomic<std::uint64_t> dropped { 0 };
    std::atomic<std::uint64_t> errors { 0 };
    std::atomic<std::int64_t> first_stamp_us { -1 };
    std::atomic<std::int64_t> last_stamp_us { 0 };
    bool have_sequence = false;
    std::uint32_t last_sequence = 0;
    bool have_published = false;
    std::chrono::steady_clock::time_point next_publish {};
    std::string last_error;

    void write_control(std::uint32_t id, int value, const char* name)
    {
        v4l2_control control {};
        control.id = id;
        control.value = value;
        if (xioctl(fd, VIDIOC_S_CTRL, &control) < 0) {
            std::fprintf(stderr, "camera %s: %s=%d unsupported (%s)\n",
                config.device.c_str(), name, value, std::strerror(errno));
            return;
        }
        v4l2_control readback {};
        readback.id = id;
        if (xioctl(fd, VIDIOC_G_CTRL, &readback) == 0) {
            std::fprintf(stderr, "camera %s: %s=%d (readback %d)\n",
                config.device.c_str(), name, value, readback.value);
        } else {
            std::fprintf(stderr, "camera %s: %s=%d\n",
                config.device.c_str(), name, value);
        }
    }

    // UVC exposure is expressed in 100 us units.  Manual exposure with
    // auto-priority disabled keeps the frame interval constant, so the camera
    // cannot trade frame rate for a long exposure and smear fast motion.
    void apply_exposure_controls()
    {
        if (config.exposure_us > 0) {
            write_control(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "exposure_auto");
            const int ticks = std::max(1, (config.exposure_us + 50) / 100);
            write_control(V4L2_CID_EXPOSURE_ABSOLUTE, ticks, "exposure_absolute");
            write_control(V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0, "exposure_auto_priority");
        }
        if (config.gain >= 0) {
            write_control(V4L2_CID_GAIN, config.gain, "gain");
        }
    }

    void open_device()
    {
        fd = ::open(config.device.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            throw MonoJpegError("open " + config.device + ": " + std::strerror(errno));
        }

        v4l2_capability cap {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            throw MonoJpegError(ioctl_error("VIDIOC_QUERYCAP"));
        }
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
            !(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw MonoJpegError(config.device + " is not a streaming video capture device");
        }

        v4l2_format format {};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = static_cast<std::uint32_t>(config.width);
        format.fmt.pix.height = static_cast<std::uint32_t>(config.height);
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        format.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            throw MonoJpegError(ioctl_error("VIDIOC_S_FMT"));
        }
        if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
            throw MonoJpegError("device rejected MJPEG");
        }
        if (static_cast<int>(format.fmt.pix.width) != config.width ||
            static_cast<int>(format.fmt.pix.height) != config.height) {
            throw MonoJpegError("device did not accept the requested MJPEG mode");
        }

        actual_width = config.width;
        actual_height = config.height;

        v4l2_streamparm parm {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(config.fps);
        (void)xioctl(fd, VIDIOC_S_PARM, &parm);

        apply_exposure_controls();

        v4l2_requestbuffers request {};
        request.count = static_cast<std::uint32_t>(std::max(config.buffer_count, 2));
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0) {
            throw MonoJpegError(ioctl_error("VIDIOC_REQBUFS"));
        }
        if (request.count < 2) {
            throw MonoJpegError("device provided fewer than two capture buffers");
        }

        buffers.resize(request.count);
        for (std::uint32_t i = 0; i < request.count; ++i) {
            v4l2_buffer buffer {};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = i;
            if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
                throw MonoJpegError(ioctl_error("VIDIOC_QUERYBUF"));
            }
            void* start = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                MAP_SHARED, fd, static_cast<off_t>(buffer.m.offset));
            if (start == MAP_FAILED) {
                throw MonoJpegError("mmap capture buffer: " + std::string(std::strerror(errno)));
            }
            buffers[i].start = start;
            buffers[i].length = buffer.length;
            if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
                throw MonoJpegError(ioctl_error("VIDIOC_QBUF"));
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            throw MonoJpegError(ioctl_error("VIDIOC_STREAMON"));
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

    MonoJpegFrame make_frame(const Buffer& buffer, const v4l2_buffer& meta) const
    {
        if (meta.bytesused == 0 || meta.bytesused > kMaxJpegBytes) {
            throw MonoJpegError("invalid MJPEG frame size");
        }
        MonoJpegFrame frame;
        frame.width = actual_width;
        frame.height = actual_height;
        const std::uint8_t* source = static_cast<const std::uint8_t*>(buffer.start);
        frame.jpeg.assign(source, source + meta.bytesused);
        return frame;
    }

    void publish(MonoJpegFrame frame, const v4l2_buffer& meta)
    {
        auto stamp = to_time_point(meta.timestamp);
        if (stamp.time_since_epoch().count() == 0) {
            stamp = std::chrono::steady_clock::now();
        }

        if (have_sequence && meta.sequence > last_sequence + 1) {
            dropped += meta.sequence - last_sequence - 1;
        }
        last_sequence = meta.sequence;
        have_sequence = true;

        // Some UVC cameras only expose 120/210 Hz modes even when a lower
        // frame rate is requested. Drop frames after capture so the host sees
        // a stable 15 Hz stream instead of a flood of stale frames.
        const auto interval = std::chrono::microseconds(
            1000000 / std::max(config.fps, 1));
        if (!have_published) {
            have_published = true;
            next_publish = stamp + interval;
        } else {
            if (stamp < next_publish) {
                ++dropped;
                return;
            }
            next_publish += interval;
            // If the camera was paused or stalled for more than one interval,
            // resynchronise instead of emitting a burst.
            if (next_publish <= stamp) {
                next_publish = stamp + interval;
            }
        }

        frame.sequence = meta.sequence;
        frame.stamp = stamp;

        const auto stamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            frame.stamp.time_since_epoch()).count();
        std::int64_t expected = -1;
        first_stamp_us.compare_exchange_strong(expected, stamp_us);
        last_stamp_us = stamp_us;

        std::function<void(const MonoJpegFrame&)> cb;
        {
            std::lock_guard<std::mutex> lock(mutex);
            latest = frame;
            cb = callback;
        }
        ++frames;
        if (cb) {
            try {
                cb(frame);
            } catch (...) {
                ++errors;
            }
        }
    }

    void capture_loop()
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
                publish(make_frame(buffers[meta.index], meta), meta);
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

MonoJpegSource::MonoJpegSource(MonoJpegConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

MonoJpegSource::~MonoJpegSource()
{
    stop();
}

void MonoJpegSource::start()
{
    if (impl_->running) {
        throw MonoJpegError("camera JPEG source is already running");
    }
    if (impl_->config.device.empty()) {
        throw MonoJpegError("camera JPEG device path is empty");
    }
    if (impl_->config.width <= 0 || impl_->config.height <= 0 ||
        impl_->config.fps <= 0) {
        throw MonoJpegError("camera JPEG width, height and fps must be positive");
    }

    impl_->frames = 0;
    impl_->dropped = 0;
    impl_->errors = 0;
    impl_->first_stamp_us = -1;
    impl_->last_stamp_us = 0;
    impl_->have_sequence = false;
    impl_->last_sequence = 0;
    impl_->have_published = false;
    impl_->next_publish = {};
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

    impl_->running = true;
    try {
        impl_->thread = std::thread([this] { impl_->capture_loop(); });
    } catch (...) {
        impl_->running = false;
        impl_->close_device();
        throw;
    }
}

void MonoJpegSource::stop()
{
    impl_->running = false;
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
    impl_->close_device();
}

bool MonoJpegSource::running() const
{
    return impl_->running;
}

std::optional<MonoJpegFrame> MonoJpegSource::latest() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->latest;
}

MonoJpegStats MonoJpegSource::stats() const
{
    MonoJpegStats result;
    result.frames = impl_->frames;
    result.dropped = impl_->dropped;
    result.errors = impl_->errors;
    result.width = impl_->actual_width;
    result.height = impl_->actual_height;
    result.running = impl_->running;
    const std::int64_t first = impl_->first_stamp_us;
    const std::int64_t last = impl_->last_stamp_us;
    if (first >= 0 && last > first && result.frames > 1) {
        result.fps = static_cast<double>(result.frames - 1) * 1000000.0 /
            static_cast<double>(last - first);
    }
    return result;
}

void MonoJpegSource::set_callback(std::function<void(const MonoJpegFrame&)> callback)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->callback = std::move(callback);
}

} // namespace flyrat::hardware
