// SPDX-License-Identifier: MIT
#include "flyrat/msp_interact.hpp"

#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>

#include "msp_frame.hpp"
#include "msp_parse.hpp"

namespace flyrat {
namespace {

using Clock = std::chrono::steady_clock;

constexpr int kDispatchQueueLimit = 256;
constexpr int kManualQueueLimit = 32;
constexpr int kConsecutiveTimeoutLimit = 3;

// Callback coalescing keys. Non-negative keys keep only the newest event of
// their type; -1 means "never coalesce" (link state, one-shot completions).
enum EventKey {
    kLinkKey = -1,
    kAttitudeKey = 1,
    kImuKey = 2,
    kStatusKey = 3,
    kAltitudeKey = 4,
    kRcKey = 5,
    kMotorKey = 6,
    kAnalogKey = 7,
    kGpsKey = 8,
    kDebugKey = 9,
    kRawRcKey = 10,
};

int elapsed_ms(Clock::time_point from, Clock::time_point to)
{
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(to - from).count());
}

} // namespace

struct MspInteract::Impl {
    MspConfig config;
    std::unique_ptr<hardware::Transport> transport;
    int wake_fd = -1;

    std::thread io_thread;
    std::thread dispatch_thread;
    std::atomic<bool> running { false };
    std::atomic<bool> dispatch_running { false };
    std::atomic<bool> connected { false };
    bool started = false;
    bool ever_connected = false;

    detail::FrameParser parser;
    std::uint64_t parser_checksum_seen = 0;
    int consecutive_timeouts = 0;

    struct PollItem {
        MspCommand cmd;
        std::chrono::microseconds period;
        Clock::time_point due;
    };
    std::vector<PollItem> polls;

    struct Pending {
        std::uint8_t cmd;
        Clock::time_point sent_at;
        Clock::time_point deadline;
        bool is_rc;
        std::function<void(bool, const std::vector<std::uint8_t>&)> done;
    };
    std::deque<Pending> pending;

    struct ManualRequest {
        MspCommand cmd;
        std::function<void(bool, const std::vector<std::uint8_t>&)> done;
    };
    std::mutex manual_mu;
    std::deque<ManualRequest> manual;

    std::mutex rc_mu;
    std::vector<std::uint16_t> rc_frame;
    bool has_rc = false;

    mutable std::mutex state_mu;
    std::optional<Sample<Attitude>> attitude_sample;
    std::optional<Sample<ImuSample>> imu_sample;
    std::optional<Sample<StatusFrame>> status_sample;
    std::optional<Sample<AltitudeFrame>> altitude_sample;
    std::optional<Sample<RcFrame>> rc_sample;
    std::optional<Sample<RawRcFrame>> raw_rc_sample;
    std::optional<Sample<MotorFrame>> motor_sample;
    std::optional<Sample<AnalogFrame>> analog_sample;
    std::optional<Sample<GpsFrame>> gps_sample;
    std::optional<Sample<DebugFrame>> debug_sample;
    std::optional<ApiVersion> api_version_sample;

    std::mutex callback_mu;
    std::function<void(const Sample<Attitude>&)> attitude_cb;
    std::function<void(const Sample<ImuSample>&)> imu_cb;
    std::function<void(const Sample<StatusFrame>&)> status_cb;
    std::function<void(const Sample<AltitudeFrame>&)> altitude_cb;
    std::function<void(const Sample<RcFrame>&)> rc_cb;
    std::function<void(const Sample<RawRcFrame>&)> raw_rc_cb;
    std::function<void(const Sample<MotorFrame>&)> motor_cb;
    std::function<void(const Sample<AnalogFrame>&)> analog_cb;
    std::function<void(const Sample<GpsFrame>&)> gps_cb;
    std::function<void(const Sample<DebugFrame>&)> debug_cb;
    std::function<void(bool, const std::string&)> link_cb;

    mutable std::mutex stats_mu;
    LinkStats stats;
    std::optional<Clock::time_point> last_rx;

    struct Event {
        int key;
        std::function<void()> fn;
    };
    std::mutex queue_mu;
    std::condition_variable queue_cv;
    std::deque<Event> queue;

    Impl(MspConfig cfg, std::unique_ptr<hardware::Transport> t)
        : config(std::move(cfg))
        , transport(std::move(t))
    {
        wake_fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        if (wake_fd < 0) {
            throw MspError(std::string("eventfd: ") + std::strerror(errno));
        }
        if (!transport) {
            ::close(wake_fd);
            wake_fd = -1;
            throw MspError("MspInteract requires a transport");
        }
        build_polls();
    }

    ~Impl()
    {
        stop();
        if (wake_fd >= 0) {
            ::close(wake_fd);
        }
    }

    void build_polls()
    {
        auto add = [this](MspCommand cmd, int hz) {
            if (hz <= 0) {
                return;
            }
            const auto period = std::chrono::microseconds(std::max(1000, 1000000 / hz));
            polls.push_back({ cmd, period, Clock::now() });
        };
        add(MspCommand::kRawImu, config.poll.imu_hz);
        add(MspCommand::kAttitude, config.poll.attitude_hz);
        add(MspCommand::kStatus, config.poll.status_hz);
        add(MspCommand::kAltitude, config.poll.altitude_hz);
        add(MspCommand::kRc, config.poll.rc_hz);
        add(MspCommand::kRawRcOriginal, config.poll.raw_rc_hz);
        add(MspCommand::kAnalog, config.poll.analog_hz);
        add(MspCommand::kMotor, config.poll.motor_hz);
        add(MspCommand::kRawGps, config.poll.gps_hz);
        add(MspCommand::kDebug, config.poll.debug_hz);
    }

    void reset_polls()
    {
        const auto now = Clock::now();
        for (auto& poll : polls) {
            poll.due = now;
        }
    }

    void wake()
    {
        const std::uint64_t one = 1;
        const ssize_t n = ::write(wake_fd, &one, sizeof(one));
        (void)n; // EAGAIN simply means a wake-up is already pending.
    }

    void drain_wake()
    {
        std::uint64_t value = 0;
        while (::read(wake_fd, &value, sizeof(value)) > 0) {
        }
    }

    void wait_wake(int timeout_ms)
    {
        struct pollfd pfd {};
        pfd.fd = wake_fd;
        pfd.events = POLLIN;
        const int rc = ::poll(&pfd, 1, timeout_ms);
        if (rc > 0 && (pfd.revents & POLLIN)) {
            drain_wake();
        }
    }

    void set_error(const std::string& error)
    {
        std::lock_guard<std::mutex> lock(stats_mu);
        stats.last_error = error;
    }

    void notify_link(bool up, const std::string& why)
    {
        std::function<void(bool, const std::string&)> cb;
        {
            std::lock_guard<std::mutex> lock(callback_mu);
            cb = link_cb;
        }
        if (!cb) {
            return;
        }
        post(kLinkKey, [cb = std::move(cb), up, why]() { cb(up, why); });
    }

    void post(int key, std::function<void()> fn)
    {
        std::lock_guard<std::mutex> lock(queue_mu);
        if (!dispatch_running) {
            return;
        }
        if (key >= 0) {
            for (auto& event : queue) {
                if (event.key == key) {
                    event.fn = std::move(fn);
                    return;
                }
            }
        }
        if (static_cast<int>(queue.size()) >= kDispatchQueueLimit) {
            const auto it = std::find_if(queue.begin(), queue.end(),
                [](const Event& event) { return event.key >= 0; });
            if (it != queue.end()) {
                queue.erase(it);
            } else {
                queue.pop_front();
            }
        }
        queue.push_back({ key, std::move(fn) });
        queue_cv.notify_one();
    }

    void dispatch_loop()
    {
        for (;;) {
            Event event;
            {
                std::unique_lock<std::mutex> lock(queue_mu);
                queue_cv.wait(lock, [this]() { return !queue.empty() || !dispatch_running; });
                if (!dispatch_running && queue.empty()) {
                    return;
                }
                event = std::move(queue.front());
                queue.pop_front();
            }
            try {
                event.fn();
            } catch (...) {
                // A user callback must never take down the dispatch thread.
            }
        }
    }

    template <typename T>
    void store(std::optional<Sample<T>>& slot, T value, Clock::time_point stamp,
        std::function<void(const Sample<T>&)>& callback, int key)
    {
        Sample<T> sample { std::move(value), stamp };
        {
            std::lock_guard<std::mutex> lock(state_mu);
            slot = sample;
        }
        std::function<void(const Sample<T>&)> handler;
        {
            std::lock_guard<std::mutex> lock(callback_mu);
            handler = callback;
        }
        if (handler) {
            post(key, [handler = std::move(handler), sample]() { handler(sample); });
        }
    }

    template <typename T>
    std::optional<Sample<T>> snapshot(const std::optional<Sample<T>>& slot) const
    {
        std::lock_guard<std::mutex> lock(state_mu);
        return slot;
    }

    bool handshake(int timeout_ms)
    {
        detail::FrameParser handshake_parser;
        const auto request = detail::encode_request(MspCommand::kApiVersion, nullptr, 0);
        std::string error;
        if (!transport->write_all(request.data(), request.size(), config.write_timeout_ms, error)) {
            set_error("handshake write: " + error);
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            ++stats.tx_frames;
        }

        const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        std::uint8_t buffer[256];
        while (Clock::now() < deadline) {
            const int remaining = std::max(
                0, elapsed_ms(Clock::now(), deadline));
            struct pollfd pfd {};
            pfd.fd = transport->native_handle();
            pfd.events = POLLIN;
            const int rc = ::poll(&pfd, 1, remaining);
            if (rc < 0 && errno == EINTR) {
                continue;
            }
            if (rc <= 0) {
                break;
            }
            const std::ptrdiff_t n = transport->read_some(buffer, sizeof(buffer), error);
            if (n < 0) {
                set_error("handshake read: " + error);
                return false;
            }
            if (n == 0) {
                continue;
            }
            handshake_parser.append(buffer, static_cast<std::size_t>(n));
            detail::Frame frame;
            while (handshake_parser.next(frame)) {
                if (frame.direction != '>'
                    || frame.cmd != static_cast<std::uint8_t>(MspCommand::kApiVersion)) {
                    continue;
                }
                ApiVersion version;
                if (!detail::parse_api_version(frame.payload, version)) {
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lock(state_mu);
                    api_version_sample = version;
                }
                return true;
            }
        }
        set_error("MSP_ApiVersion handshake timeout");
        return false;
    }

    bool send_request(MspCommand cmd, const std::vector<std::uint8_t>& payload,
        std::function<void(bool, const std::vector<std::uint8_t>&)> done, bool is_rc,
        int timeout_ms)
    {
        std::vector<std::uint8_t> frame;
        try {
            frame = detail::encode_request(cmd, payload.data(), payload.size());
        } catch (const MspError& error) {
            set_error(error.what());
            if (done) {
                post(kLinkKey, [done = std::move(done)]() { done(false, {}); });
            }
            return false;
        }

        std::string error;
        if (!transport->write_all(frame.data(), frame.size(), config.write_timeout_ms, error)) {
            set_error("write: " + error);
            if (done) {
                post(kLinkKey, [done = std::move(done)]() { done(false, {}); });
            }
            disconnect("write: " + error);
            return false;
        }

        const auto now = Clock::now();
        pending.push_back({ static_cast<std::uint8_t>(cmd), now,
            now + std::chrono::milliseconds(timeout_ms), is_rc, std::move(done) });
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            ++stats.tx_frames;
            if (is_rc) {
                ++stats.rc_sent;
            }
        }
        return true;
    }

    void send_rc_if_pending()
    {
        if (!connected || pending.size() >= static_cast<std::size_t>(config.max_inflight)) {
            return;
        }
        std::vector<std::uint16_t> channels;
        {
            std::lock_guard<std::mutex> lock(rc_mu);
            if (!has_rc) {
                return;
            }
            channels = std::move(rc_frame);
            has_rc = false;
        }
        if (channels.size() != 16U) {
            return;
        }
        std::vector<std::uint8_t> payload(32);
        for (std::size_t i = 0; i < channels.size(); ++i) {
            payload[2U * i] = static_cast<std::uint8_t>(channels[i] & 0xFFU);
            payload[2U * i + 1U] = static_cast<std::uint8_t>(channels[i] >> 8);
        }
        send_request(MspCommand::kSetRawRc, payload, {}, true, config.poll_timeout_ms);
    }

    void process_manual()
    {
        for (;;) {
            ManualRequest request;
            {
                std::lock_guard<std::mutex> lock(manual_mu);
                if (manual.empty()
                    || pending.size() >= static_cast<std::size_t>(config.max_inflight)) {
                    return;
                }
                request = std::move(manual.front());
                manual.pop_front();
            }
            if (!connected) {
                if (request.done) {
                    post(kLinkKey, [done = std::move(request.done)]() { done(false, {}); });
                }
                continue;
            }
            if (!send_request(request.cmd, {}, std::move(request.done), false,
                    config.request_timeout_ms)) {
                return;
            }
        }
    }

    void send_due(Clock::time_point now)
    {
        send_rc_if_pending();
        if (!connected) {
            return;
        }
        for (auto& poll : polls) {
            if (poll.due > now) {
                continue;
            }
            if (pending.size() >= static_cast<std::size_t>(config.max_inflight)) {
                break;
            }
            if (!send_request(poll.cmd, {}, {}, false, config.poll_timeout_ms)) {
                return;
            }
            do {
                poll.due += std::chrono::duration_cast<Clock::duration>(poll.period);
            } while (poll.due <= now);
        }
    }

    void fail_pending(const std::vector<std::uint8_t>& payload = {})
    {
        std::deque<Pending> failed;
        failed.swap(pending);
        for (auto& request : failed) {
            if (request.done) {
                post(kLinkKey, [done = std::move(request.done), payload]() {
                    done(false, payload);
                });
            }
        }
    }

    void disconnect(const std::string& reason)
    {
        const bool was_connected = connected.exchange(false);
        transport->close();
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            stats.connected = false;
            stats.last_error = reason;
        }
        fail_pending();
        {
            std::lock_guard<std::mutex> lock(rc_mu);
            has_rc = false;
            rc_frame.clear();
        }
        {
            std::lock_guard<std::mutex> lock(manual_mu);
            std::deque<ManualRequest> dropped;
            dropped.swap(manual);
            for (auto& request : dropped) {
                if (request.done) {
                    post(kLinkKey, [done = std::move(request.done)]() { done(false, {}); });
                }
            }
        }
        if (was_connected) {
            notify_link(false, reason);
        }
    }

    bool reconnect()
    {
        transport->close();
        std::string error;
        if (!transport->open(error)) {
            set_error("reconnect open: " + error);
            return false;
        }
        if (!handshake(config.handshake_timeout_ms)) {
            transport->close();
            return false;
        }

        parser.reset();
        parser_checksum_seen = 0;
        consecutive_timeouts = 0;
        pending.clear();
        {
            std::lock_guard<std::mutex> lock(rc_mu);
            has_rc = false;
            rc_frame.clear();
        }
        reset_polls();
        connected = true;
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            stats.connected = true;
            stats.last_error.clear();
            ++stats.reconnects;
        }
        ever_connected = true;
        notify_link(true, "connected");
        return true;
    }

    void sync_parser_stats()
    {
        const std::uint64_t current = parser.checksum_errors();
        if (current <= parser_checksum_seen) {
            return;
        }
        const std::uint64_t delta = current - parser_checksum_seen;
        parser_checksum_seen = current;
        std::lock_guard<std::mutex> lock(stats_mu);
        stats.checksum_errors += delta;
    }

    bool read_available()
    {
        std::uint8_t buffer[512];
        for (;;) {
            std::string error;
            const std::ptrdiff_t n = transport->read_some(buffer, sizeof(buffer), error);
            if (n < 0) {
                disconnect("read: " + error);
                return false;
            }
            if (n == 0) {
                break;
            }
            parser.append(buffer, static_cast<std::size_t>(n));
            detail::Frame frame;
            while (parser.next(frame)) {
                handle_frame(frame);
            }
            sync_parser_stats();
        }
        return true;
    }

    void expire_pending(Clock::time_point now)
    {
        while (!pending.empty() && pending.front().deadline <= now) {
            Pending request = std::move(pending.front());
            pending.pop_front();
            if (request.is_rc) {
                std::lock_guard<std::mutex> lock(stats_mu);
                ++stats.rc_timeouts;
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(stats_mu);
                ++stats.timeouts;
            }
            if (request.done) {
                post(kLinkKey, [done = std::move(request.done)]() { done(false, {}); });
            }
            ++consecutive_timeouts;
        }
        if (consecutive_timeouts >= kConsecutiveTimeoutLimit && connected) {
            disconnect("too many MSP timeouts");
        }
    }

    void handle_frame(detail::Frame& frame)
    {
        const auto now = Clock::now();
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            ++stats.rx_frames;
            last_rx = now;
        }

        if (frame.direction == '!') {
            {
                std::lock_guard<std::mutex> lock(stats_mu);
                ++stats.malformed_frames;
            }
            complete_matching(static_cast<std::uint8_t>(frame.cmd), false, {}, now);
            return;
        }

        complete_matching(frame.cmd, true, frame.payload, now);

        const auto cmd = static_cast<MspCommand>(frame.cmd);
        switch (cmd) {
        case MspCommand::kApiVersion: {
            ApiVersion version;
            if (detail::parse_api_version(frame.payload, version)) {
                std::lock_guard<std::mutex> lock(state_mu);
                api_version_sample = version;
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kAttitude: {
            Attitude value;
            if (detail::parse_attitude(frame.payload, value)) {
                store(attitude_sample, value, now, attitude_cb, kAttitudeKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kRawImu: {
            ImuSample value;
            if (detail::parse_raw_imu(frame.payload, value)) {
                store(imu_sample, value, now, imu_cb, kImuKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kStatus: {
            StatusFrame value;
            if (detail::parse_status(frame.payload, config.status_extended, value)) {
                store(status_sample, value, now, status_cb, kStatusKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kAltitude: {
            AltitudeFrame value;
            if (detail::parse_altitude(frame.payload, value)) {
                store(altitude_sample, value, now, altitude_cb, kAltitudeKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kRc: {
            RcFrame value;
            if (detail::parse_rc(frame.payload, value)) {
                store(rc_sample, value, now, rc_cb, kRcKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kRawRcOriginal: {
            RawRcFrame value;
            if (detail::parse_raw_rc(frame.payload, value)) {
                store(raw_rc_sample, value, now, raw_rc_cb, kRawRcKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kMotor: {
            MotorFrame value;
            if (detail::parse_motor(frame.payload, value)) {
                store(motor_sample, value, now, motor_cb, kMotorKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kAnalog: {
            AnalogFrame value;
            if (detail::parse_analog(frame.payload, value)) {
                store(analog_sample, value, now, analog_cb, kAnalogKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kRawGps: {
            GpsFrame value;
            if (detail::parse_raw_gps(frame.payload, value)) {
                store(gps_sample, value, now, gps_cb, kGpsKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kDebug: {
            DebugFrame value;
            if (detail::parse_debug(frame.payload, value)) {
                store(debug_sample, value, now, debug_cb, kDebugKey);
            } else {
                mark_malformed();
            }
            break;
        }
        case MspCommand::kSetRawRc:
            break; // ACK only; already counted in complete_matching.
        default:
            break;
        }
    }

    void mark_malformed()
    {
        std::lock_guard<std::mutex> lock(stats_mu);
        ++stats.malformed_frames;
    }

    void complete_matching(std::uint8_t cmd, bool ok,
        const std::vector<std::uint8_t>& payload, Clock::time_point now)
    {
        const auto it = std::find_if(pending.begin(), pending.end(),
            [cmd](const Pending& request) { return request.cmd == cmd; });
        if (it == pending.end()) {
            return;
        }

        for (auto before = pending.begin(); before != it; ++before) {
            if (before->is_rc) {
                std::lock_guard<std::mutex> lock(stats_mu);
                ++stats.rc_timeouts;
            } else {
                {
                    std::lock_guard<std::mutex> lock(stats_mu);
                    ++stats.timeouts;
                }
                ++consecutive_timeouts;
                if (before->done) {
                    post(kLinkKey, [done = std::move(before->done)]() { done(false, {}); });
                }
            }
        }

        Pending request = std::move(*it);
        pending.erase(pending.begin(), it + 1);

        if (request.is_rc) {
            if (ok) {
                std::lock_guard<std::mutex> lock(stats_mu);
                ++stats.rc_ack;
                stats.last_rc_ack_ms = elapsed_ms(request.sent_at, now);
            }
            return;
        }

        if (ok) {
            consecutive_timeouts = 0;
            if (request.cmd == static_cast<std::uint8_t>(MspCommand::kRawImu)) {
                std::lock_guard<std::mutex> lock(stats_mu);
                stats.last_imu_latency_ms = elapsed_ms(request.sent_at, now);
            }
        }
        if (request.done) {
            post(kLinkKey, [done = std::move(request.done), ok, payload]() {
                done(ok, payload);
            });
        }
    }

    int compute_timeout_ms()
    {
        const auto now = Clock::now();
        auto next = now + std::chrono::seconds(1);
        for (const auto& poll : polls) {
            if (poll.due < next) {
                next = poll.due;
            }
        }
        for (const auto& request : pending) {
            if (request.deadline < next) {
                next = request.deadline;
            }
        }
        {
            std::lock_guard<std::mutex> lock(rc_mu);
            if (has_rc) {
                return 0;
            }
        }
        {
            std::lock_guard<std::mutex> lock(manual_mu);
            if (!manual.empty()) {
                return 0;
            }
        }
        return std::clamp(elapsed_ms(now, next), 0, 1000);
    }

    void io_loop()
    {
        while (running) {
            if (!connected) {
                if (!reconnect()) {
                    wait_wake(config.reconnect_backoff_ms);
                    continue;
                }
            }

            const int timeout = compute_timeout_ms();
            struct pollfd fds[2] {};
            fds[0].fd = transport->native_handle();
            fds[0].events = POLLIN;
            fds[1].fd = wake_fd;
            fds[1].events = POLLIN;

            const int rc = ::poll(fds, 2, timeout);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                disconnect(std::string("poll: ") + std::strerror(errno));
                continue;
            }
            if (rc == 0) {
                const auto now = Clock::now();
                expire_pending(now);
                process_manual();
                send_due(now);
                continue;
            }
            if ((fds[1].revents & POLLIN) != 0) {
                drain_wake();
            }
            if ((fds[0].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
                if (!read_available()) {
                    continue;
                }
            }

            const auto now = Clock::now();
            expire_pending(now);
            if (!connected) {
                continue;
            }
            process_manual();
            send_due(now);
        }
    }

    void start()
    {
        if (started) {
            if (running) {
                return;
            }
            throw MspError("MspInteract cannot be restarted after stop()");
        }
        if (!transport) {
            throw MspError("MspInteract requires a transport");
        }

        std::string error;
        if (!transport->open(error)) {
            throw MspError("open transport: " + error);
        }
        if (!handshake(config.handshake_timeout_ms)) {
            transport->close();
            throw MspError("MSP handshake failed");
        }

        started = true;
        ever_connected = true;
        connected = true;
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            stats.connected = true;
            stats.last_error.clear();
        }

        dispatch_running = true;
        running = true;
        dispatch_thread = std::thread(&Impl::dispatch_loop, this);
        io_thread = std::thread(&Impl::io_loop, this);
        notify_link(true, "connected");
    }

    void stop()
    {
        if (!started) {
            transport->close();
            connected = false;
            return;
        }
        running = false;
        wake();
        if (io_thread.joinable()) {
            io_thread.join();
        }
        {
            std::lock_guard<std::mutex> lock(queue_mu);
            dispatch_running = false;
            queue.clear();
        }
        queue_cv.notify_all();
        if (dispatch_thread.joinable()) {
            dispatch_thread.join();
        }
        transport->close();
        connected = false;
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            stats.connected = false;
        }
        started = false;
    }

    LinkStats get_stats() const
    {
        LinkStats out;
        {
            std::lock_guard<std::mutex> lock(stats_mu);
            out = stats;
            if (last_rx) {
                out.last_rx_age_ms = elapsed_ms(*last_rx, Clock::now());
            }
        }
        out.connected = connected;
        return out;
    }

    void request(MspCommand cmd,
        std::function<void(bool, const std::vector<std::uint8_t>&)> done)
    {
        if (!connected) {
            if (done) {
                post(kLinkKey, [done = std::move(done)]() { done(false, {}); });
            }
            return;
        }
        std::lock_guard<std::mutex> lock(manual_mu);
        if (static_cast<int>(manual.size()) >= kManualQueueLimit) {
            ManualRequest dropped = std::move(manual.front());
            manual.pop_front();
            if (dropped.done) {
                post(kLinkKey, [done = std::move(dropped.done)]() { done(false, {}); });
            }
        }
        manual.push_back({ cmd, std::move(done) });
        wake();
    }
};

MspInteract::MspInteract(MspConfig config, std::unique_ptr<hardware::Transport> transport)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(transport)))
{
}

MspInteract::~MspInteract() = default;

void MspInteract::start() { impl_->start(); }
void MspInteract::stop() { impl_->stop(); }

void MspInteract::set_attitude_callback(std::function<void(const Sample<Attitude>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->attitude_cb = std::move(cb);
}

void MspInteract::set_imu_callback(std::function<void(const Sample<ImuSample>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->imu_cb = std::move(cb);
}

void MspInteract::set_status_callback(std::function<void(const Sample<StatusFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->status_cb = std::move(cb);
}

void MspInteract::set_altitude_callback(std::function<void(const Sample<AltitudeFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->altitude_cb = std::move(cb);
}

void MspInteract::set_rc_callback(std::function<void(const Sample<RcFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->rc_cb = std::move(cb);
}

void MspInteract::set_raw_rc_callback(std::function<void(const Sample<RawRcFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->raw_rc_cb = std::move(cb);
}

void MspInteract::set_motor_callback(std::function<void(const Sample<MotorFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->motor_cb = std::move(cb);
}

void MspInteract::set_analog_callback(std::function<void(const Sample<AnalogFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->analog_cb = std::move(cb);
}

void MspInteract::set_gps_callback(std::function<void(const Sample<GpsFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->gps_cb = std::move(cb);
}

void MspInteract::set_debug_callback(std::function<void(const Sample<DebugFrame>&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->debug_cb = std::move(cb);
}

void MspInteract::set_link_callback(std::function<void(bool, const std::string&)> cb)
{
    std::lock_guard<std::mutex> lock(impl_->callback_mu);
    impl_->link_cb = std::move(cb);
}

std::optional<Sample<Attitude>> MspInteract::attitude() const
{
    return impl_->snapshot(impl_->attitude_sample);
}

std::optional<Sample<ImuSample>> MspInteract::imu() const
{
    return impl_->snapshot(impl_->imu_sample);
}

std::optional<Sample<StatusFrame>> MspInteract::status() const
{
    return impl_->snapshot(impl_->status_sample);
}

std::optional<Sample<AltitudeFrame>> MspInteract::altitude() const
{
    return impl_->snapshot(impl_->altitude_sample);
}

std::optional<Sample<RcFrame>> MspInteract::rc() const
{
    return impl_->snapshot(impl_->rc_sample);
}

std::optional<Sample<RawRcFrame>> MspInteract::rawRc() const
{
    return impl_->snapshot(impl_->raw_rc_sample);
}

std::optional<Sample<MotorFrame>> MspInteract::motor() const
{
    return impl_->snapshot(impl_->motor_sample);
}

std::optional<Sample<AnalogFrame>> MspInteract::analog() const
{
    return impl_->snapshot(impl_->analog_sample);
}

std::optional<Sample<GpsFrame>> MspInteract::gps() const
{
    return impl_->snapshot(impl_->gps_sample);
}

std::optional<Sample<DebugFrame>> MspInteract::debug() const
{
    return impl_->snapshot(impl_->debug_sample);
}

std::optional<ApiVersion> MspInteract::api_version() const
{
    std::lock_guard<std::mutex> lock(impl_->state_mu);
    return impl_->api_version_sample;
}

bool MspInteract::send_rc(const std::vector<std::uint16_t>& channels)
{
    if (!impl_->connected || channels.size() != 16U) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(impl_->rc_mu);
        impl_->rc_frame = channels;
        impl_->has_rc = true;
    }
    impl_->wake();
    return true;
}

void MspInteract::request(MspCommand cmd,
    std::function<void(bool, const std::vector<std::uint8_t>& payload)> done)
{
    impl_->request(cmd, std::move(done));
}

LinkStats MspInteract::stats() const { return impl_->get_stats(); }

bool MspInteract::connected() const { return impl_->connected; }

} // namespace flyrat
