// SPDX-License-Identifier: MIT
#include "flyrat/msp_client.hpp"

#include "msp_codec.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>

#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

namespace flyrat {

using namespace codec;
namespace {

constexpr char kHeader0 = '$';
constexpr char kHeader1 = 'M';
constexpr char kKindReply = '>';
constexpr char kKindError = '!';
constexpr char kKindRequest = '<';

speed_t baud_constant(int baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        throw MspError("unsupported baud rate: " + std::to_string(baud));
    }
}

} // namespace

MspClient::~MspClient() { close(); }

void MspClient::connect(const std::string& host, int port, double timeout_s) {
    close();
    timeout_ = timeout_s;

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    const std::string port_str = std::to_string(port);
    if (::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0) {
        throw MspError("getaddrinfo failed for " + host);
    }

    int fd = -1;
    for (addrinfo* rp = res; rp != nullptr; rp = rp->ai_next) {
        fd = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0)
            continue;
        if (::connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
            break;
        ::close(fd);
        fd = -1;
    }
    ::freeaddrinfo(res);

    if (fd < 0)
        throw MspError("cannot connect to " + host + ":" + port_str);
    // MSP is a request/response protocol with tiny frames. With Nagle enabled
    // every request stalls for the delayed-ACK timer (~40 ms on Linux), which
    // is what kept the telemetry loop at half its configured rate.
    const int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    fd_ = fd;
    is_socket_ = true;
    buf_.clear();
}

void MspClient::connect_serial(const std::string& device, int baud, double timeout_s) {
    close();
    timeout_ = timeout_s;

    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0)
        throw MspError("cannot open " + device + ": " + std::strerror(errno));

    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) {
        const std::string err = std::strerror(errno);
        ::close(fd);
        throw MspError("tcgetattr " + device + ": " + err);
    }
    ::cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const speed_t speed = baud_constant(baud);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        const std::string err = std::strerror(errno);
        ::close(fd);
        throw MspError("tcsetattr " + device + ": " + err);
    }
    ::tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    is_socket_ = false;
    buf_.clear();
}

void MspClient::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    is_socket_ = false;
    buf_.clear();
}

bool MspClient::fill(double timeout_s) {
    if (fd_ < 0)
        throw MspError("MSP link is not connected");
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int ms = static_cast<int>(std::max(0.0, timeout_s) * 1000.0);
    if (ms == 0 && timeout_s > 0.0)
        ms = 1;
    const int ready = ::poll(&pfd, 1, ms);
    if (ready < 0) {
        if (errno == EINTR)
            return false;
        throw MspError(std::string("poll: ") + std::strerror(errno));
    }
    if (ready == 0)
        return false;

    uint8_t tmp[4096];
    const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            return false;
        throw MspError(std::string("read: ") + std::strerror(errno));
    }
    if (n == 0) {
        if (is_socket_)
            throw MspError("MSP connection closed by peer");
        return false;
    }
    buf_.insert(buf_.end(), tmp, tmp + n);
    return true;
}

void MspClient::flush() {
    buf_.clear();
    while (fill(0.0))
        buf_.clear();
}

std::optional<std::pair<uint8_t, std::vector<uint8_t>>> MspClient::recv_frame(double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout_s));

    while (true) {
        // Resynchronise on a frame boundary, dropping any leading garbage.
        while (!buf_.empty() && buf_[0] != kHeader0)
            buf_.erase(buf_.begin());
        if (buf_.size() >= 2 && buf_[1] != kHeader1) {
            buf_.erase(buf_.begin());
            continue;
        }
        if (buf_.size() >= 3) {
            const char kind = static_cast<char>(buf_[2]);
            if (kind != kKindReply && kind != kKindError) {
                buf_.erase(buf_.begin(), buf_.begin() + 2);
                continue;
            }
            if (buf_.size() >= 4) {
                const uint8_t size = buf_[3];
                const size_t total = 5 + size + 1;
                if (buf_.size() >= total) {
                    uint8_t cs = 0;
                    const size_t end = 5u + size;
                    for (size_t k = 3; k < end; ++k)
                        cs ^= buf_[k];
                    const uint8_t got = buf_[5 + size];
                    const uint8_t cmd = buf_[4];
                    std::vector<uint8_t> payload(buf_.begin() + 5, buf_.begin() + 5 + size);
                    buf_.erase(buf_.begin(), buf_.begin() + total);
                    if (cs != got)
                        continue; // corrupt frame, resync
                    if (kind == kKindError)
                        throw MspError("MSP error frame");
                    return std::make_pair(cmd, std::move(payload));
                }
            }
        }

        if (timeout_s <= 0.0)
            return std::nullopt;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            return std::nullopt;
        const double remaining = std::chrono::duration<double>(deadline - now).count();
        if (!fill(remaining))
            return std::nullopt;
    }
}

void MspClient::send_frame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    if (fd_ < 0)
        throw MspError("MSP link is not connected");
    std::lock_guard<std::mutex> lock(send_mutex_);
    std::vector<uint8_t> frame;
    frame.reserve(6 + payload.size());
    // MSP v1: the size byte counts the payload only (not the command byte).
    const uint8_t size = static_cast<uint8_t>(payload.size());
    frame.push_back(kHeader0);
    frame.push_back(kHeader1);
    frame.push_back(kKindRequest);
    frame.push_back(size);
    frame.push_back(cmd);
    uint8_t cs = static_cast<uint8_t>(size ^ cmd);
    for (uint8_t b : payload) {
        frame.push_back(b);
        cs ^= b;
    }
    frame.push_back(cs);

    size_t off = 0;
    while (off < frame.size()) {
        const size_t remaining = frame.size() - off;
        const ssize_t n = is_socket_
                            ? ::send(fd_, frame.data() + off, remaining, MSG_NOSIGNAL)
                            : ::write(fd_, frame.data() + off, remaining);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            throw MspError(std::string("write: ") + std::strerror(errno));
        }
        if (n == 0)
            throw MspError("MSP link closed while writing");
        off += static_cast<size_t>(n);
    }
}

std::vector<uint8_t>
    MspClient::request(uint8_t cmd, const std::vector<uint8_t>& payload, int expect, int attempts) {
    const int want = expect >= 0 ? expect : static_cast<int>(cmd);
    // The flight controller occasionally drops or delays a reply while it is
    // busy with the RC stream, so resend a couple of times before giving up.
    for (int resend = 0; resend < 3; ++resend) {
        flush();
        send_frame(cmd, payload);
        for (int i = 0; i < attempts; ++i) {
            auto frame = recv_frame(timeout_);
            if (!frame)
                break;
            if (frame->first == want)
                return std::move(frame->second);
        }
    }
    throw MspError("MSP timeout waiting for command " + std::to_string(want));
}

void MspClient::set_raw_rc(const std::vector<uint16_t>& channels, double drain_timeout) {
    std::vector<uint8_t> payload;
    payload.reserve(channels.size() * 2);
    for (uint16_t ch : channels) {
        payload.push_back(static_cast<uint8_t>(ch & 0xFF));
        payload.push_back(static_cast<uint8_t>(ch >> 8));
    }
    send_frame(kSetRawRc, payload);

    // Waiting for the ack here would stall the RC stream (each recv can block for
    // the whole timeout), so it is only done when explicitly requested. Stale
    // acks are flushed by the next request()/flush().
    if (drain_timeout <= 0.0)
        return;
    try {
        for (int i = 0; i < 8; ++i) {
            if (!recv_frame(drain_timeout))
                break;
        }
    } catch (const MspError&) {
        // Protocol noise while draining is not fatal.
    }
}

} // namespace flyrat
