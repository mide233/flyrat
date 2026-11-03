// SPDX-License-Identifier: MIT
#include "flyrat/hardware/serial_transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace flyrat::hardware {
namespace {

bool baud_to_speed(int baud, speed_t& speed, std::string& error)
{
    switch (baud) {
    case 9600:
        speed = B9600;
        return true;
    case 19200:
        speed = B19200;
        return true;
    case 38400:
        speed = B38400;
        return true;
    case 57600:
        speed = B57600;
        return true;
    case 115200:
        speed = B115200;
        return true;
    case 230400:
        speed = B230400;
        return true;
#ifdef B460800
    case 460800:
        speed = B460800;
        return true;
#endif
#ifdef B921600
    case 921600:
        speed = B921600;
        return true;
#endif
#ifdef B1000000
    case 1000000:
        speed = B1000000;
        return true;
#endif
#ifdef B1500000
    case 1500000:
        speed = B1500000;
        return true;
#endif
#ifdef B2000000
    case 2000000:
        speed = B2000000;
        return true;
#endif
    default:
        error = "unsupported serial baud rate: " + std::to_string(baud);
        return false;
    }
}

} // namespace

SerialTransport::SerialTransport(std::string device, int baud)
    : device_(std::move(device))
    , baud_(baud)
{
}

SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::open(std::string& error)
{
    close();

    speed_t speed = B0;
    if (!baud_to_speed(baud_, speed, error)) {
        return false;
    }

    const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        error = "open " + device_ + ": " + std::strerror(errno);
        return false;
    }

    struct termios tio {};
    if (::tcgetattr(fd, &tio) != 0) {
        error = "tcgetattr " + device_ + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }

    ::cfmakeraw(&tio);
    tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tio.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
    tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
        error = "cfsetispeed/cfsetospeed " + device_ + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        error = "tcsetattr " + device_ + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }

    ::tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    return true;
}

void SerialTransport::close() noexcept
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

std::ptrdiff_t SerialTransport::read_some(
    std::uint8_t* data, std::size_t size, std::string& error)
{
    if (fd_ < 0) {
        error = "serial port is not open";
        return -1;
    }
    for (;;) {
        const ssize_t n = ::read(fd_, data, size);
        if (n > 0) {
            return n;
        }
        if (n == 0) {
            return 0;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        error = "read " + device_ + ": " + std::strerror(errno);
        return -1;
    }
}

bool SerialTransport::write_all(
    const std::uint8_t* data, std::size_t size, int timeout_ms, std::string& error)
{
    if (fd_ < 0) {
        error = "serial port is not open";
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::size_t written = 0;
    while (written < size) {
        const ssize_t n = ::write(fd_, data + written, size - written);
        if (n > 0) {
            written += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                error = "write " + device_ + ": timeout";
                return false;
            }
            const auto remaining
                = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            struct pollfd pfd {};
            pfd.fd = fd_;
            pfd.events = POLLOUT;
            const int rc = ::poll(&pfd, 1, static_cast<int>(remaining));
            if (rc < 0 && errno == EINTR) {
                continue;
            }
            if (rc <= 0) {
                error = "write " + device_ + ": timeout";
                return false;
            }
            continue;
        }
        error = "write " + device_ + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

std::unique_ptr<Transport> make_serial_transport(std::string device, int baud)
{
    return std::make_unique<SerialTransport>(std::move(device), baud);
}

} // namespace flyrat::hardware
