// SPDX-License-Identifier: MIT
#include "flyrat/hardware/tcp_transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace flyrat::hardware {
namespace {

bool wait_for(int fd, short events, int timeout_ms, std::string& error)
{
    struct pollfd pfd {};
    pfd.fd = fd;
    pfd.events = events;
    for (;;) {
        const int rc = ::poll(&pfd, 1, timeout_ms);
        if (rc < 0 && errno == EINTR) {
            continue;
        }
        if (rc < 0) {
            error = std::string("poll: ") + std::strerror(errno);
            return false;
        }
        if (rc == 0) {
            error = "poll: timeout";
            return false;
        }
        return true;
    }
}

} // namespace

TcpTransport::TcpTransport(std::string host, int port, int connect_timeout_ms)
    : host_(std::move(host))
    , port_(port)
    , connect_timeout_ms_(connect_timeout_ms)
{
}

TcpTransport::~TcpTransport() { close(); }

bool TcpTransport::open(std::string& error)
{
    close();

    struct addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    struct addrinfo* result = nullptr;
    const std::string service = std::to_string(port_);
    const int gai = ::getaddrinfo(host_.c_str(), service.c_str(), &hints, &result);
    if (gai != 0) {
        error = "getaddrinfo " + host_ + ": " + ::gai_strerror(gai);
        return false;
    }

    std::string last_error = "no address";
    for (struct addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
        const int fd = ::socket(ai->ai_family, ai->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC,
            ai->ai_protocol);
        if (fd < 0) {
            last_error = std::string("socket: ") + std::strerror(errno);
            continue;
        }

        const int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        int rc = ::connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc != 0 && errno != EINPROGRESS) {
            last_error = std::string("connect: ") + std::strerror(errno);
            ::close(fd);
            continue;
        }

        if (rc != 0) {
            std::string poll_error;
            if (!wait_for(fd, POLLOUT, connect_timeout_ms_, poll_error)) {
                last_error = "connect " + host_ + ": " + poll_error;
                ::close(fd);
                continue;
            }
            int so_error = 0;
            socklen_t len = sizeof(so_error);
            if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) != 0) {
                last_error = std::string("getsockopt: ") + std::strerror(errno);
                ::close(fd);
                continue;
            }
            if (so_error != 0) {
                last_error = std::string("connect: ") + std::strerror(so_error);
                ::close(fd);
                continue;
            }
        }

        ::freeaddrinfo(result);
        fd_ = fd;
        return true;
    }

    ::freeaddrinfo(result);
    error = "connect " + host_ + ":" + service + ": " + last_error;
    return false;
}

void TcpTransport::close() noexcept
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

std::ptrdiff_t TcpTransport::read_some(
    std::uint8_t* data, std::size_t size, std::string& error)
{
    if (fd_ < 0) {
        error = "tcp socket is not open";
        return -1;
    }
    for (;;) {
        const ssize_t n = ::recv(fd_, data, size, 0);
        if (n > 0) {
            return n;
        }
        if (n == 0) {
            error = "tcp peer closed the connection";
            return -1;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        error = std::string("recv: ") + std::strerror(errno);
        return -1;
    }
}

bool TcpTransport::write_all(
    const std::uint8_t* data, std::size_t size, int timeout_ms, std::string& error)
{
    if (fd_ < 0) {
        error = "tcp socket is not open";
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::size_t written = 0;
    while (written < size) {
        const ssize_t n = ::send(fd_, data + written, size - written, MSG_NOSIGNAL);
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
                error = "send: timeout";
                return false;
            }
            const auto remaining
                = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            std::string poll_error;
            if (!wait_for(fd_, POLLOUT, static_cast<int>(remaining), poll_error)) {
                error = "send: " + poll_error;
                return false;
            }
            continue;
        }
        error = std::string("send: ") + std::strerror(errno);
        return false;
    }
    return true;
}

std::unique_ptr<Transport> make_tcp_transport(
    std::string host, int port, int connect_timeout_ms)
{
    return std::make_unique<TcpTransport>(std::move(host), port, connect_timeout_ms);
}

} // namespace flyrat::hardware
