// SPDX-License-Identifier: MIT
#include "flyrat/rc_receiver.hpp"

#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace flyrat {

RcReceiver::~RcReceiver() { close(); }

void RcReceiver::open(const std::string& host, int port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
        throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("invalid receiver address: " + host);
    }
}

void RcReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void RcReceiver::send(
    const std::vector<uint16_t>& channels, double timestamp, uint16_t default_channel) {
    if (fd_ < 0)
        throw std::runtime_error("RC receiver is not open");
    uint8_t buf[8 + 32];
    std::memcpy(buf, &timestamp, sizeof(double));
    for (size_t i = 0; i < 16; ++i) {
        const uint16_t ch = i < channels.size() ? channels[i] : default_channel;
        buf[8 + 2 * i] = static_cast<uint8_t>(ch & 0xFF);
        buf[8 + 2 * i + 1] = static_cast<uint8_t>(ch >> 8);
    }
    const ssize_t n = ::sendto(
        fd_, buf, sizeof(buf), 0, reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_));
    if (n < 0)
        throw std::runtime_error(std::string("sendto: ") + std::strerror(errno));
}

} // namespace flyrat
