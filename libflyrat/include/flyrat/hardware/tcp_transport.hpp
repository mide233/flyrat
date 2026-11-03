// SPDX-License-Identifier: MIT
#pragma once

#include <string>

#include "flyrat/hardware/transport.hpp"

namespace flyrat::hardware {

// Non-blocking TCP transport. Used for the SITL MSP socket and for tests; the
// real vehicle uses SerialTransport.
class TcpTransport final : public Transport {
public:
    TcpTransport(std::string host, int port, int connect_timeout_ms = 1000);
    ~TcpTransport() override;

    TcpTransport(const TcpTransport&) = delete;
    TcpTransport& operator=(const TcpTransport&) = delete;

    bool open(std::string& error) override;
    void close() noexcept override;
    bool is_open() const noexcept override { return fd_ >= 0; }
    int native_handle() const noexcept override { return fd_; }
    std::ptrdiff_t read_some(std::uint8_t* data, std::size_t size, std::string& error) override;
    bool write_all(const std::uint8_t* data, std::size_t size, int timeout_ms,
        std::string& error) override;

private:
    std::string host_;
    int port_;
    int connect_timeout_ms_;
    int fd_ = -1;
};

} // namespace flyrat::hardware
