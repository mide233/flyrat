// SPDX-License-Identifier: MIT
#pragma once

#include <string>

#include "flyrat/hardware/transport.hpp"

namespace flyrat::hardware {

// 8N1 raw serial port. Supports the baud rates used by Betaflight MSP
// (9600 .. 921600 plus 1M/1.5M/2M where the platform defines them).
class SerialTransport final : public Transport {
public:
    SerialTransport(std::string device, int baud);
    ~SerialTransport() override;

    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    bool open(std::string& error) override;
    void close() noexcept override;
    bool is_open() const noexcept override { return fd_ >= 0; }
    int native_handle() const noexcept override { return fd_; }
    std::ptrdiff_t read_some(std::uint8_t* data, std::size_t size, std::string& error) override;
    bool write_all(const std::uint8_t* data, std::size_t size, int timeout_ms,
        std::string& error) override;

private:
    std::string device_;
    int baud_;
    int fd_ = -1;
};

} // namespace flyrat::hardware
