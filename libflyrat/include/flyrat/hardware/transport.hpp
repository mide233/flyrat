// SPDX-License-Identifier: MIT
//
// Byte-stream transport used by the MSP protocol layer.
//
// The interface is deliberately tiny and Linux-oriented: the MSP I/O thread
// polls `native_handle()` together with an internal eventfd, then drains with
// `read_some()` and writes with `write_all()`. Keeping open/close here (rather
// than in msp_interact) lets the same object be reopened after a link failure,
// so the protocol layer never has to know whether the peer is a serial port or
// a TCP socket.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace flyrat::hardware {

class Transport {
public:
    virtual ~Transport() = default;

    // Open the underlying device/socket. May be called again after close() to
    // reconnect. On failure returns false and fills `error`.
    virtual bool open(std::string& error) = 0;

    virtual void close() noexcept = 0;
    virtual bool is_open() const noexcept = 0;

    // File descriptor suitable for poll(). Returns -1 when closed.
    virtual int native_handle() const noexcept = 0;

    // Non-blocking read. Returns the number of bytes read, 0 when no data is
    // currently available (EAGAIN), or -1 on a fatal error.
    virtual std::ptrdiff_t read_some(std::uint8_t* data, std::size_t size, std::string& error) = 0;

    // Write the whole buffer, waiting at most `timeout_ms` in total. Returns
    // false on timeout or a fatal error.
    virtual bool write_all(const std::uint8_t* data, std::size_t size, int timeout_ms,
        std::string& error) = 0;
};

std::unique_ptr<Transport> make_serial_transport(std::string device, int baud);
std::unique_ptr<Transport> make_tcp_transport(
    std::string host, int port, int connect_timeout_ms = 1000);

} // namespace flyrat::hardware
