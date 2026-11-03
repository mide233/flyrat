// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "flyrat/msp/types.hpp"

namespace flyrat::detail {

struct Frame {
    char direction = '>';
    std::uint8_t cmd = 0;
    std::vector<std::uint8_t> payload;
};

// Incremental MSP v1 frame parser. It resynchronises on `$M<`, `$M>` and
// `$M!`, validates the XOR checksum, and drops malformed bytes one at a time.
class FrameParser {
public:
    void append(const std::uint8_t* data, std::size_t size);
    bool next(Frame& frame);
    void reset();

    std::uint64_t checksum_errors() const { return checksum_errors_; }

private:
    std::vector<std::uint8_t> buffer_;
    std::uint64_t checksum_errors_ = 0;
};

// Encode an MSP v1 request. Throws MspError if the payload exceeds 255 bytes.
std::vector<std::uint8_t> encode_request(
    MspCommand cmd, const std::uint8_t* payload, std::size_t size);

} // namespace flyrat::detail
