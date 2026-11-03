// SPDX-License-Identifier: MIT
#include "msp_frame.hpp"

#include <algorithm>
#include <limits>

namespace flyrat::detail {
namespace {

bool is_direction(std::uint8_t byte)
{
    return byte == '<' || byte == '>' || byte == '!';
}

} // namespace

void FrameParser::append(const std::uint8_t* data, std::size_t size)
{
    buffer_.insert(buffer_.end(), data, data + size);
}

void FrameParser::reset()
{
    buffer_.clear();
    checksum_errors_ = 0;
}

bool FrameParser::next(Frame& frame)
{
    while (buffer_.size() >= 3) {
        std::size_t start = std::numeric_limits<std::size_t>::max();
        for (std::size_t i = 0; i + 2 < buffer_.size(); ++i) {
            if (buffer_[i] == '$' && buffer_[i + 1] == 'M' && is_direction(buffer_[i + 2])) {
                start = i;
                break;
            }
        }

        if (start == std::numeric_limits<std::size_t>::max()) {
            // Keep the last two bytes: they may be the beginning of `$M`.
            if (buffer_.size() > 2) {
                buffer_.erase(buffer_.begin(), buffer_.end() - 2);
            }
            return false;
        }
        if (start > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start));
        }
        if (buffer_.size() < 4) {
            return false;
        }

        const std::uint8_t size = buffer_[3];
        const std::size_t total = 6U + size;
        if (buffer_.size() < total) {
            return false;
        }

        std::uint8_t checksum = 0;
        for (std::size_t i = 3; i < 5U + size; ++i) {
            checksum ^= buffer_[i];
        }
        if (checksum != buffer_[5U + size]) {
            ++checksum_errors_;
            buffer_.erase(buffer_.begin());
            continue;
        }

        frame.direction = static_cast<char>(buffer_[2]);
        frame.cmd = buffer_[4];
        frame.payload.assign(buffer_.begin() + 5,
            buffer_.begin() + 5 + static_cast<std::ptrdiff_t>(size));
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
        return true;
    }
    return false;
}

std::vector<std::uint8_t> encode_request(
    MspCommand cmd, const std::uint8_t* payload, std::size_t size)
{
    if (size > 255U) {
        throw MspError("MSP payload exceeds 255 bytes");
    }

    std::vector<std::uint8_t> frame;
    frame.reserve(6U + size);
    frame.push_back('$');
    frame.push_back('M');
    frame.push_back('<');
    frame.push_back(static_cast<std::uint8_t>(size));
    frame.push_back(static_cast<std::uint8_t>(cmd));

    std::uint8_t checksum = static_cast<std::uint8_t>(size);
    checksum ^= static_cast<std::uint8_t>(cmd);
    for (std::size_t i = 0; i < size; ++i) {
        frame.push_back(payload[i]);
        checksum ^= payload[i];
    }
    frame.push_back(checksum);
    return frame;
}

} // namespace flyrat::detail
