// SPDX-License-Identifier: MIT
//
// Little-endian field readers shared by the MSP transport and the
// telemetry parsers.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "flyrat/msp_client.hpp"

namespace flyrat::codec {

inline uint16_t read_u16(const std::vector<uint8_t>& p, size_t o, const char* what) {
    if (o + 2 > p.size())
        throw MspError(std::string("short ") + what);
    return static_cast<uint16_t>(p[o]) | static_cast<uint16_t>(p[o + 1]) << 8;
}

inline int16_t read_i16(const std::vector<uint8_t>& p, size_t o, const char* what) {
    return static_cast<int16_t>(read_u16(p, o, what));
}

inline uint32_t read_u32(const std::vector<uint8_t>& p, size_t o, const char* what) {
    if (o + 4 > p.size())
        throw MspError(std::string("short ") + what);
    return static_cast<uint32_t>(p[o]) | static_cast<uint32_t>(p[o + 1]) << 8
         | static_cast<uint32_t>(p[o + 2]) << 16 | static_cast<uint32_t>(p[o + 3]) << 24;
}

// MSP_STATUS/STATUS_EX carry the first 32 flight-mode bits inline; boxes above
// 31 (e.g. MSP OVERRIDE = 50) live in an extension block whose length is a
// byte at offset 15 and whose bytes start at offset 16.
inline bool box_active(const std::vector<uint8_t>& p, uint16_t box) {
    if (box < 32) {
        return (read_u32(p, 6, "MSP_STATUS") >> box) & 1u;
    }
    const size_t index = box - 32;
    const size_t byte = index / 8;
    const size_t offset = 16 + byte;
    if (offset >= p.size())
        return false;
    return (p[offset] >> (index % 8)) & 1u;
}

} // namespace flyrat::codec
