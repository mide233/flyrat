// SPDX-License-Identifier: MIT
//
// Virtual RC receiver for the SITL's RX_UDP input.
//
// The SITL listens on UDP 9004 for
//   struct { double timestamp; uint16_t channels[16]; }
// with channels in raw receiver order (roll, pitch, throttle, yaw, aux...).
// This is the "transmitter": it carries the ARM and MSP OVERRIDE switches, so
// they are never driven over the companion link.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <netinet/in.h>

namespace flyrat_msp {

class RcReceiver {
 public:
  RcReceiver() = default;
  ~RcReceiver();

  RcReceiver(const RcReceiver&) = delete;
  RcReceiver& operator=(const RcReceiver&) = delete;

  void open(const std::string& host, int port);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // `channels` is raw receiver order with up to 16 entries; missing channels
  // default to the idle value.
  void send(const std::vector<uint16_t>& channels, double timestamp,
            uint16_t default_channel = 1000);

 private:
  int fd_ = -1;
  sockaddr_in addr_{};
};

}  // namespace flyrat_msp
