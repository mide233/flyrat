// SPDX-License-Identifier: MIT
#include "flyrat_msp/msp_client.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>

#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace flyrat_msp {
namespace {

constexpr char kHeader0 = '$';
constexpr char kHeader1 = 'M';
constexpr char kKindReply = '>';
constexpr char kKindError = '!';
constexpr char kKindRequest = '<';

uint16_t read_u16(const std::vector<uint8_t>& p, size_t o,
                  const char* what) {
  if (o + 2 > p.size()) throw MspError(std::string("short ") + what);
  return static_cast<uint16_t>(p[o]) |
         static_cast<uint16_t>(p[o + 1]) << 8;
}

int16_t read_i16(const std::vector<uint8_t>& p, size_t o, const char* what) {
  return static_cast<int16_t>(read_u16(p, o, what));
}

uint32_t read_u32(const std::vector<uint8_t>& p, size_t o,
                  const char* what) {
  if (o + 4 > p.size()) throw MspError(std::string("short ") + what);
  return static_cast<uint32_t>(p[o]) |
         static_cast<uint32_t>(p[o + 1]) << 8 |
         static_cast<uint32_t>(p[o + 2]) << 16 |
         static_cast<uint32_t>(p[o + 3]) << 24;
}

// MSP_STATUS/STATUS_EX carry the first 32 flight-mode bits inline; boxes above
// 31 (e.g. MSP OVERRIDE = 50) live in an extension block whose length is a
// byte at offset 15 and whose bytes start at offset 16.
bool box_active(const std::vector<uint8_t>& p, uint16_t box) {
  if (box < 32) {
    return (read_u32(p, 6, "MSP_STATUS") >> box) & 1u;
  }
  const size_t index = box - 32;
  const size_t byte = index / 8;
  const size_t offset = 16 + byte;
  if (offset >= p.size()) return false;
  return (p[offset] >> (index % 8)) & 1u;
}

}  // namespace

MspClient::~MspClient() { close(); }

void MspClient::connect(const std::string& host, int port, double timeout_s) {
  close();
  timeout_ = timeout_s;

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  const std::string port_str = std::to_string(port);
  if (::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0) {
    throw MspError("getaddrinfo failed for " + host);
  }

  int fd = -1;
  for (addrinfo* rp = res; rp != nullptr; rp = rp->ai_next) {
    fd = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (fd < 0) continue;
    if (::connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) break;
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(res);

  if (fd < 0) throw MspError("cannot connect to " + host + ":" + port_str);
  fd_ = fd;
  buf_.clear();
}

void MspClient::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  buf_.clear();
}

bool MspClient::fill(double timeout_s) {
  if (fd_ < 0) throw MspError("MSP socket is not connected");
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  int ms = static_cast<int>(std::max(0.0, timeout_s) * 1000.0);
  if (ms == 0 && timeout_s > 0.0) ms = 1;
  const int ready = ::poll(&pfd, 1, ms);
  if (ready < 0) {
    if (errno == EINTR) return false;
    throw MspError(std::string("poll: ") + std::strerror(errno));
  }
  if (ready == 0) return false;

  uint8_t tmp[4096];
  const ssize_t n = ::recv(fd_, tmp, sizeof(tmp), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return false;
    throw MspError(std::string("recv: ") + std::strerror(errno));
  }
  if (n == 0) throw MspError("MSP connection closed by peer");
  buf_.insert(buf_.end(), tmp, tmp + n);
  return true;
}

void MspClient::flush() {
  buf_.clear();
  while (fill(0.0)) buf_.clear();
}

std::optional<std::pair<uint8_t, std::vector<uint8_t>>> MspClient::recv_frame(
    double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(timeout_s));

  while (true) {
    // Resynchronise on a frame boundary, dropping any leading garbage.
    while (!buf_.empty() && buf_[0] != kHeader0) buf_.erase(buf_.begin());
    if (buf_.size() >= 2 && buf_[1] != kHeader1) {
      buf_.erase(buf_.begin());
      continue;
    }
    if (buf_.size() >= 3) {
      const char kind = static_cast<char>(buf_[2]);
      if (kind != kKindReply && kind != kKindError) {
        buf_.erase(buf_.begin(), buf_.begin() + 2);
        continue;
      }
      if (buf_.size() >= 4) {
        const uint8_t size = buf_[3];
        const size_t total = 5 + size + 1;
        if (buf_.size() >= total) {
          uint8_t cs = 0;
          const size_t end = 5u + size;
          for (size_t k = 3; k < end; ++k) cs ^= buf_[k];
          const uint8_t got = buf_[5 + size];
          const uint8_t cmd = buf_[4];
          std::vector<uint8_t> payload(buf_.begin() + 5,
                                       buf_.begin() + 5 + size);
          buf_.erase(buf_.begin(), buf_.begin() + total);
          if (cs != got) continue;  // corrupt frame, resync
          if (kind == kKindError) throw MspError("MSP error frame");
          return std::make_pair(cmd, std::move(payload));
        }
      }
    }

    if (timeout_s <= 0.0) return std::nullopt;
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return std::nullopt;
    const double remaining =
        std::chrono::duration<double>(deadline - now).count();
    if (!fill(remaining)) return std::nullopt;
  }
}

void MspClient::send_frame(uint8_t cmd, const std::vector<uint8_t>& payload) {
  if (fd_ < 0) throw MspError("MSP socket is not connected");
  std::lock_guard<std::mutex> lock(send_mutex_);
  std::vector<uint8_t> frame;
  frame.reserve(6 + payload.size());
  // MSP v1: the size byte counts the payload only (not the command byte).
  const uint8_t size = static_cast<uint8_t>(payload.size());
  frame.push_back(kHeader0);
  frame.push_back(kHeader1);
  frame.push_back(kKindRequest);
  frame.push_back(size);
  frame.push_back(cmd);
  uint8_t cs = static_cast<uint8_t>(size ^ cmd);
  for (uint8_t b : payload) {
    frame.push_back(b);
    cs ^= b;
  }
  frame.push_back(cs);

  size_t off = 0;
  while (off < frame.size()) {
    const ssize_t n =
        ::send(fd_, frame.data() + off, frame.size() - off, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) continue;
      throw MspError(std::string("send: ") + std::strerror(errno));
    }
    off += static_cast<size_t>(n);
  }
}

std::vector<uint8_t> MspClient::request(uint8_t cmd,
                                        const std::vector<uint8_t>& payload,
                                        int expect, int attempts) {
  const int want = expect >= 0 ? expect : static_cast<int>(cmd);
  // The flight controller occasionally drops or delays a reply while it is
  // busy with the RC stream, so resend a couple of times before giving up.
  for (int resend = 0; resend < 3; ++resend) {
    flush();
    send_frame(cmd, payload);
    for (int i = 0; i < attempts; ++i) {
      auto frame = recv_frame(timeout_);
      if (!frame) break;
      if (frame->first == want) return std::move(frame->second);
    }
  }
  throw MspError("MSP timeout waiting for command " + std::to_string(want));
}

ApiVersion MspClient::api_version() {
  const auto p = request(kApiVersion, {}, -1, 16);
  if (p.size() < 3) throw MspError("short MSP_API_VERSION");
  return ApiVersion{p[0], p[1], p[2]};
}

Attitude MspClient::attitude() {
  const auto p = request(kAttitude, {}, -1, 16);
  Attitude a;
  a.roll_deg = read_i16(p, 0, "MSP_ATTITUDE") / 10.0;
  a.pitch_deg = read_i16(p, 2, "MSP_ATTITUDE") / 10.0;
  a.yaw_deg = read_i16(p, 4, "MSP_ATTITUDE");
  return a;
}

RcFrame MspClient::rc() {
  const auto p = request(kRc, {}, -1, 16);
  RcFrame f;
  for (size_t o = 0; o + 2 <= p.size(); o += 2) {
    f.channels.push_back(read_u16(p, o, "MSP_RC"));
  }
  return f;
}

MotorFrame MspClient::motor() {
  const auto p = request(kMotor, {}, -1, 16);
  MotorFrame f;
  for (size_t o = 0; o + 2 <= p.size(); o += 2) {
    f.motors.push_back(read_u16(p, o, "MSP_MOTOR"));
  }
  return f;
}

ImuSample MspClient::raw_imu() {
  const auto p = request(kRawImu, {}, -1, 16);
  ImuSample s;
  for (int i = 0; i < 3; ++i) {
    s.acc[i] = read_i16(p, 2 * i, "MSP_RAW_IMU");
    s.gyro[i] = read_i16(p, 6 + 2 * i, "MSP_RAW_IMU");
    s.mag[i] = read_i16(p, 12 + 2 * i, "MSP_RAW_IMU");
  }
  return s;
}

StatusFrame MspClient::status(bool extended) {
  const auto p = request(extended ? kStatusEx : kStatus, {}, -1, 16);
  StatusFrame s;
  s.cycletime = read_u16(p, 0, "MSP_STATUS");
  s.i2c_errors = read_u16(p, 2, "MSP_STATUS");
  s.sensors = read_u16(p, 4, "MSP_STATUS");
  s.flight_mode_flags = read_u32(p, 6, "MSP_STATUS");
  s.armed = box_active(p, kBoxArm);
  s.msp_override = box_active(p, kBoxMspOverride);
  return s;
}

AnalogFrame MspClient::analog() {
  const auto p = request(kAnalog, {}, -1, 16);
  if (p.size() < 9) throw MspError("short MSP_ANALOG");
  AnalogFrame a;
  a.voltage_v = p[0] / 10.0;
  a.mah = read_u16(p, 1, "MSP_ANALOG");
  a.rssi = read_u16(p, 3, "MSP_ANALOG");
  a.amperage_a = read_u16(p, 5, "MSP_ANALOG") / 100.0;
  a.voltage_v = read_u16(p, 7, "MSP_ANALOG") / 100.0;
  return a;
}

AltitudeFrame MspClient::altitude() {
  const auto p = request(kAltitude, {}, -1, 16);
  AltitudeFrame a;
  const uint32_t cm = read_u32(p, 0, "MSP_ALTITUDE");
  a.vario_m_s = read_i16(p, 4, "MSP_ALTITUDE") / 100.0;
  // The SITL reports 0xFFFFFFFx when it has no estimate.
  a.valid = cm < 0xFFFFFF00u;
  a.altitude_m = a.valid ? cm / 100.0 : 0.0;
  return a;
}

bool MspClient::is_armed() { return status(true).armed; }

bool MspClient::is_msp_override_active() {
  return status(true).msp_override;
}

bool MspClient::is_box_active(uint16_t box) {
  const auto p = request(kStatusEx, {}, -1, 16);
  return box_active(p, box);
}

void MspClient::set_raw_rc(const std::vector<uint16_t>& channels,
                           double drain_timeout) {
  std::vector<uint8_t> payload;
  payload.reserve(channels.size() * 2);
  for (uint16_t ch : channels) {
    payload.push_back(static_cast<uint8_t>(ch & 0xFF));
    payload.push_back(static_cast<uint8_t>(ch >> 8));
  }
  send_frame(kSetRawRc, payload);

  // Waiting for the ack here would stall the RC stream (each recv can block for
  // the whole timeout), so it is only done when explicitly requested. Stale
  // acks are flushed by the next request()/flush().
  if (drain_timeout <= 0.0) return;
  try {
    for (int i = 0; i < 8; ++i) {
      if (!recv_frame(drain_timeout)) break;
    }
  } catch (const MspError&) {
    // Protocol noise while draining is not fatal.
  }
}

}  // namespace flyrat_msp
