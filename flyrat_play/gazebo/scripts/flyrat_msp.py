#!/usr/bin/env python3
"""Minimal MSP v1 client for the Betaflight SITL stack.

The SITL exposes UART1 as a TCP socket (default 5761) carrying the ordinary
serial MSP protocol (``$M<`` request / ``$M>`` reply). This module implements
just enough of that protocol to drive the SITL from a companion computer or a
bring-up script:

  * :data:`MSP_SET_RAW_RC` - feed roll/pitch/yaw/throttle (and AUX) to the FC
  * :data:`MSP_ATTITUDE`   - estimated attitude
  * :data:`MSP_RAW_IMU`    - accelerometer / gyro
  * :data:`MSP_ALTITUDE`   - estimated altitude and variometer
  * :data:`MSP_MOTOR`      - motor outputs
  * :data:`MSP_RC`         - receiver channels as seen by the FC
  * :data:`MSP_ANALOG`     - battery / RSSI
  * :data:`MSP_STATUS[_EX]`- flight-mode / arming flags

Command ids and payload layouts mirror Betaflight
``src/main/msp/msp_protocol.h`` and ``src/main/msp/msp.c``.
"""
from __future__ import annotations

import select
import socket
import struct
import threading
import time

# --- MSP command ids -------------------------------------------------------
MSP_API_VERSION = 1
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_STATUS_EX = 150
MSP_ATTITUDE_QUATERNION = 167
MSP_SET_RAW_RC = 200

# --- Box ids (src/main/msp/msp_box.c) --------------------------------------
BOX_ARM = 0
BOX_MSP_OVERRIDE = 50

_HEADER = b"$M"
_REPLY = ord(">")
_ERROR = ord("!")
_REQUEST = ord("<")


class MspError(Exception):
    """Protocol level failure."""


class MspTimeout(MspError):
    """No (matching) frame arrived in time."""


def _xor(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


class MspClient:
    """Synchronous MSP v1 client over a TCP-streamed UART."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5761,
                 timeout: float = 0.5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._buf = bytearray()
        # Guards _send only, so a fire-and-forget RC writer thread and a
        # telemetry reader can share one socket without interleaving bytes.
        self._send_lock = threading.Lock()
        # Guards the receive side (_fill/pump/flush), which is also touched by
        # the RC writer thread when it drains the flight controller's replies.
        self._recv_lock = threading.Lock()

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> "MspClient":
        self._sock = socket.create_connection((self.host, self.port),
                                              timeout=self.timeout)
        self._sock.settimeout(self.timeout)
        self._buf.clear()
        return self

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def flush(self) -> None:
        """Discard anything already queued on the socket / receive buffer.

        Stale reply frames (e.g. ACKs of ``MSP_SET_RAW_RC``) would otherwise
        exhaust the retry budget of the next request, so reads drop them first.

        ``select`` is used to test readability instead of switching the socket
        to non-blocking mode. The socket is shared with a fire-and-forget RC
        writer thread; if ``sendall`` ever ran while the socket was temporarily
        non-blocking it could emit a *truncated* MSP frame, which desynchronises
        the firmware's parser and wedges its MSP task.
        """
        with self._recv_lock:
            self._buf.clear()
            if self._sock is None:
                return
            while True:
                try:
                    ready, _, _ = select.select([self._sock], [], [], 0)
                except (OSError, ValueError):
                    break
                if not ready:
                    break
                try:
                    data = self._sock.recv(4096)
                except (socket.timeout, BlockingIOError, ConnectionError):
                    break
                if not data:
                    break

    def pump(self) -> None:
        """Move whatever the FC has sent into the receive buffer (non-blocking).

        The firmware acknowledges *every* ``MSP_SET_RAW_RC`` frame. If those
        replies are never read they fill the emulated UART's transmit buffer,
        after which the firmware silently drops later replies -- including the
        telemetry a ``request`` is waiting for. Call this regularly from the
        high-rate RC writer thread so the link never backs up; it never blocks
        and the buffered frames are simply skipped by the next ``request``.
        """
        if self._sock is None:
            return
        # Never block: this is called from the high-rate RC writer, and the
        # telemetry reader may hold the lock for the duration of a request. A
        # missed drain is harmless -- the next call (or flush) catches up.
        if not self._recv_lock.acquire(blocking=False):
            return
        try:
            while True:
                try:
                    ready, _, _ = select.select([self._sock], [], [], 0)
                except (OSError, ValueError):
                    return
                if not ready:
                    return
                try:
                    data = self._sock.recv(4096)
                except (socket.timeout, BlockingIOError):
                    return
                if not data:
                    raise MspError("MSP connection closed by the flight controller")
                self._buf.extend(data)
        finally:
            self._recv_lock.release()

    def __enter__(self) -> "MspClient":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- framing ------------------------------------------------------------
    def _fill(self, timeout: float | None = None) -> None:
        assert self._sock is not None
        with self._recv_lock:
            previous = self._sock.gettimeout()
            if timeout is not None:
                self._sock.settimeout(timeout)
            try:
                data = self._sock.recv(4096)
            except socket.timeout as exc:
                raise MspTimeout("timed out waiting for an MSP frame") from exc
            finally:
                if timeout is not None:
                    self._sock.settimeout(previous)
            if not data:
                raise MspError("MSP connection closed by the flight controller")
            self._buf.extend(data)

    def _recv_frame(self, timeout: float | None = None) -> tuple[int, bytes]:
        while True:
            index = self._buf.find(_HEADER)
            if index < 0:
                # keep the last byte in case '$' and 'M' straddle two reads
                if len(self._buf) > 1:
                    del self._buf[:-1]
                self._fill(timeout)
                continue
            if index > 0:
                del self._buf[:index]
            if len(self._buf) < 5:
                self._fill(timeout)
                continue
            kind = self._buf[2]
            if kind not in (_REPLY, _ERROR):
                del self._buf[:1]
                continue
            size = self._buf[3]
            total = 5 + size + 1
            if len(self._buf) < total:
                self._fill(timeout)
                continue
            frame = bytes(self._buf[:total])
            del self._buf[:total]
            cmd = frame[4]
            payload = frame[5:5 + size]
            if _xor(frame[3:5 + size]) != frame[5 + size]:
                continue  # checksum mismatch: drop and resynchronise
            if kind == _ERROR:
                raise MspError(f"flight controller rejected MSP command {cmd}")
            return cmd, payload

    def _send(self, cmd: int, payload: bytes = b"") -> None:
        if self._sock is None:
            raise MspError("client is not connected")
        body = bytes([len(payload) & 0xFF, cmd & 0xFF]) + payload
        frame = _HEADER + bytes([_REQUEST]) + body + bytes([_xor(body)])
        with self._send_lock:
            self._sock.sendall(frame)

    def request(self, cmd: int, payload: bytes = b"", *,
                expect: int | None = None, attempts: int = 256,
                timeout: float = 0.05, resends: int = 5,
                round_timeout: float = 0.4):
        """Send ``cmd`` and return the reply matching ``expect`` (default ``cmd``).

        The flight controller acknowledges other commands (notably every
        ``MSP_SET_RAW_RC``), so unrelated frames on the wire are skipped freely:
        the inner loop runs until a match arrives or ``round_timeout`` elapses,
        then the request is resent. ``attempts`` only bounds how many frames a
        single round may consume. Short per-frame timeouts keep a read from
        blocking the caller (and any shared use of the socket) for long.
        """
        want = cmd if expect is None else expect
        last = None
        for _ in range(resends):
            self.flush()
            self._send(cmd, payload)
            deadline = time.monotonic() + round_timeout
            consumed = 0
            while consumed < attempts and time.monotonic() < deadline:
                try:
                    reply_cmd, reply_payload = self._recv_frame(timeout)
                except MspTimeout:
                    continue  # keep waiting until the round deadline
                consumed += 1
                last = reply_cmd
                if reply_cmd == want:
                    return reply_cmd, reply_payload
        raise MspTimeout(f"no reply for command {cmd} (saw {last})")

    # -- high level helpers -------------------------------------------------
    def api_version(self) -> tuple[int, int, int]:
        _, payload = self.request(MSP_API_VERSION)
        return struct.unpack_from("<BBB", payload, 0)

    def set_raw_rc(self, channels, *, wait: bool = True,
                   ack_timeout: float = 0.01, drain: bool = True) -> None:
        payload = struct.pack(f"<{len(channels)}H", *channels)
        self._send(MSP_SET_RAW_RC, payload)
        if wait:
            # Best effort ACK drain; unresolved frames are skipped by later
            # request() calls, so a missing/slow ACK never breaks the loop.
            try:
                self._recv_frame(timeout=ack_timeout)
            except MspTimeout:
                pass
        elif drain:
            # The FC ACKs every one of these frames; keep the UART's transmit
            # buffer empty so telemetry replies are not dropped later on.
            self.pump()

    def status(self, *, extended: bool = False) -> dict:
        cmd = MSP_STATUS_EX if extended else MSP_STATUS
        _, payload = self.request(cmd)
        cycletime, i2c_errors, sensors = struct.unpack_from("<HHH", payload, 0)
        flags = struct.unpack_from("<I", payload, 6)[0]
        return {"cycletime": cycletime, "i2c_errors": i2c_errors,
                "sensors": sensors, "flight_mode_flags": flags}

    def is_armed(self) -> bool:
        return bool(self.status()["flight_mode_flags"] & (1 << BOX_ARM))

    def is_msp_override_active(self) -> bool:
        return bool(self.status()["flight_mode_flags"] &
                    (1 << BOX_MSP_OVERRIDE))

    def attitude(self) -> dict:
        _, payload = self.request(MSP_ATTITUDE)
        roll, pitch, yaw = struct.unpack_from("<hhh", payload, 0)
        return {"roll": roll / 10.0, "pitch": pitch / 10.0, "yaw": float(yaw)}

    def altitude_cm(self) -> tuple[int, int]:
        _, payload = self.request(MSP_ALTITUDE)
        return struct.unpack_from("<Ih", payload, 0)

    def raw_imu(self) -> tuple:
        _, payload = self.request(MSP_RAW_IMU)
        return struct.unpack_from("<9h", payload, 0)

    def motor(self) -> tuple:
        _, payload = self.request(MSP_MOTOR)
        return struct.unpack_from("<8H", payload, 0)

    def rc(self) -> list:
        _, payload = self.request(MSP_RC)
        count = len(payload) // 2
        return list(struct.unpack_from(f"<{count}H", payload, 0))

    def analog(self) -> dict:
        _, payload = self.request(MSP_ANALOG)
        vbat, mah, rssi, amperage, voltage = struct.unpack_from("<BHHHH",
                                                                payload, 0)
        return {"vbat": vbat / 10.0, "mah": mah, "rssi": rssi,
                "amperage": amperage / 100.0, "voltage": voltage / 100.0}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Query a Betaflight SITL over MSP")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    args = parser.parse_args()

    with MspClient(args.host, args.port) as client:
        print("api_version     :", client.api_version())
        print("status          :", client.status(extended=True))
        print("armed           :", client.is_armed())
        print("msp override    :", client.is_msp_override_active())
        print("attitude        :", client.attitude())
        print("altitude_cm     :", client.altitude_cm())
        print("rc              :", client.rc())
        print("motor           :", client.motor())
