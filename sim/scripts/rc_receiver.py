#!/usr/bin/env python3
"""Virtual RC receiver for the Betaflight SITL (UDP port 9004).

The SITL firmware runs ``RX_UDP``: it expects a packed ``rc_packet`` on UDP
9004 twice per frame::

    struct rc_packet {
        double   timestamp;                 // seconds
        uint16_t channels[16];              // microseconds, 1000..2000
    };

``channels[0..3]`` are roll/pitch/yaw/throttle (AETR); ``channels[4..]`` are
AUX1..AUX12. In our control scheme the receiver stays in charge of the
safety-critical switches:

  * AUX1 -> ARM
  * AUX2 -> MSP OVERRIDE (roll/pitch/yaw/throttle then come from MSP_SET_RAW_RC)

This class can be used as a library (see ``bringup_test.py``) or as a small
standalone tool.
"""
from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

CHANNEL_COUNT = 16
RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000

AUX_ARM = 4          # AUX1
AUX_MSP_OVERRIDE = 5  # AUX2
SWITCH_LOW = 1000
SWITCH_HIGH = 1800   # inside the 1700..2100 activation range from the CLI config


class RcReceiver:
    """Streams RC frames to the SITL on UDP 9004 at a fixed rate."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9004,
                 rate_hz: float = 50.0):
        self.host = host
        self.port = port
        self.rate_hz = rate_hz
        self.channels = [RC_MIN] * CHANNEL_COUNT
        self.channels[0] = RC_MID  # roll
        self.channels[1] = RC_MID  # pitch
        self.channels[2] = RC_MID  # yaw
        self.channels[3] = RC_MIN  # throttle
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self.sent = 0

    # -- channel helpers ----------------------------------------------------
    @staticmethod
    def _clamp(value: float) -> int:
        return int(max(RC_MIN, min(RC_MAX, value)))

    def set_channel(self, index: int, value: float) -> None:
        with self._lock:
            self.channels[index] = self._clamp(value)

    def set_aetr(self, roll=None, pitch=None, yaw=None, throttle=None) -> None:
        for index, value in enumerate((roll, pitch, yaw, throttle)):
            if value is not None:
                self.set_channel(index, value)

    def set_arm(self, on: bool) -> None:
        self.set_channel(AUX_ARM, SWITCH_HIGH if on else SWITCH_LOW)

    def set_msp_override(self, on: bool) -> None:
        self.set_channel(AUX_MSP_OVERRIDE, SWITCH_HIGH if on else SWITCH_LOW)

    def snapshot(self) -> list:
        with self._lock:
            return list(self.channels)

    # -- streaming ----------------------------------------------------------
    def _run(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        period = 1.0 / self.rate_hz
        next_at = time.monotonic()
        while not self._stop.is_set():
            channels = self.snapshot()
            packet = struct.pack(f"<d{CHANNEL_COUNT}H", time.monotonic(),
                                 *channels)
            try:
                self._sock.sendto(packet, (self.host, self.port))
                self.sent += 1
            except OSError:
                pass
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_at = time.monotonic()
        self._sock.close()
        self._sock = None

    def start(self) -> "RcReceiver":
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="rc-tx",
                                            daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def __enter__(self) -> "RcReceiver":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--rate", type=float, default=50.0,
                        help="frames per second (default 50)")
    parser.add_argument("--arm", action="store_true",
                        help="hold AUX1 (ARM) high")
    parser.add_argument("--msp-override", action="store_true",
                        help="hold AUX2 (MSP OVERRIDE) high")
    parser.add_argument("--throttle", type=int, default=RC_MIN)
    parser.add_argument("--roll", type=int, default=RC_MID)
    parser.add_argument("--pitch", type=int, default=RC_MID)
    parser.add_argument("--yaw", type=int, default=RC_MID)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds to transmit (0 = forever)")
    args = parser.parse_args()

    receiver = RcReceiver(args.host, args.port, args.rate)
    receiver.set_aetr(args.roll, args.pitch, args.yaw, args.throttle)
    receiver.set_arm(args.arm)
    receiver.set_msp_override(args.msp_override)
    receiver.start()
    print(f"[rc_receiver] streaming to {args.host}:{args.port} @ {args.rate} Hz "
          f"(arm={args.arm} msp_override={args.msp_override} "
          f"throttle={args.throttle})")
    try:
        if args.duration > 0:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1.0)
                print(f"[rc_receiver] sent {receiver.sent} frames")
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
