#!/usr/bin/env python3
"""End-to-end bring-up test for the Betaflight SITL + Gazebo stack (P6).

It exercises exactly the interfaces the ROS 2 layer will use later:

  1. the virtual receiver on UDP 9004 (``rc_receiver.RcReceiver``) which carries
     the safety switches (AUX1 = ARM, AUX2 = MSP OVERRIDE),
  2. the MSP UART on TCP 5761 (``flyrat_msp.MspClient``) which carries
     roll/pitch/yaw/throttle through ``MSP_SET_RAW_RC``.

Two things must be true for this test (and the real autopilot) to work, and both
were bugs worth documenting:

  * The RC stream must never stall. The firmware only applies the MSP override
    while ``MSP_SET_RAW_RC`` frames are fresh (``RX_MSP_RC_FRAME_FRESH_MS`` =
    300 ms); on a gap it silently falls back to the *receiver's* channels (idle
    throttle here), the motors drop to idle and the vehicle stops climbing. So a
    dedicated sender thread pushes ``MSP_SET_RAW_RC`` at ``--rate`` Hz and is
    never paused - not for telemetry, not for anything. The test verifies the
    largest inter-frame gap stayed below 300 ms.

  * The MSP socket must not be driven by two things at once in a way that can
    corrupt a frame. The socket is shared by the sender and the telemetry
    reader; every write is a single ``sendall`` of a whole frame under a lock,
    and incoming data is drained with ``select`` (never by switching the socket
    to non-blocking, which could truncate a concurrent ``sendall``). A truncated
    frame desynchronises the firmware's MSP parser, which will then treat a stray
    ``#`` byte as "enter CLI mode" and stop answering MSP altogether until the
    SITL is restarted.

The Gazebo pose oracle is used ONLY here, to verify the bridge independently of
the (unreliable in SITL) ``MSP_ALTITUDE``. It is sampled once before and once at
the end of the climb: each sample spawns a ``gz model -p`` subprocess, and doing
that continuously demonstrably starves the simulation and suppresses the climb.

Checks performed:
  * MSP link answers (API version readable),
  * the receiver link is accepted (no immediate RXLOSS),
  * MSP OVERRIDE is genuinely active - verified by giving the receiver and MSP
    *different* roll/pitch/yaw and reading ``MSP_RC`` back,
  * arming succeeds,
  * (with ``--takeoff``) the vehicle climbs when throttle is raised,
  * afterwards it disarms cleanly,
  * the RC stream stayed fresh (largest inter-frame gap < 300 ms).

Run it against a live stack, e.g.
    flyrat_play/gazebo/scripts/launch_sim.sh &            # in another shell
    python3 flyrat_play/gazebo/scripts/bringup_test.py --takeoff
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import threading
import time

from flyrat_msp import MspClient, MspError, MspTimeout
from rc_receiver import RcReceiver, RC_MID, RC_MIN

# Distinctive values so we can tell receiver- from MSP-sourced channels.
RX_BASELINE = 1050      # receiver roll/pitch/yaw while MSP override is active
MSP_CENTER = 1500
# The firmware's MSP RC freshness window (rx/msp.c RX_MSP_RC_FRAME_FRESH_MS).
RC_FRESH_MS = 300.0


class BringupTest:
    def __init__(self, args):
        self.args = args
        self.receiver = RcReceiver(args.host, args.rc_port, args.rate)
        self.msp: MspClient | None = None
        self.failures: list[str] = []

        # Desired MSP_SET_RAW_RC attitude/throttle, shared with the sender
        # thread. AUX channels always come from the receiver snapshot.
        self._cmd = {"roll": MSP_CENTER, "pitch": MSP_CENTER,
                     "yaw": MSP_CENTER, "throttle": RC_MIN}
        self._cmd_lock = threading.Lock()
        # Serialises every use of the single MSP socket (sender + reads).
        self._msp_lock = threading.Lock()
        self._stop = threading.Event()
        self._sender: threading.Thread | None = None

        # Send timestamps, used to prove the RC stream stayed fresh.
        self._send_times: list[float] = []
        self._send_times_lock = threading.Lock()

    # -- helpers ------------------------------------------------------------
    def _check(self, label: str, ok: bool, detail: str = "") -> bool:
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {label}" + (f" - {detail}" if detail else ""))
        if not ok:
            self.failures.append(label)
        return ok

    def _set_cmd(self, roll=None, pitch=None, yaw=None, throttle=None) -> None:
        with self._cmd_lock:
            for key, value in (("roll", roll), ("pitch", pitch),
                               ("yaw", yaw), ("throttle", throttle)):
                if value is not None:
                    self._cmd[key] = int(value)

    def _read(self, fn, *args, **kwargs):
        """Run a blocking MspClient method (telemetry etc.).

        The sender thread keeps pushing RC frames throughout: reads and writes
        use the same socket but never block each other, and ``request`` is
        bounded, so a read cannot stall the control stream.
        """
        with self._msp_lock:
            return fn(*args, **kwargs)

    # -- background RC sender ----------------------------------------------
    def _sender_loop(self) -> None:
        period = 1.0 / self.args.rate
        next_at = time.monotonic()
        while not self._stop.is_set():
            with self._cmd_lock:
                cmd = dict(self._cmd)
            # MSP_SET_RAW_RC takes the raw receiver order: roll, pitch,
            # throttle, yaw, then AUX1.. (note: *not* AETR).
            channels = [cmd["roll"], cmd["pitch"], cmd["throttle"], cmd["yaw"]]
            channels += self.receiver.snapshot()[4:]
            # Fire-and-forget: never read ACKs here. Reads flush stale replies.
            try:
                self.msp.set_raw_rc(channels, wait=False)
                with self._send_times_lock:
                    self._send_times.append(time.monotonic())
            except (MspError, OSError):
                pass
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_at = time.monotonic()

    def _start_threads(self) -> None:
        self._stop.clear()
        self._sender = threading.Thread(target=self._sender_loop,
                                        name="msp-sender", daemon=True)
        self._sender.start()

    def _stop_threads(self) -> None:
        self._stop.set()
        if self._sender is not None:
            self._sender.join(timeout=2.0)
            self._sender = None

    def _pump(self, seconds: float) -> None:
        """Keep both links fed for ``seconds``; the sender thread does the work."""
        time.sleep(seconds)

    def _max_send_gap_ms(self) -> float:
        with self._send_times_lock:
            times = list(self._send_times)
        if len(times) < 2:
            return float("inf")
        return max((b - a) for a, b in zip(times, times[1:])) * 1000.0

    # -- phases -------------------------------------------------------------
    def phase_link(self) -> None:
        version = self._read(self.msp.api_version)
        self._check("MSP link is up", True,
                    f"protocol v{version[0]}, API {version[1]}.{version[2]}")

    def phase_override(self) -> None:
        # Receiver and MSP are deliberately different: if MSP_RC echoes the MSP
        # values, the override is really in charge of roll/pitch/yaw.
        self.receiver.set_aetr(RX_BASELINE, RX_BASELINE, RX_BASELINE)
        self.receiver.set_msp_override(True)
        self._set_cmd(roll=MSP_CENTER, pitch=MSP_CENTER, yaw=MSP_CENTER,
                      throttle=RC_MIN)
        time.sleep(1.0)

        channels = self._read(self.msp.rc)
        # MSP_RC returns the internal order: roll, pitch, yaw, throttle, AUX1..
        roll, pitch, yaw = channels[0], channels[1], channels[2]
        aux1 = channels[4] if len(channels) > 4 else -1
        ok = (abs(roll - MSP_CENTER) <= 25 and abs(pitch - MSP_CENTER) <= 25
              and abs(yaw - MSP_CENTER) <= 25)
        self._check("MSP OVERRIDE drives roll/pitch/yaw", ok,
                    f"FC sees roll/pitch/yaw={roll}/{pitch}/{yaw} "
                    f"(MSP={MSP_CENTER}, receiver={RX_BASELINE})")
        self._check("ARM (AUX1) stays receiver-driven",
                    abs(aux1 - self.receiver.snapshot()[4]) <= 5,
                    f"FC AUX1={aux1}, receiver AUX1={self.receiver.snapshot()[4]}")
        # Hand roll/pitch/yaw back to neutral; from here on only the MSP side
        # moves the vehicle, so a stale-frame fallback is a level command.
        self.receiver.set_aetr(MSP_CENTER, MSP_CENTER, MSP_CENTER)

    def phase_arm(self) -> None:
        self.receiver.set_aetr(MSP_CENTER, MSP_CENTER, MSP_CENTER)
        self.receiver.set_arm(True)
        end = time.monotonic() + self.args.arm_timeout
        armed = False
        while time.monotonic() < end:
            time.sleep(0.5)
            try:
                if self._read(self.msp.is_armed):
                    armed = True
                    break
            except MspTimeout:
                pass
        if not armed:
            try:
                print("       FC channels:", self._read(self.msp.rc))
                print("       status      :",
                      self._read(self.msp.status, extended=True))
            except MspTimeout:
                pass
        self._check("vehicle armed", armed,
                    f"within {self.args.arm_timeout:.1f}s"
                    if armed else "timed out (check SITL log for RXLOSS)")

    # -- Gazebo ground-truth oracle (bring-up test ONLY) --------------------
    # The control loop never reads Gazebo state; this exists purely to verify
    # the bridge independently of the (unreliable in SITL) MSP_ALTITUDE.
    def _gazebo_pose(self):
        """Return (x, y, z, roll, pitch, yaw) of the iris, or None."""
        try:
            out = subprocess.run(["gz", "model", "-m", "iris", "-p"],
                                 capture_output=True, text=True,
                                 timeout=3.0).stdout
        except (OSError, subprocess.SubprocessError):
            return None
        triples = re.findall(r"\[\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s*\]",
                             out)
        if len(triples) < 2:
            return None
        return tuple(float(v) for t in triples[:2] for v in t)

    def phase_takeoff(self) -> None:
        baseline_pose = self._gazebo_pose()
        baseline = baseline_pose[2] if baseline_pose else None
        start = time.monotonic()
        end = start + self.args.climb_timeout
        while time.monotonic() < end:
            elapsed = time.monotonic() - start
            throttle = min(self.args.hover_throttle,
                           RC_MIN + (self.args.hover_throttle - RC_MIN) *
                           min(1.0, elapsed / 2.0))
            self._set_cmd(throttle=throttle)
            # Nothing blocking in this loop: the RC sender must stay on
            # schedule. The pose is sampled once afterwards.
            time.sleep(1.0 / self.args.rate)

        # One sample while still at full throttle; a climbing vehicle only
        # descends after throttle is cut, so this captures the peak.
        peak_pose = self._gazebo_pose()
        if peak_pose is not None and baseline is not None:
            peak = peak_pose[2]
            climbed = (peak - baseline) >= self.args.climb_threshold_m
            self._check("vehicle climbed under throttle", climbed,
                        f"gazebo z: baseline={baseline:.3f}m "
                        f"peak={peak:.3f}m delta={peak - baseline:.3f}m")
            # Gazebo pose roll/pitch are radians.
            tilt = max(abs(math.degrees(peak_pose[3])),
                       abs(math.degrees(peak_pose[4])))
        else:
            self._check("vehicle climbed under throttle", False,
                        "Gazebo pose oracle unavailable")
            tilt = 0.0
        self._check("vehicle stayed upright", tilt < 45.0,
                    f"tilt (Gazebo pose) = {tilt:.1f} deg")

    def phase_disarm(self) -> None:
        # Cut throttle, hold for a moment, then drop ARM.
        self._set_cmd(throttle=RC_MIN)
        time.sleep(1.5)
        self.receiver.set_arm(False)
        # Give the FC a moment and retry the (sometimes contended) read.
        armed = True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                armed = self._read(self.msp.is_armed)
            except MspTimeout:
                time.sleep(0.1)
                continue
            if not armed:
                break
            time.sleep(0.1)
        self._check("vehicle disarmed", not armed)

    # -- driver -------------------------------------------------------------
    def run(self) -> int:
        print(f"connecting to SITL  MSP={self.args.host}:{self.args.msp_port}  "
              f"RC={self.args.host}:{self.args.rc_port}")
        self.msp = MspClient(self.args.host, self.args.msp_port).connect()
        # Provide a receiver link before the SITL starts evaluating failsafe.
        self.receiver.set_msp_override(True)
        self.receiver.start()
        self._start_threads()
        try:
            self._pump(0.5)          # let the SITL see the receiver link
            self.phase_link()
            self.phase_override()
            self.phase_arm()
            if self.args.takeoff:
                self.phase_takeoff()
            else:
                # Give the freshness measurement a meaningful window.
                self._pump(1.0)
            self.phase_disarm()
        finally:
            self._stop_threads()
            self.receiver.stop()
            self.msp.close()

        gap_ms = self._max_send_gap_ms()
        self._check("RC stream stayed fresh (< 300ms gaps)", gap_ms < RC_FRESH_MS,
                    f"max gap = {gap_ms:.0f}ms")

        print()
        if self.failures:
            print(f"RESULT: FAILED ({len(self.failures)}): "
                  + ", ".join(self.failures))
            return 1
        print("RESULT: OK")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--msp-port", type=int, default=5761)
    parser.add_argument("--rc-port", type=int, default=9004)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--arm-timeout", type=float, default=10.0)
    parser.add_argument("--takeoff", action="store_true",
                        help="also verify the vehicle climbs")
    parser.add_argument("--climb-timeout", type=float, default=8.0)
    parser.add_argument("--climb-threshold-m", type=float, default=0.25)
    parser.add_argument("--hover-throttle", type=int, default=1800)
    args = parser.parse_args()

    try:
        return BringupTest(args).run()
    except (MspError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
