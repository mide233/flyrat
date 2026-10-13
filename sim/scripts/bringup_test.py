#!/usr/bin/env python3
"""End-to-end bring-up test for the Betaflight SITL + Gazebo stack (P6).

It exercises exactly the interfaces the ROS 2 layer will use later:

  1. the virtual receiver on UDP 9004 (``rc_receiver.RcReceiver``) which carries
     the safety switches (AUX1 = ARM, AUX2 = MSP OVERRIDE),
  2. the MSP UART on TCP 5761 (``flyrat_msp.MspClient``) which carries
     roll/pitch/yaw/throttle through ``MSP_SET_RAW_RC``.

Control loops must feed ``MSP_SET_RAW_RC`` faster than the firmware's RC
freshness window (``RX_MSP_RC_FRAME_FRESH_MS`` = 300 ms). If frames go stale the
flight controller silently falls back to the *receiver's* channels, so this
test runs a dedicated background "pusher" thread at ``--rate`` Hz. That mirrors
the intended ROS 2 node design (a high-rate RC loop independent of any slow
telemetry/pose work).

Checks performed:
  * MSP link answers (API version readable),
  * the receiver link is accepted (no immediate RXLOSS),
  * MSP OVERRIDE is genuinely active - verified by giving the receiver and MSP
    *different* roll/pitch/yaw and reading ``MSP_RC`` back,
  * arming succeeds,
  * (with ``--takeoff``) the vehicle climbs when throttle is raised,
  * afterwards it disarms cleanly.

Run it against a live stack, e.g.
    sim/scripts/launch_sim.sh &            # in another shell
    python3 sim/scripts/bringup_test.py --takeoff
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


class BringupTest:
    def __init__(self, args):
        self.args = args
        self.receiver = RcReceiver(args.host, args.rc_port, args.rate)
        self.msp: MspClient | None = None
        self.failures: list[str] = []

        # Desired MSP_SET_RAW_RC attitude/throttle, shared with the pusher
        # thread. AUX channels always come from the receiver snapshot.
        self._cmd = {"roll": MSP_CENTER, "pitch": MSP_CENTER,
                     "yaw": MSP_CENTER, "throttle": RC_MIN}
        self._cmd_lock = threading.Lock()
        # Serialises every use of the single MSP socket (pusher + reads).
        self._msp_lock = threading.Lock()
        self._pusher_stop = threading.Event()
        # While set, the pusher stops sending so a telemetry read can own the
        # single MSP socket. The SITL tolerates ~300ms without RC frames, so a
        # short read window is safe and keeps the two directions serialised.
        self._pause = threading.Event()
        self._pusher: threading.Thread | None = None

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
        """Pause the RC pusher, then run a blocking MspClient method.

        The RC frames and the telemetry reads share one MSP socket; letting
        them interleave perturbs the stream and can make the MSP override go
        stale. Pausing the pusher for the (short) read keeps the traffic
        serialised, and stays well inside the firmware's 300ms RC freshness
        window.
        """
        self._pause.set()
        time.sleep(0.05)  # let any in-flight send finish
        try:
            with self._msp_lock:
                return fn(*args, **kwargs)
        finally:
            self._pause.clear()

    def _push_msp(self, roll=MSP_CENTER, pitch=MSP_CENTER, yaw=MSP_CENTER,
                  throttle=RC_MIN) -> None:
        """Send one MSP_SET_RAW_RC frame mirroring the receiver's AUX channels.

        ``MSP_SET_RAW_RC`` uses the raw receiver order, i.e.
        roll, pitch, throttle, yaw, then AUX1.. (note: *not* AETR).
        """
        channels = [int(roll), int(pitch), int(throttle), int(yaw)]
        channels += self.receiver.snapshot()[4:]
        with self._msp_lock:
            self.msp.set_raw_rc(channels)

    # -- background pusher --------------------------------------------------
    def _pusher_loop(self) -> None:
        period = 1.0 / self.args.rate
        next_at = time.monotonic()
        sent = 0
        while not self._pusher_stop.is_set():
            if self._pause.is_set():
                self._pusher_stop.wait(0.01)
                continue
            with self._cmd_lock:
                cmd = dict(self._cmd)
            channels = [cmd["roll"], cmd["pitch"], cmd["throttle"], cmd["yaw"]]
            channels += self.receiver.snapshot()[4:]
            # Fire-and-forget: never read ACKs here, otherwise this thread would
            # race the telemetry reader for reply frames (and block on the
            # socket). Reads flush stale ACKs themselves.
            try:
                self.msp.set_raw_rc(channels, wait=False)
            except (MspError, OSError):
                pass
            sent += 1
            # The FC acknowledges every SET_RAW_RC frame; if those replies are
            # never drained they back up and eventually stall the firmware's
            # MSP task, which then stops consuming our RC override. Discard
            # them periodically. Reads pause us first, so this can never eat a
            # telemetry reply.
            if sent % 20 == 0 and not self._pause.is_set():
                try:
                    with self._msp_lock:
                        self.msp.flush()
                except (MspError, OSError):
                    pass
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                self._pusher_stop.wait(delay)
            else:
                next_at = time.monotonic()

    def _start_pusher(self) -> None:
        if self._pusher is None:
            self._pusher_stop.clear()
            self._pusher = threading.Thread(target=self._pusher_loop,
                                            name="msp-pusher", daemon=True)
            self._pusher.start()

    def _stop_pusher(self) -> None:
        self._pusher_stop.set()
        if self._pusher is not None:
            self._pusher.join(timeout=1.0)
            self._pusher = None

    def _pump(self, seconds: float) -> None:
        """Keep both links fed for ``seconds``; the pusher thread does the work."""
        time.sleep(seconds)

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
        # Poll slowly: every read briefly pauses the RC pusher, and hammering
        # the socket at 50 Hz while the link is settling keeps the override
        # stale and can hold the FC in RXLOSS.
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
                      self._read(self.msp.status, True))
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

    def _altitude_m(self):
        """Best-effort MSP_ALTITUDE in metres; None for the invalid sentinel."""
        try:
            altitude, _ = self._read(self.msp.altitude_cm)
        except MspTimeout:
            return None
        # 0xFFFFFFFF (and 0xFFFFFFFE) are the "no estimate" sentinels.
        if altitude >= 0xFFFFFF00:
            return None
        return altitude / 100.0

    def phase_takeoff(self) -> None:
        pose = self._gazebo_pose()
        baseline = pose[2] if pose else None
        peak = baseline
        max_tilt = 0.0
        # Sample Gazebo only a few times per second: each sample spawns a
        # `gz model` subprocess, and doing that at the 50Hz loop rate starves
        # Gazebo/SITL and destroys the real-time sync (and thus the thrust).
        pose_period = 0.5
        next_pose = time.monotonic()
        end = time.monotonic() + self.args.climb_timeout
        ramp = self.args.climb_timeout
        while time.monotonic() < end:
            elapsed = ramp - (end - time.monotonic())
            throttle = min(self.args.hover_throttle,
                           RC_MIN + (self.args.hover_throttle - RC_MIN) *
                           min(1.0, elapsed / 2.0))
            self._set_cmd(throttle=throttle)
            # No MSP reads in this thrust-critical loop: they would starve the
            # 50Hz RC push and let the MSP override go stale (falling back to
            # the receiver's idle throttle). Climb and tilt come from the
            # independent Gazebo pose oracle instead.
            now = time.monotonic()
            if now >= next_pose:
                next_pose = now + pose_period
                pose = self._gazebo_pose()
                if pose is not None:
                    if peak is None:
                        peak = pose[2]
                    peak = max(peak, pose[2])
                    # Gazebo pose roll/pitch are radians.
                    max_tilt = max(max_tilt, abs(math.degrees(pose[3])),
                                   abs(math.degrees(pose[4])))
            time.sleep(1.0 / self.args.rate)

        # Climb is proven by the independent Gazebo pose oracle.
        if baseline is not None and peak is not None:
            climbed = (peak - baseline) >= self.args.climb_threshold_m
            self._check("vehicle climbed under throttle", climbed,
                        f"gazebo z: baseline={baseline:.3f}m "
                        f"peak={peak:.3f}m delta={peak - baseline:.3f}m")
        else:
            self._check("vehicle climbed under throttle", False,
                        "Gazebo pose oracle unavailable")
        self._check("vehicle stayed upright", max_tilt < 45.0,
                    f"max tilt (Gazebo pose) = {max_tilt:.1f} deg")

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
        self._start_pusher()
        try:
            self._pump(0.5)          # let the SITL see the receiver link
            self.phase_link()
            self.phase_override()
            self.phase_arm()
            if self.args.takeoff:
                self.phase_takeoff()
            self.phase_disarm()
        finally:
            self._stop_pusher()
            self.receiver.stop()
            self.msp.close()

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
