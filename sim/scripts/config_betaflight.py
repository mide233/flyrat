#!/usr/bin/env python3
"""Apply the Betaflight SITL flight-controller configuration.

Betaflight exposes both MSP and the CLI on its UARTs. SITL maps UART1 to
TCP 5761, so this script opens that socket, switches the firmware into CLI
mode (the '#' byte), feeds the commands from sim/config/betaflight_sim.txt and
finally issues 'save' to persist them to eeprom.bin.

The settings themselves (MSP override mask, ARM / MSP OVERRIDE mode ranges)
live in the committed text file, so the configuration stays auditable and
reproducible.

Usage:
    sim/scripts/config_betaflight.py [--host 127.0.0.1] [--port 5761]
                                     [--commands sim/config/betaflight_sim.txt]
                                     [--no-save]
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_COMMANDS = os.path.join(SIM_DIR, "config", "betaflight_sim.txt")


def read_commands(path: str) -> list[str]:
    commands: list[str] = []
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            commands.append(line)
    return commands


def drain(sock: socket.socket, timeout: float = 1.5) -> str:
    """Read whatever the CLI produces until it goes quiet."""
    sock.settimeout(0.25)
    deadline = time.time() + timeout
    chunks: list[bytes] = []
    while time.time() < deadline:
        try:
            data = sock.recv(8192)
        except socket.timeout:
            continue
        if not data:
            break
        chunks.append(data)
    return b"".join(chunks).decode(errors="replace")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--commands", default=DEFAULT_COMMANDS)
    parser.add_argument("--no-save", action="store_true",
                        help="apply the settings but do not persist them")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    commands = read_commands(args.commands)
    if not commands:
        print(f"error: no commands found in {args.commands}", file=sys.stderr)
        return 1

    try:
        sock = socket.create_connection((args.host, args.port), timeout=5)
    except OSError as exc:
        print(f"error: cannot connect to MSP/CLI at {args.host}:{args.port}: {exc}",
              file=sys.stderr)
        return 1

    with sock:
        # '#' followed by CR switches the UART into CLI mode.
        sock.sendall(b"#\r")
        banner = drain(sock, timeout=1.0)
        if not args.quiet and banner:
            print(banner.strip())

        for command in commands:
            sock.sendall(command.encode() + b"\r\n")
            output = drain(sock)
            if not args.quiet:
                print(output.strip())

        if not args.no_save:
            sock.sendall(b"save\r\n")
            save_output = drain(sock, timeout=2.0)
            if not args.quiet:
                print(save_output.strip())

    print(f"applied {len(commands)} command(s) from {args.commands}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
