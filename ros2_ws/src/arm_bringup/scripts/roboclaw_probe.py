#!/usr/bin/env python3
"""Check the shared RoboClaw packet-serial UART: who answers, and their live data.

    roboclaw_probe.py                                  # scan 128-135 on /dev/roboclaw @ 38400
    roboclaw_probe.py --port /dev/ttyTHS1 --baud 115200
    roboclaw_probe.py --addresses 128 --watch          # stream encoders/currents at 5 Hz

Read-only: it never commands motion. Run it with arm.launch.py STOPPED (only one
program can own the UART). Standard library only.

Packet serial framing (BasicMicro RoboClaw user manual, "Packet Serial"):
    read  request : [address][command]
    read  reply   : [data ...][CRC16 hi][CRC16 lo]   CRC over address+command+data
    write request : [address][command][data ...][CRC16 hi][CRC16 lo]
    write reply   : 0xFF
    CRC16 = CCITT / XMODEM: polynomial 0x1021, initial value 0, no reflection.
"""

import argparse
import os
import select
import struct
import sys
import termios
import time

FIRMWARE, MAIN_BATTERY, CURRENTS, TEMPERATURE, ENCODERS, STATUS = 21, 24, 49, 82, 78, 90


def crc16(data, crc=0):
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


class Port:
    def __init__(self, path, baud):
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            sys.exit(f"unsupported baud {baud}")
        try:
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            sys.exit(f"cannot open {path}: {e} (udev symlink? dialout group?)")
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = termios.IGNPAR                                   # iflag
        attrs[1] = 0                                                # oflag
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD     # cflag: 8N1
        attrs[3] = 0                                                # lflag: raw
        attrs[4] = attrs[5] = speed                                 # ispeed / ospeed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def read_command(self, address, command, idle=0.02, timeout=0.1):
        """Send a read command; return the reply payload (CRC stripped), or None."""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, bytes([address, command]))
        reply, deadline = b"", time.monotonic() + timeout
        while time.monotonic() < deadline:
            wait = idle if reply else deadline - time.monotonic()
            if not select.select([self.fd], [], [], max(wait, 0))[0]:
                if reply:
                    break  # line went idle after some bytes: reply complete
                continue
            reply += os.read(self.fd, 256)
        if len(reply) < 3:
            return None
        payload, received = reply[:-2], struct.unpack(">H", reply[-2:])[0]
        if crc16(bytes([address, command]) + payload) != received:
            return "crc"
        return payload


def show(port, address):
    fw = port.read_command(address, FIRMWARE)
    if fw is None:
        return False
    if fw == "crc":
        print(f"address {address}: replied with a bad CRC (baud mismatch or two units "
              "with the same address?)")
        return True
    print(f"address {address}: {fw.rstrip(bytes([0, 10])).decode(errors='replace')}")

    def field(command, fmt, render):
        data = port.read_command(address, command)
        if data in (None, "crc") or len(data) < struct.calcsize(fmt):
            return "no/bad reply"
        return render(*struct.unpack(fmt, data[:struct.calcsize(fmt)]))

    print("  main battery  " + field(MAIN_BATTERY, ">H", lambda v: f"{v / 10:.1f} V"))
    print("  temperature   " + field(TEMPERATURE, ">H", lambda v: f"{v / 10:.1f} C"))
    print("  currents      " + field(CURRENTS, ">hh",
                                     lambda a, b: f"M1 {a / 100:.2f} A, M2 {b / 100:.2f} A"))
    print("  encoders      " + field(ENCODERS, ">ii", lambda a, b: f"M1 {a}, M2 {b}"))
    status = port.read_command(address, STATUS)
    if status not in (None, "crc"):
        print(f"  status        0x{int.from_bytes(status, 'big'):0{len(status) * 2}X} "
              "(bit meanings: RoboClaw manual, command 90)")
    return True


def watch(port, addresses):
    print("address  M1 enc      M2 enc      M1 A    M2 A   (Ctrl-C to stop)")
    while True:
        for address in addresses:
            enc = port.read_command(address, ENCODERS)
            cur = port.read_command(address, CURRENTS)
            if enc in (None, "crc") or cur in (None, "crc"):
                print(f"{address:<8} no/bad reply")
                continue
            m1, m2 = struct.unpack(">ii", enc[:8])
            c1, c2 = struct.unpack(">hh", cur[:4])
            print(f"{address:<8} {m1:<11} {m2:<11} {c1 / 100:<7.2f} {c2 / 100:.2f}")
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/roboclaw")
    parser.add_argument("--baud", type=int, default=38400,
                        help="must match Motion Studio's packet serial baud (default 38400)")
    parser.add_argument("--addresses", type=int, nargs="+", default=list(range(128, 136)))
    parser.add_argument("--watch", action="store_true",
                        help="stream encoder counts and currents (turn joints by hand)")
    args = parser.parse_args()

    port = Port(args.port, args.baud)
    try:
        if args.watch:
            watch(port, args.addresses)
        found = [a for a in args.addresses if show(port, a)]
        if not found:
            print(f"no RoboClaw answered on {args.port} @ {args.baud}. Check baud, wiring "
                  "(TX->S1, S2->RX, common GND), packet-serial mode, multi-unit mode.")
            return 1
        print(f"\nanswering addresses: {found}")
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
