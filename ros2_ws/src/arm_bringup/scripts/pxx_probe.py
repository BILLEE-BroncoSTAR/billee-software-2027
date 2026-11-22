#!/usr/bin/env python3
"""Check the P17 / P20 CANopen bus: find drives, read their state, decode traffic.

    pxx_probe.py scan    [--interface can1] [--nodes 1 2]   # who is on the bus + state
    pxx_probe.py monitor [--interface can1]                 # decode every frame live
    pxx_probe.py rates   [--interface can1]                 # per-node frames/s, once a second

`scan` sends SDO reads, so run it with arm.launch.py STOPPED (its SDO replies
would race the driver's). `monitor` and `rates` only listen and are safe to run
while the arm is up. No dependencies beyond the Python standard library.
Frame layouts are the ones pxx configures (see docs/ARM_BRINGUP_GUIDE.md §3.1).
"""

import argparse
import collections
import socket
import struct
import sys
import time

CAN_FRAME = struct.Struct("=IB3x8s")
NMT_COMMANDS = {0x01: "start", 0x02: "stop", 0x80: "pre-operational",
                0x81: "reset node", 0x82: "reset communication"}
NMT_STATES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}
ERRORS = {0x0000: "no error", 0x3230: "overload", 0x4210: "over-temperature",
          0x7121: "stalled", 0x7310: "overspeed", 0x8130: "heartbeat offline",
          0x8500: "speed error too large", 0x8611: "position error too large"}
COUNTS_PER_MOTOR_REV = 65536


def open_bus(interface):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((interface,))
    except OSError as e:
        sys.exit(f"cannot open {interface}: {e} (bring it up with tooling/can-up)")
    return sock


def send(sock, can_id, data):
    sock.send(CAN_FRAME.pack(can_id, len(data), bytes(data).ljust(8, b"\0")))


def recv(sock, timeout):
    sock.settimeout(timeout)
    try:
        can_id, length, data = CAN_FRAME.unpack(sock.recv(CAN_FRAME.size))
    except socket.timeout:
        return None
    return can_id & 0x7FF, data[:length]


def sdo_read(sock, node, index, sub=0, timeout=0.05):
    send(sock, 0x600 + node, [0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        frame = recv(sock, max(end - time.monotonic(), 0.001))
        if not frame:
            break
        can_id, data = frame
        if can_id == 0x580 + node and len(data) == 8 and \
                (data[1] | data[2] << 8, data[3]) == (index, sub):
            value = int.from_bytes(data[4:8], "little")
            return ("abort", value) if data[0] == 0x80 else ("ok", value)
    return ("timeout", None)


def describe_status(sw):
    if sw & 0x08:
        return "FAULT"
    if sw & 0x04:
        return "operation enabled"
    if sw & 0x02:
        return "switched on"
    if sw & 0x01:
        return "ready to switch on"
    if sw & 0x40:
        return "switch on disabled"
    return "not ready"


def signed(v, bits=32):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


def cmd_scan(sock, nodes):
    found = []
    for node in nodes:
        if sdo_read(sock, node, 0x1000)[0] != "timeout":
            found.append(node)
    if not found:
        print("no drives answered. Check power, bitrate, termination, node IDs "
              "(candump while powering on shows 7XX#00 boot-up).")
        return 1
    print(f"drives on the bus: {found}\n")
    for node in found:
        print(f"node {node}:")
        for label, index, sub, fmt in (
                ("status word 6041h", 0x6041, 0, "status"),
                ("error code  603Fh", 0x603F, 0, "error"),
                ("mode        6060h", 0x6060, 0, "int8"),
                ("position    6064h", 0x6064, 0, "position"),
                ("velocity    606Ch", 0x606C, 0, "int32"),
                ("gear ratio  6091h:01", 0x6091, 1, "uint"),
                ("gear ratio  6091h:02", 0x6091, 2, "uint"),
                ("CAN baud    2001h:02", 0x2001, 2, "uint")):
            state, value = sdo_read(sock, node, index, sub)
            if state == "timeout":
                text = "no answer"
            elif state == "abort":
                text = f"abort 0x{value:08X}"
            elif fmt == "status":
                text = f"0x{value & 0xFFFF:04X} ({describe_status(value)})"
            elif fmt == "error":
                text = f"0x{value & 0xFFFF:04X} ({ERRORS.get(value & 0xFFFF, 'unknown')})"
            elif fmt == "int8":
                text = str(signed(value & 0xFF, 8))
            elif fmt == "position":
                counts = signed(value)
                text = f"{counts} counts = {counts / COUNTS_PER_MOTOR_REV:+.3f} motor rev"
            elif fmt == "int32":
                text = str(signed(value))
            else:
                text = str(value)
            print(f"  {label:<22} {text}")
        print()
    return 0


def decode(can_id, data):
    function, node = can_id & 0x780, can_id & 0x7F
    if can_id == 0x000 and len(data) >= 2:
        target = "all" if data[1] == 0 else f"node {data[1]}"
        return f"NMT       {NMT_COMMANDS.get(data[0], hex(data[0]))} -> {target}"
    if can_id == 0x080:
        return "SYNC"
    if function == 0x080 and len(data) >= 2:
        code = int.from_bytes(data[0:2], "little")
        return f"EMCY      node {node}: 0x{code:04X} {ERRORS.get(code, 'unknown')}"
    if function == 0x180 and len(data) >= 8:
        pos, vel = struct.unpack("<ii", bytes(data[:8]))
        return (f"TPDO1     node {node}: position {pos} ({pos / COUNTS_PER_MOTOR_REV:+.3f} "
                f"motor rev), velocity {vel}")
    if function == 0x280 and len(data) >= 2:
        sw = int.from_bytes(data[0:2], "little")
        return f"TPDO2     node {node}: status 0x{sw:04X} ({describe_status(sw)})"
    if function == 0x200 and len(data) >= 4:
        target = struct.unpack("<i", bytes(data[:4]))[0]
        return f"RPDO1     node {node}: target {target} ({target / COUNTS_PER_MOTOR_REV:+.3f} motor rev)"
    if function in (0x580, 0x600) and len(data) == 8:
        index, sub = data[1] | data[2] << 8, data[3]
        value = int.from_bytes(data[4:8], "little")
        kind = {0x40: "read", 0x60: "write ok", 0x80: "ABORT"}.get(data[0])
        if kind is None:
            kind = "write" if function == 0x600 else "read reply"
        arrow = "->" if function == 0x600 else "<-"
        return f"SDO {arrow}    node {node}: {kind} {index:04X}h:{sub:02X} value 0x{value:08X}"
    if function == 0x700 and data:
        return f"HEARTBEAT node {node}: {NMT_STATES.get(data[0] & 0x7F, hex(data[0]))}"
    return f"?         id 0x{can_id:03X} [{len(data)}] {bytes(data).hex(' ')}"


def cmd_monitor(sock):
    start = time.monotonic()
    while True:
        frame = recv(sock, 1.0)
        if frame:
            print(f"{time.monotonic() - start:9.4f}  {frame[0]:03X}  {decode(*frame)}", flush=True)


def cmd_rates(sock):
    counts = collections.Counter()
    last = time.monotonic()
    while True:
        frame = recv(sock, 0.1)
        if frame:
            can_id = frame[0]
            name = {0x180: "TPDO1", 0x280: "TPDO2", 0x200: "RPDO1", 0x080: "EMCY"}.get(
                can_id & 0x780)
            if can_id == 0x080:
                counts["SYNC"] += 1
            elif name:
                counts[f"{name} n{can_id & 0x7F}"] += 1
        if time.monotonic() - last >= 1.0:
            last = time.monotonic()
            line = "  ".join(f"{k}:{v}" for k, v in sorted(counts.items())) or "(silent bus)"
            print(line, flush=True)
            counts.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["scan", "monitor", "rates"])
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--nodes", type=int, nargs="+", default=list(range(1, 128)),
                        help="node IDs to scan (default: all 1-127)")
    args = parser.parse_args()
    sock = open_bus(args.interface)
    try:
        if args.command == "scan":
            return cmd_scan(sock, args.nodes)
        if args.command == "monitor":
            cmd_monitor(sock)
        else:
            cmd_rates(sock)
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
