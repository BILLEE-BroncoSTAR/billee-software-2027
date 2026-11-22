#!/usr/bin/env python3
"""Minimal CANopen (CiA301 / CiA402) P17 / P20 drive emulator for SocketCAN vcan.

Answers expedited SDOs, follows NMT and the CiA402 control-word state machine,
and on every SYNC applies the received RPDOs and transmits the TPDOs. PDOs are
packed / unpacked from whatever mapping the host configured (16xxh / 1Axxh), so
a wrong mapping shows up as wrong data rather than being silently papered over.

In cyclic synchronous position mode (6060h = 8) with operation enabled, the
emulated motor moves toward 607Ah at most --max-speed motor revolutions per
second. Send SIGUSR1 to inject a "position error too large" fault (EMCY 8611h)
on every node; a fault-reset control word (0x80) clears it.
"""

import argparse
import signal
import socket
import struct
import time

# COB-ID function codes.
NMT, SYNC_EMCY, TPDO1, RPDO1, TSDO, RSDO = 0x000, 0x080, 0x180, 0x200, 0x580, 0x600
FUNCTION_MASK, NODE_MASK = 0x780, 0x07F
PDO_DISABLED = 0x80000000
PDO_COUNT = 4

# NMT states (as reported in the heartbeat) and commands.
NMT_OPERATIONAL, NMT_STOPPED, NMT_PRE_OPERATIONAL = 0x05, 0x04, 0x7F
NMT_COMMANDS = {
    0x01: NMT_OPERATIONAL,
    0x02: NMT_STOPPED,
    0x80: NMT_PRE_OPERATIONAL,
    0x81: NMT_PRE_OPERATIONAL,  # reset node
    0x82: NMT_PRE_OPERATIONAL,  # reset communication
}

# SDO abort codes.
ABORT_NOT_EXIST = 0x06020000
ABORT_WRITE_NOT_ALLOWED = 0x06010002
ABORT_DEVICE_STATE = 0x08000022
ABORT_COMMAND = 0x05040001

# CiA402 status words (bits 0-6) per state; bit 5 (quick stop) is 1 when not active.
SWITCH_ON_DISABLED, READY, SWITCHED_ON, ENABLED, FAULT = 0x0040, 0x0021, 0x0023, 0x0027, 0x0008
TARGET_REACHED = 1 << 10
MODE_CSP = 8

COUNTS_PER_REV = 65536  # per motor-shaft rev; matches pxx_can::kPositionCountsPerRev
TARGET_WINDOW_COUNTS = 100
FAULT_CODE = 0x8611  # position error too large

CONTROL_WORD, STATUS_WORD, MODE, ERROR_CODE = 0x6040, 0x6041, 0x6060, 0x603F
POSITION, VELOCITY, TARGET = 0x6064, 0x606C, 0x607A
READ_ONLY = {STATUS_WORD, ERROR_CODE, POSITION, VELOCITY}

CAN_FRAME = struct.Struct("=IB3x8s")


def to_signed(value, bits):
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


class Drive:
    def __init__(self, node_id, initial_counts, max_speed_counts):
        self.node_id = node_id
        self.max_speed_counts = max_speed_counts
        self.nmt = NMT_PRE_OPERATIONAL
        self.state = SWITCH_ON_DISABLED
        self.control_word = 0
        self.position = initial_counts
        self.velocity = 0
        self.target = initial_counts
        self.error_code = 0
        self.pending_rpdos = {}
        # Object dictionary: (index, subindex) -> (value, size in bytes).
        self.od = {(MODE, 0): (0, 1), (TARGET, 0): (initial_counts & 0xFFFFFFFF, 4)}
        for pdo in range(PDO_COUNT):
            # Typical factory defaults: RPDO1 -> control word, TPDO1 -> status word,
            # everything else disabled. The driver is expected to remap these.
            rpdo_enabled = pdo == 0
            self.od[(0x1400 + pdo, 1)] = (
                (RPDO1 + pdo * 0x100 + node_id) | (0 if rpdo_enabled else PDO_DISABLED), 4)
            self.od[(0x1400 + pdo, 2)] = (0xFF, 1)
            self.od[(0x1600 + pdo, 0)] = (1 if rpdo_enabled else 0, 1)
            self.od[(0x1600 + pdo, 1)] = ((CONTROL_WORD << 16) | 16 if rpdo_enabled else 0, 4)
            tpdo_enabled = pdo == 0
            self.od[(0x1800 + pdo, 1)] = (
                (TPDO1 + pdo * 0x100 + node_id) | (0 if tpdo_enabled else PDO_DISABLED), 4)
            self.od[(0x1800 + pdo, 2)] = (0xFF, 1)
            self.od[(0x1A00 + pdo, 0)] = (1 if tpdo_enabled else 0, 1)
            self.od[(0x1A00 + pdo, 1)] = ((STATUS_WORD << 16) | 16 if tpdo_enabled else 0, 4)

    # ---- object dictionary ----

    def status_word(self):
        status = self.state
        if self.state == ENABLED and abs(self.target - self.position) <= TARGET_WINDOW_COUNTS:
            status |= TARGET_REACHED
        return status

    def read(self, index, subindex):
        live = {
            (CONTROL_WORD, 0): (self.control_word, 2),
            (STATUS_WORD, 0): (self.status_word(), 2),
            (ERROR_CODE, 0): (self.error_code, 2),
            (POSITION, 0): (self.position & 0xFFFFFFFF, 4),
            (VELOCITY, 0): (self.velocity & 0xFFFFFFFF, 4),
            (TARGET, 0): (self.target & 0xFFFFFFFF, 4),
        }
        if (index, subindex) in live:
            return live[(index, subindex)]
        return self.od.get((index, subindex))

    def write(self, index, subindex, value, size):
        """Returns an SDO abort code, or None on success."""
        if index in READ_ONLY:
            return ABORT_WRITE_NOT_ALLOWED
        is_pdo_object = 0x1400 <= index < 0x1400 + PDO_COUNT or 0x1600 <= index < 0x1600 + PDO_COUNT \
            or 0x1800 <= index < 0x1800 + PDO_COUNT or 0x1A00 <= index < 0x1A00 + PDO_COUNT
        if is_pdo_object and self.nmt == NMT_OPERATIONAL:
            return ABORT_DEVICE_STATE
        is_mapping = 0x1600 <= index < 0x1600 + PDO_COUNT or 0x1A00 <= index < 0x1A00 + PDO_COUNT
        if is_mapping and subindex > 0 and self.od.get((index, 0), (0, 1))[0] != 0:
            return ABORT_DEVICE_STATE  # CiA301: clear sub0 before changing entries
        if index == CONTROL_WORD and subindex == 0:
            self.apply_control_word(value)
        elif index == TARGET and subindex == 0:
            self.target = to_signed(value, 32)
        elif (index, subindex) not in self.od and not (is_mapping and 1 <= subindex <= 8):
            return ABORT_NOT_EXIST
        self.od[(index, subindex)] = (value, size)
        return None

    def mapping(self, mapping_index):
        count = self.od[(mapping_index, 0)][0]
        entries = []
        for sub in range(1, count + 1):
            entry = self.od.get((mapping_index, sub), (0, 4))[0]
            entries.append((entry >> 16, (entry >> 8) & 0xFF, entry & 0xFF))
        return entries

    # ---- CiA402 ----

    def apply_control_word(self, word):
        previous = self.control_word
        self.control_word = word
        if self.state == FAULT:
            if word & 0x80 and not previous & 0x80:
                self.state = SWITCH_ON_DISABLED
                self.error_code = 0
                print(f"node {self.node_id}: fault reset", flush=True)
            return
        command = word & 0x0F
        if not word & 0x02:  # disable voltage
            self.state = SWITCH_ON_DISABLED
        elif command & 0x07 == 0x06:  # shutdown
            self.state = READY
        elif command == 0x07 and self.state in (READY, ENABLED):  # switch on / disable op
            self.state = SWITCHED_ON
        elif command == 0x0F and self.state in (READY, SWITCHED_ON):  # enable operation
            self.state = ENABLED
            print(f"node {self.node_id}: operation enabled at {self.position} counts, "
                  f"target {self.target}", flush=True)
            if self.od[(MODE, 0)][0] != MODE_CSP:
                print(f"node {self.node_id}: WARNING mode of operation is "
                      f"{self.od[(MODE, 0)][0]}, not CSP ({MODE_CSP})", flush=True)

    def fault(self, code):
        self.state = FAULT
        self.error_code = code
        self.velocity = 0

    # ---- cyclic ----

    def on_sync(self, dt):
        for pdo, payload in self.pending_rpdos.items():
            offset = 0
            for index, subindex, bits in self.mapping(0x1600 + pdo):
                nbytes = bits // 8
                value = int.from_bytes(payload[offset:offset + nbytes], "little")
                self.write(index, subindex, value, nbytes)
                offset += nbytes
        self.pending_rpdos.clear()

        previous = self.position
        if self.state == ENABLED and self.od[(MODE, 0)][0] == MODE_CSP:
            step = int(self.max_speed_counts * dt)
            error = self.target - self.position
            self.position += max(-step, min(step, error))
        self.velocity = int((self.position - previous) / dt) if dt > 0 else 0

    def tpdo_frames(self):
        frames = []
        for pdo in range(PDO_COUNT):
            cob_id = self.od[(0x1800 + pdo, 1)][0]
            if cob_id & PDO_DISABLED:
                continue
            payload = b""
            for index, subindex, bits in self.mapping(0x1A00 + pdo):
                entry = self.read(index, subindex)
                value = entry[0] if entry else 0
                payload += (value & ((1 << bits) - 1)).to_bytes(bits // 8, "little")
            frames.append((cob_id & 0x7FF, payload[:8]))
        return frames


def sdo_reply(drive, data):
    command = data[0]
    index = data[1] | (data[2] << 8)
    subindex = data[3]
    header = data[1:4]
    if command == 0x40:  # upload (read)
        entry = drive.read(index, subindex)
        if entry is None:
            return bytes([0x80]) + header + ABORT_NOT_EXIST.to_bytes(4, "little")
        value, size = entry
        return bytes([0x43 | ((4 - size) << 2)]) + header + \
            (value & 0xFFFFFFFF).to_bytes(4, "little")
    if command & 0xE3 == 0x23:  # expedited download (write) with size indicated
        size = 4 - ((command >> 2) & 0x03)
        value = int.from_bytes(data[4:4 + size], "little")
        abort = drive.write(index, subindex, value, size)
        if abort is not None:
            print(f"node {drive.node_id}: SDO write {index:04X}h:{subindex:02X} "
                  f"= 0x{value:X} aborted 0x{abort:08X}", flush=True)
            return bytes([0x80]) + header + abort.to_bytes(4, "little")
        return bytes([0x60]) + header + bytes(4)
    return bytes([0x80]) + header + ABORT_COMMAND.to_bytes(4, "little")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="vcan0")
    parser.add_argument("--nodes", type=int, nargs="+", default=[1, 2],
                        help="CANopen node IDs to emulate (default: 1 2, see "
                             "arm_bringup/config/arm_axes.yaml)")
    parser.add_argument("--initial-counts", type=int, default=0,
                        help="starting 6064h position of every node, in encoder counts")
    parser.add_argument("--max-speed", type=float, default=30.0,
                        help="CSP slew limit in motor-shaft revolutions per second "
                             "(default 30 ~= 0.25 output rev/s through a 121:1 reducer)")
    args = parser.parse_args()

    max_speed_counts = args.max_speed * COUNTS_PER_REV
    drives = {node: Drive(node, args.initial_counts, max_speed_counts) for node in args.nodes}

    can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_socket.bind((args.interface,))
    can_socket.settimeout(0.1)

    def send(cob_id, payload):
        can_socket.send(CAN_FRAME.pack(cob_id, len(payload), payload.ljust(8, b"\0")))

    running = True
    fault_requested = False

    def stop(*_unused):
        nonlocal running
        running = False

    def request_fault(*_unused):
        nonlocal fault_requested
        fault_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGUSR1, request_fault)

    print(f"pxx vCAN emulator listening on {args.interface} (nodes {args.nodes})", flush=True)
    last_sync = None

    while running:
        if fault_requested:
            fault_requested = False
            for drive in drives.values():
                drive.fault(FAULT_CODE)
                send(SYNC_EMCY + drive.node_id,
                     FAULT_CODE.to_bytes(2, "little") + bytes([0x01]) + bytes(5))
                print(f"node {drive.node_id}: injected fault 0x{FAULT_CODE:04X}", flush=True)

        try:
            raw = can_socket.recv(CAN_FRAME.size)
        except (socket.timeout, InterruptedError):
            continue
        if len(raw) < CAN_FRAME.size:
            continue
        can_id, length, data = CAN_FRAME.unpack(raw)
        can_id &= 0x7FF
        data = data[:length]
        function, node_id = can_id & FUNCTION_MASK, can_id & NODE_MASK

        if can_id == NMT and length >= 2:
            state = NMT_COMMANDS.get(data[0])
            for drive in drives.values():
                if state is not None and data[1] in (0, drive.node_id):
                    drive.nmt = state
        elif can_id == SYNC_EMCY:
            now = time.monotonic()
            dt = min(now - last_sync, 0.1) if last_sync is not None else 0.0
            last_sync = now
            for drive in drives.values():
                if drive.nmt != NMT_OPERATIONAL:
                    continue
                drive.on_sync(dt)
                for cob_id, payload in drive.tpdo_frames():
                    send(cob_id, payload)
        elif function == RSDO and node_id in drives and length == 8:
            send(TSDO + node_id, sdo_reply(drives[node_id], data))
        elif node_id in drives and RPDO1 <= function <= RPDO1 + 0x300 and function & 0x80 == 0:
            drive = drives[node_id]
            pdo = (function - RPDO1) >> 8
            cob_id = drive.od[(0x1400 + pdo, 1)][0]
            if drive.nmt == NMT_OPERATIONAL and not cob_id & PDO_DISABLED and \
                    (cob_id & 0x7FF) == can_id:
                drive.pending_rpdos[pdo] = data

    can_socket.close()


if __name__ == "__main__":
    main()
