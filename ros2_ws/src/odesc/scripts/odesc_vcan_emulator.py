#!/usr/bin/env python3
"""ODESC (ODrive CANSimple) emulator for a SocketCAN bus such as vcan0.

Behaves like the rover's ODESC boards (pre-0.5.2 ODrive firmware):
  * 10 Hz heartbeat (0x01): axis error 0, axis state IDLE (1) until the driver
    sends Set_Axis_Requested_State (0x07), then whatever it requested (8 = closed loop).
  * Get_Encoder_Estimates (0x09) only in reply to an RTR request (--mode rtr, default),
    or broadcast at 100 Hz like newer firmware (--mode cyclic).
  * Set_Input_Vel (0x0D) sets the motor-shaft velocity; it only moves in closed loop.
Positions/velocities are motor-shaft turns and turns/s, as on the real bus.
"""

import argparse
import signal
import socket
import struct
import time

HEARTBEAT = 0x01
SET_AXIS_STATE = 0x07
GET_ENCODER_ESTIMATES = 0x09
SET_INPUT_VEL = 0x0D
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000

HEARTBEAT_HZ = 10.0
CYCLIC_ENCODER_HZ = 100.0
FRAME = "=IB3x8s"


def arbitration_id(node_id, command_id):
    return (node_id << 5) | command_id


def parse_nodes(text):
    nodes = [int(n) for n in text.split(",") if n.strip()]
    if not nodes or any(n < 0 or n > 63 for n in nodes):
        raise argparse.ArgumentTypeError("expected comma-separated node IDs in [0, 63]")
    return nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="vcan0")
    parser.add_argument("--nodes", type=parse_nodes, default=parse_nodes("2,3,4,5,6,7"),
                        help="node IDs to emulate (default: 2,3,4,5,6,7, as in node_map.yaml)")
    parser.add_argument("--mode", choices=["rtr", "cyclic"], default="rtr",
                        help="rtr: answer encoder requests (the rover's firmware); "
                             "cyclic: broadcast encoder estimates at 100 Hz")
    args = parser.parse_args()

    can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_socket.bind((args.interface,))
    can_socket.setblocking(False)

    running = True

    def stop(*_unused):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    positions = {n: 0.0 for n in args.nodes}    # motor-shaft turns
    velocities = {n: 0.0 for n in args.nodes}   # commanded motor-shaft turns / s
    states = {n: AXIS_STATE_IDLE for n in args.nodes}

    def send(node_id, command_id, payload):
        try:
            can_socket.send(struct.pack(FRAME, arbitration_id(node_id, command_id),
                                        len(payload), payload.ljust(8, b"\x00")))
        except (BlockingIOError, OSError):
            pass  # bus full / down: drop it, like a real node would

    def send_encoder(node_id):
        moving = states[node_id] == AXIS_STATE_CLOSED_LOOP
        send(node_id, GET_ENCODER_ESTIMATES,
             struct.pack("<ff", positions[node_id], velocities[node_id] if moving else 0.0))

    now = time.monotonic()
    previous = now
    next_heartbeat = now
    next_cyclic = now
    print(f"ODESC emulator on {args.interface}: nodes {args.nodes}, encoder mode {args.mode}",
          flush=True)

    while running:
        now = time.monotonic()
        dt = now - previous
        previous = now
        for node_id in args.nodes:
            if states[node_id] == AXIS_STATE_CLOSED_LOOP:
                positions[node_id] += velocities[node_id] * dt

        while True:
            try:
                frame = can_socket.recv(16)
            except BlockingIOError:
                break
            if len(frame) < 16:
                continue
            can_id, length, payload = struct.unpack(FRAME, frame)
            if can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG):
                continue
            node_id, command_id = (can_id & 0x7FF) >> 5, can_id & 0x1F
            if node_id not in states:
                continue
            if can_id & CAN_RTR_FLAG:
                if command_id == GET_ENCODER_ESTIMATES:
                    send_encoder(node_id)
            elif command_id == SET_INPUT_VEL and length >= 4:
                velocities[node_id] = struct.unpack("<f", payload[:4])[0]
            elif command_id == SET_AXIS_STATE and length >= 4:
                states[node_id] = struct.unpack("<I", payload[:4])[0]
                if states[node_id] != AXIS_STATE_CLOSED_LOOP:
                    velocities[node_id] = 0.0

        if now >= next_heartbeat:
            for node_id in args.nodes:
                send(node_id, HEARTBEAT, struct.pack("<II", 0, states[node_id]))
            next_heartbeat = now + 1.0 / HEARTBEAT_HZ

        if args.mode == "cyclic" and now >= next_cyclic:
            for node_id in args.nodes:
                send_encoder(node_id)
            next_cyclic = now + 1.0 / CYCLIC_ENCODER_HZ

        time.sleep(0.0005)

    can_socket.close()


if __name__ == "__main__":
    main()
