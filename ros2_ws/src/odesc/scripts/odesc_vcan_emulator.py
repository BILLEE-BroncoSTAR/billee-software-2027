#!/usr/bin/env python3
"""Minimal six-node ODrive CANSimple encoder emulator for SocketCAN vcan."""

import argparse
import math
import signal
import socket
import struct
import time


SET_INPUT_VEL = 0x0D
GET_ENCODER_ESTIMATES = 0x09
NODE_COUNT = 6
UPDATE_HZ = 100.0


def arbitration_id(node_id, command_id):
    return (node_id << 5) | command_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="vcan0")
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

    positions = [0.0] * NODE_COUNT  # motor-shaft turns
    velocities = [0.0] * NODE_COUNT  # motor-shaft turns / s
    previous = time.monotonic()
    interval = 1.0 / UPDATE_HZ
    next_publish = previous
    print(f"ODESC vCAN emulator listening on {args.interface} ({NODE_COUNT} nodes)", flush=True)

    while running:
        now = time.monotonic()
        dt = now - previous
        previous = now
        for node_id in range(NODE_COUNT):
            positions[node_id] += velocities[node_id] * dt

        while True:
            try:
                frame = can_socket.recv(16)
            except BlockingIOError:
                break
            if len(frame) < 16:
                continue
            can_id, length, payload = struct.unpack("=IB3x8s", frame)
            node_id, command_id = (can_id & 0x7FF) >> 5, (can_id & 0x1F)
            if node_id < NODE_COUNT and command_id == SET_INPUT_VEL and length >= 4:
                velocities[node_id] = struct.unpack("<f", payload[:4])[0]

        if now >= next_publish:
            for node_id in range(NODE_COUNT):
                frame = struct.pack(
                    "=IB3x8s",
                    arbitration_id(node_id, GET_ENCODER_ESTIMATES),
                    8,
                    struct.pack("<ff", positions[node_id], velocities[node_id]),
                )
                can_socket.send(frame)
            next_publish = now + interval
        time.sleep(0.001)

    can_socket.close()


if __name__ == "__main__":
    main()
