"""Printable ArUco markers at an exact physical size.

    ros2 run arm_vision aruco_generate --ids 0 1 2 3             # DICT_5X5_50, 50 mm, ./markers
    ros2 run arm_vision aruco_generate --dictionary DICT_4X4_50 --ids 7 --size-mm 100 --dpi 600

One PNG per id, with the DPI stored in the file so it prints at size: print at 100% / "actual
size", never "fit to page", then measure the black square and set aruco_node's marker_size to it
(metres). The white quiet zone around the square is part of the marker; don't cut into it.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import zlib

import cv2

from arm_vision.aruco import DICTIONARIES, generate_marker


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dictionary", default="DICT_5X5_50", choices=sorted(DICTIONARIES))
    parser.add_argument("--ids", type=int, nargs="+", default=[0])
    parser.add_argument("--size-mm", type=float, default=50.0,
                        help="Side of the black square, mm (= aruco_node marker_size x 1000).")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--out", default="markers", help="Output directory.")
    args = parser.parse_args(argv)

    side_px = round(args.size_mm / 25.4 * args.dpi)
    os.makedirs(args.out, exist_ok=True)
    for marker_id in args.ids:
        try:
            image = generate_marker(args.dictionary, marker_id, side_px)
        except ValueError as e:
            sys.exit(str(e))
        path = os.path.join(args.out, f"{args.dictionary}_{marker_id}_{args.size_mm:g}mm.png")
        # PNG pHYs in pixels per metre, so printers and viewers know the physical size
        ok, png = cv2.imencode(".png", image)
        if not ok:
            sys.exit(f"could not encode {path}")
        with open(path, "wb") as f:
            f.write(with_dpi(png.tobytes(), args.dpi))
        print(f"{path}: {args.size_mm:g} mm square at {args.dpi} dpi ({side_px} px)")


def with_dpi(png: bytes, dpi: int) -> bytes:
    """Insert a pHYs chunk (pixels per metre) after IHDR."""
    ppm = round(dpi / 0.0254)
    data = struct.pack(">IIB", ppm, ppm, 1)
    chunk = struct.pack(">I", len(data)) + b"pHYs" + data
    chunk += struct.pack(">I", zlib.crc32(b"pHYs" + data) & 0xFFFFFFFF)
    ihdr_end = 8 + 4 + 4 + 13 + 4  # signature, length, type, IHDR data, CRC
    return png[:ihdr_end] + chunk + png[ihdr_end:]


if __name__ == "__main__":
    main()
