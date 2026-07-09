#!/usr/bin/env python3
"""Write a tiny synthetic .splat scene for sanity-checking the viewer.

Layout (world: x right, y down, z forward; default camera at z=-5 looks +z):
  red    at ( 0,  0, 5)  dead centre
  green  at ( 0,  0, 8)  directly BEHIND red (occlusion check)
  blue   at ( 2,  0, 5)  screen right
  yellow at ( 0, -2, 5)  screen up
  white  at ( 0,  2, 5)  screen down

Usage: python make_test_splat.py [out.splat]
"""

import struct
import sys


def splat(x, y, z, scale, rgba, quat=(1, 0, 0, 0)):
    qb = [max(0, min(255, round(c * 128 + 128))) for c in quat]
    return struct.pack("<3f3f4B4B", x, y, z, scale, scale, scale, *rgba, *qb)


SCENE = [
    splat(0, 0, 5, 0.4, (255, 0, 0, 255)),
    splat(0, 0, 8, 0.4, (0, 255, 0, 255)),
    splat(2, 0, 5, 0.4, (0, 0, 255, 255)),
    splat(0, -2, 5, 0.4, (255, 255, 0, 255)),
    splat(0, 2, 5, 0.4, (255, 255, 255, 255)),
]

def write_ply(path):
    """Same scene as a 3D-Gaussian-splatting .ply (what trainers output)."""
    import math

    SH_C0 = 0.28209479177387814
    points = [
        (0, 0, 5, (1, 0, 0)),
        (0, 0, 8, (0, 1, 0)),
        (2, 0, 5, (0, 0, 1)),
        (0, -2, 5, (1, 1, 0)),
        (0, 2, 5, (1, 1, 1)),
    ]
    props = ["x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2", "opacity",
             "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        + "".join(f"property float {p}\n" for p in props)
        + "end_header\n"
    )
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        for x, y, z, (r, g, b) in points:
            dc = [(c - 0.5) / SH_C0 for c in (r, g, b)]
            fh.write(struct.pack(
                "<14f", x, y, z, *dc,
                9.0,                              # sigmoid(9) ~ opacity 1
                *([math.log(0.4)] * 3),           # scales are stored as log
                1, 0, 0, 0,
            ))
    print(f"wrote {path}: {len(points)} splats")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "test.splat"
    with open(out, "wb") as fh:
        fh.write(b"".join(SCENE))
    print(f"wrote {out}: {len(SCENE)} splats")
    write_ply(out.rsplit(".", 1)[0] + ".ply")
