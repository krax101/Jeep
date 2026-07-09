"""Geometry self-test: `python -m splat360.selftest` (no video needed).

Builds a synthetic equirectangular image with known-colour markers at
known directions and checks that each reprojected pinhole view centres
on the right marker.
"""

import sys

import cv2
import numpy as np

from .reproject import RigView, build_remap, pinhole_intrinsics


EQ_W, EQ_H = 2048, 1024
SIZE, FOV = 400, 90.0

# (yaw, pitch) -> marker colour (BGR)
MARKERS = {
    (0, 0): (0, 0, 255),      # forward: red
    (90, 0): (0, 255, 0),     # right: green
    (180, 0): (255, 0, 0),    # behind: blue
    (270, 0): (0, 255, 255),  # left: yellow
    (0, 60): (255, 0, 255),   # up-forward: magenta
    (0, -60): (255, 255, 0),  # down-forward: cyan
}


def make_equirect() -> np.ndarray:
    eq = np.full((EQ_H, EQ_W, 3), 32, np.uint8)
    for (yaw, pitch), color in MARKERS.items():
        lon = np.radians(((yaw + 180) % 360) - 180)  # to (-pi, pi]
        lat = np.radians(pitch)
        px = int((lon / (2 * np.pi) + 0.5) * EQ_W)
        py = int((0.5 - lat / np.pi) * EQ_H)
        cv2.circle(eq, (px, py), 40, color, -1)
    return eq


def main() -> int:
    eq = make_equirect()
    failures = 0
    for (yaw, pitch), expected in MARKERS.items():
        map_x, map_y = build_remap(EQ_W, EQ_H, SIZE, FOV, RigView(yaw, pitch))
        persp = cv2.remap(eq, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        got = tuple(int(c) for c in persp[SIZE // 2, SIZE // 2])
        ok = got == expected
        failures += not ok
        print(f"  view yaw={yaw:>3} pitch={pitch:>3}: centre={got} expected={expected} "
              f"{'OK' if ok else 'FAIL'}")

    # Antipodal seam: the yaw=180 view crosses the equirect wrap; its centre
    # column must be continuous (no seam artefact from BORDER_WRAP).
    map_x, map_y = build_remap(EQ_W, EQ_H, SIZE, FOV, RigView(180, 0))
    persp = cv2.remap(eq, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
    col = persp[:, SIZE // 2].astype(int)
    max_jump = int(np.abs(np.diff(col, axis=0)).max())
    seam_ok = max_jump < 250  # only the marker edge itself may jump
    print(f"  wrap seam continuity at yaw=180: max column jump {max_jump} "
          f"{'OK' if seam_ok else 'FAIL'}")
    failures += not seam_ok

    f, cx, cy = pinhole_intrinsics(SIZE, FOV)
    f_ok = abs(f - SIZE / 2) < 1e-6  # 90 deg FOV -> f = size/2
    print(f"  intrinsics: f={f:.2f} cx={cx} cy={cy} {'OK' if f_ok else 'FAIL'}")
    failures += not f_ok

    print("PASS" if failures == 0 else f"FAIL ({failures})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
