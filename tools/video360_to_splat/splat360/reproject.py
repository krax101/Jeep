"""Stage 2: reproject equirectangular frames into a rig of pinhole views.

SfM and Gaussian splatting both want pinhole images, so each 360 frame is
resampled into several overlapping perspective views (a "virtual rig").
World frame: x right, y up, z forward at yaw=0. Camera looks down +z,
image u right / v down. Yaw is around +y (positive = look left->right
eastward), pitch around +x (positive = look up).
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class RigView:
    yaw_deg: float
    pitch_deg: float

    @property
    def name(self) -> str:
        return f"y{int(round(self.yaw_deg)) % 360:03d}_p{int(round(self.pitch_deg)):+03d}"


def default_rig(yaw_step: float = 45.0, pitches: tuple[float, ...] = (-30.0, 0.0, 30.0)) -> list[RigView]:
    """Ring of yaw headings at each pitch. With 90-degree FOV and 45-degree
    yaw spacing, adjacent views overlap by half a frame — plenty for SfM.
    The nadir (tripod / operator's hand) and zenith are left out on purpose.
    """
    yaws = np.arange(0.0, 360.0, yaw_step)
    return [RigView(float(y), float(p)) for p in pitches for y in yaws]


def rotation(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    y, p = np.radians(yaw_deg), np.radians(pitch_deg)
    ry = np.array([
        [np.cos(y), 0, np.sin(y)],
        [0, 1, 0],
        [-np.sin(y), 0, np.cos(y)],
    ])
    rx = np.array([
        [1, 0, 0],
        [0, np.cos(p), np.sin(p)],
        [0, -np.sin(p), np.cos(p)],
    ])
    return ry @ rx


def pinhole_intrinsics(size: int, fov_deg: float) -> tuple[float, float, float]:
    """Return (f, cx, cy) for a square view of `size` px and horizontal FOV."""
    f = 0.5 * size / np.tan(np.radians(fov_deg) / 2)
    c = (size - 1) / 2
    return f, c, c


def build_remap(eq_w: int, eq_h: int, size: int, fov_deg: float, view: RigView) -> tuple[np.ndarray, np.ndarray]:
    """Precompute cv2.remap maps from a pinhole view into the equirect image.

    The maps depend only on geometry, not pixel data, so they are computed
    once per view and reused for every frame.
    """
    f, cx, cy = pinhole_intrinsics(size, fov_deg)
    u, v = np.meshgrid(np.arange(size, dtype=np.float64), np.arange(size, dtype=np.float64))

    dirs = np.stack([(u - cx) / f, -(v - cy) / f, np.ones_like(u)], axis=-1)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    dirs = dirs @ rotation(view.yaw_deg, view.pitch_deg).T

    lon = np.arctan2(dirs[..., 0], dirs[..., 2])          # (-pi, pi]
    lat = np.arcsin(np.clip(dirs[..., 1], -1.0, 1.0))     # [-pi/2, pi/2]

    map_x = (lon / (2 * np.pi) + 0.5) * eq_w - 0.5
    map_y = (0.5 - lat / np.pi) * eq_h - 0.5
    return map_x.astype(np.float32), map_y.astype(np.float32)


def render_views(
    frames: list[Path],
    out_dir: Path,
    size: int = 1200,
    fov_deg: float = 90.0,
    rig: list[RigView] | None = None,
    jpeg_quality: int = 95,
) -> list[Path]:
    """Render every rig view of every frame into out_dir.

    File names are frame-major (f00001_y000_p+00.jpg, f00001_y045_p+00.jpg, ...)
    so that COLMAP's sequential matcher sees rig-neighbours and
    temporal-neighbours as name-neighbours.
    """
    rig = rig or default_rig()
    out_dir.mkdir(parents=True, exist_ok=True)

    first = cv2.imread(str(frames[0]))
    if first is None:
        raise RuntimeError(f"Cannot read frame {frames[0]}")
    eq_h, eq_w = first.shape[:2]

    maps = {view.name: build_remap(eq_w, eq_h, size, fov_deg, view) for view in rig}

    written: list[Path] = []
    for i, frame_path in enumerate(frames, start=1):
        eq = first if frame_path == frames[0] else cv2.imread(str(frame_path))
        if eq is None:
            print(f"WARNING: skipping unreadable frame {frame_path}")
            continue
        if eq.shape[:2] != (eq_h, eq_w):
            raise RuntimeError(f"Frame {frame_path} has different resolution than the first frame")
        for view in rig:
            map_x, map_y = maps[view.name]
            persp = cv2.remap(eq, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
            out = out_dir / f"f{i:05d}_{view.name}.jpg"
            cv2.imwrite(str(out), persp, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            written.append(out)
        if i % 10 == 0 or i == len(frames):
            print(f"  rendered {i}/{len(frames)} frames ({len(written)} views)")
    return written
