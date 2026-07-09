"""Stage 1: extract equirectangular frames from a 360 video with ffmpeg."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import cv2


class FfmpegNotFound(RuntimeError):
    pass


def _require_ffmpeg():
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        raise FfmpegNotFound(
            "ffmpeg/ffprobe not found on PATH. Install it first "
            "(e.g. `apt install ffmpeg` or `brew install ffmpeg`)."
        )


def probe_video(video: Path) -> dict:
    """Return {width, height, duration, fps} for the first video stream."""
    _require_ffmpeg()
    out = subprocess.run(
        [
            "ffprobe", "-v", "error",
            "-select_streams", "v:0",
            "-show_entries", "stream=width,height,r_frame_rate,duration",
            "-of", "json", str(video),
        ],
        check=True, capture_output=True, text=True,
    ).stdout
    stream = json.loads(out)["streams"][0]
    num, den = stream["r_frame_rate"].split("/")
    return {
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps": float(num) / float(den or 1),
        "duration": float(stream.get("duration") or 0.0),
    }


def extract_frames(video: Path, out_dir: Path, fps: float, quality: int = 2) -> list[Path]:
    """Extract frames at the given rate as high-quality JPEGs.

    Returns the sorted list of written frame paths.
    """
    _require_ffmpeg()
    info = probe_video(video)
    aspect = info["width"] / info["height"]
    if abs(aspect - 2.0) > 0.05:
        print(
            f"WARNING: {video.name} is {info['width']}x{info['height']} "
            f"(aspect {aspect:.2f}), not the 2:1 of stitched equirectangular "
            "360 video. If this is raw dual-fisheye footage, stitch it with "
            "your camera vendor's software first.",
            file=sys.stderr,
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "ffmpeg", "-y", "-v", "error",
            "-i", str(video),
            "-vf", f"fps={fps}",
            "-qscale:v", str(quality),
            str(out_dir / "%05d.jpg"),
        ],
        check=True,
    )
    frames = sorted(out_dir.glob("*.jpg"))
    if not frames:
        raise RuntimeError("ffmpeg produced no frames — is the input a valid video?")
    return frames


def sharpness_score(image_path: Path) -> float:
    """Variance of the Laplacian — higher means sharper."""
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return 0.0
    # Downscale very large equirect frames so scoring stays fast and
    # scores are comparable across resolutions.
    h, w = img.shape
    if w > 2048:
        scale = 2048 / w
        img = cv2.resize(img, (2048, int(h * scale)), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(img, cv2.CV_64F).var())


def filter_blurry(frames: list[Path], keep_ratio: float) -> list[Path]:
    """Keep the sharpest `keep_ratio` of frames, preserving temporal order.

    Motion blur is the main enemy of SfM on video, so dropping the softest
    frames usually improves the reconstruction more than the lost baselines
    hurt it.
    """
    if not 0 < keep_ratio < 1:
        return frames
    scored = [(sharpness_score(f), f) for f in frames]
    n_keep = max(2, int(round(len(scored) * keep_ratio)))
    cutoff = sorted((s for s, _ in scored), reverse=True)[n_keep - 1]
    kept = [f for s, f in scored if s >= cutoff][:n_keep]
    dropped = len(frames) - len(kept)
    if dropped:
        print(f"Dropped {dropped}/{len(frames)} blurriest frames (sharpness < {cutoff:.1f})")
    return kept
