"""Stage 3: structure-from-motion with COLMAP.

Produces the layout every Gaussian splatting trainer understands:

    dataset/
      images/            pinhole views (input)
      sparse/0/          cameras.bin, images.bin, points3D.bin
"""

import shutil
import subprocess
from pathlib import Path

from .reproject import pinhole_intrinsics


class ColmapNotFound(RuntimeError):
    pass


def _require_colmap():
    if shutil.which("colmap") is None:
        raise ColmapNotFound(
            "colmap not found on PATH. Install it first "
            "(e.g. `apt install colmap` or https://colmap.github.io/install.html)."
        )


def _run(args: list[str]):
    print("  $", " ".join(args))
    subprocess.run(args, check=True)


def run_sfm(
    dataset: Path,
    view_size: int,
    fov_deg: float,
    views_per_frame: int,
    matcher: str = "auto",
    use_gpu: bool = True,
) -> Path:
    """Run COLMAP feature extraction, matching and mapping over dataset/images.

    All views share exact known intrinsics (we synthesised them), so the
    camera model is a single fixed PINHOLE — this removes a whole family of
    SfM failure modes.

    Returns the path to the selected sparse model (dataset/sparse/0).
    """
    _require_colmap()
    images = dataset / "images"
    db = dataset / "colmap.db"
    sparse = dataset / "sparse"
    n_images = len(list(images.glob("*.jpg")))
    if n_images < 2:
        raise RuntimeError(f"Need at least 2 images in {images}, found {n_images}")

    if db.exists():
        db.unlink()
    sparse.mkdir(parents=True, exist_ok=True)

    f, cx, cy = pinhole_intrinsics(view_size, fov_deg)
    gpu = "1" if use_gpu else "0"

    print(f"[sfm] feature extraction over {n_images} images")
    _run([
        "colmap", "feature_extractor",
        "--database_path", str(db),
        "--image_path", str(images),
        "--ImageReader.camera_model", "PINHOLE",
        "--ImageReader.single_camera", "1",
        "--ImageReader.camera_params", f"{f:.6f},{f:.6f},{cx:.6f},{cy:.6f}",
        "--SiftExtraction.use_gpu", gpu,
    ])

    if matcher == "auto":
        matcher = "exhaustive" if n_images <= 500 else "sequential"
    print(f"[sfm] {matcher} matching")
    if matcher == "exhaustive":
        _run([
            "colmap", "exhaustive_matcher",
            "--database_path", str(db),
            "--SiftMatching.use_gpu", gpu,
        ])
    elif matcher == "sequential":
        # Names are frame-major, so an overlap of ~3 rig rings connects each
        # view to its rig neighbours and to the surrounding frames.
        overlap = max(10, 3 * views_per_frame)
        _run([
            "colmap", "sequential_matcher",
            "--database_path", str(db),
            "--SequentialMatching.overlap", str(overlap),
            "--SiftMatching.use_gpu", gpu,
        ])
    else:
        raise ValueError(f"Unknown matcher: {matcher}")

    print("[sfm] mapping (this is the slow part)")
    _run([
        "colmap", "mapper",
        "--database_path", str(db),
        "--image_path", str(images),
        "--output_path", str(sparse),
        "--Mapper.ba_refine_focal_length", "0",
        "--Mapper.ba_refine_principal_point", "0",
        "--Mapper.ba_refine_extra_params", "0",
    ])

    models = sorted(p for p in sparse.iterdir() if p.is_dir() and (p / "images.bin").exists())
    if not models:
        raise RuntimeError(
            "COLMAP mapper produced no model. Common causes: too few frames, "
            "heavy motion blur, or a scene with little texture. Try a higher "
            "--fps, --keep-sharpest 0.5, or slower camera motion."
        )
    best = max(models, key=lambda p: (p / "images.bin").stat().st_size)
    target = sparse / "0"
    if best != target:
        if target.exists():
            shutil.rmtree(target)
        best.rename(target)

    _run([
        "colmap", "model_analyzer",
        "--path", str(target),
    ])
    return target
