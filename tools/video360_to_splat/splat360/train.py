"""Stage 4: hand the COLMAP dataset to a Gaussian splatting trainer.

Training needs a CUDA GPU and one of the well-known implementations. This
stage shells out to whichever one you have; `none` (the default) just prints
the commands so you can run training on another machine.
"""

import shutil
import subprocess
from pathlib import Path

TRAINERS = ("none", "nerfstudio", "opensplat", "inria")


def _run(args: list[str], cwd: Path | None = None):
    print("  $", " ".join(args))
    subprocess.run(args, check=True, cwd=cwd)


def train(dataset: Path, trainer: str, inria_repo: Path | None = None):
    dataset = dataset.resolve()
    if trainer == "none":
        print_next_steps(dataset)
    elif trainer == "nerfstudio":
        if shutil.which("ns-train") is None:
            raise RuntimeError("ns-train not found — `pip install nerfstudio` first.")
        _run(["ns-train", "splatfacto", "colmap", "--data", str(dataset)])
    elif trainer == "opensplat":
        if shutil.which("opensplat") is None:
            raise RuntimeError("opensplat not found — build it from https://github.com/pierotofy/OpenSplat")
        _run(["opensplat", str(dataset), "-o", str(dataset / "splat.ply")])
    elif trainer == "inria":
        if inria_repo is None or not (inria_repo / "train.py").exists():
            raise RuntimeError(
                "--inria-repo must point at a checkout of "
                "https://github.com/graphdeco-inria/gaussian-splatting"
            )
        _run(
            ["python", "train.py", "-s", str(dataset), "-m", str(dataset / "model")],
            cwd=inria_repo,
        )
    else:
        raise ValueError(f"Unknown trainer: {trainer} (choose from {TRAINERS})")


def print_next_steps(dataset: Path):
    print(
        f"""
Dataset ready at: {dataset}
  images/    pinhole views
  sparse/0/  COLMAP camera poses + sparse points

Train a Gaussian splat with any of (CUDA GPU required):

  # nerfstudio (pip install nerfstudio)
  ns-train splatfacto colmap --data {dataset}

  # OpenSplat (https://github.com/pierotofy/OpenSplat)
  opensplat {dataset} -o {dataset}/splat.ply

  # Reference implementation (https://github.com/graphdeco-inria/gaussian-splatting)
  python train.py -s {dataset} -m {dataset}/model

View the resulting .ply/.splat in https://playcanvas.com/supersplat/editor,
https://antimatter15.com/splat/, or the trainer's own viewer.
"""
    )
