#!/usr/bin/env python3
"""Turn 360-degree video into a 3D Gaussian splat.

Full pipeline:
    python video360_to_splat.py run walkthrough.mp4 --out dataset/

Individual stages (frames -> views -> sfm -> train) can be run separately;
each stage reads the previous stage's output from the same --out directory.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from splat360 import colmap, frames, reproject, train  # noqa: E402


def add_common(p: argparse.ArgumentParser):
    p.add_argument("--out", type=Path, required=True, help="Dataset/work directory")


def add_frame_args(p: argparse.ArgumentParser):
    p.add_argument("--fps", type=float, default=2.0,
                   help="Frames per second to extract (default 2; raise for fast camera motion)")
    p.add_argument("--keep-sharpest", type=float, default=0.8, metavar="RATIO",
                   help="Keep only this fraction of frames, dropping the blurriest (default 0.8; 1 disables)")


def add_view_args(p: argparse.ArgumentParser):
    p.add_argument("--view-size", type=int, default=1200, help="Pinhole view size in px (default 1200)")
    p.add_argument("--fov", type=float, default=90.0, help="Horizontal FOV of each view in degrees (default 90)")
    p.add_argument("--yaw-step", type=float, default=45.0, help="Yaw spacing between views in degrees (default 45)")
    p.add_argument("--pitches", type=float, nargs="+", default=[-30.0, 0.0, 30.0],
                   help="Pitch angles of the view rings (default -30 0 30)")


def add_sfm_args(p: argparse.ArgumentParser):
    p.add_argument("--matcher", choices=["auto", "exhaustive", "sequential"], default="auto",
                   help="COLMAP matcher (auto: exhaustive up to 500 images, else sequential)")
    p.add_argument("--no-gpu", action="store_true", help="Run COLMAP SIFT on CPU")


def add_train_args(p: argparse.ArgumentParser):
    p.add_argument("--trainer", choices=train.TRAINERS, default="none",
                   help="Gaussian splat trainer to invoke (default: none, just print commands)")
    p.add_argument("--inria-repo", type=Path, default=None,
                   help="Path to a graphdeco-inria/gaussian-splatting checkout (for --trainer inria)")


def stage_frames(args) -> list[Path]:
    print(f"[frames] extracting from {args.video} at {args.fps} fps")
    fr = frames.extract_frames(args.video, args.out / "frames", args.fps)
    print(f"[frames] extracted {len(fr)} frames")
    fr = frames.filter_blurry(fr, args.keep_sharpest)
    (args.out / "frames_kept.txt").write_text("\n".join(f.name for f in fr) + "\n")
    return fr


def load_kept_frames(out: Path) -> list[Path]:
    kept_file = out / "frames_kept.txt"
    frames_dir = out / "frames"
    if kept_file.exists():
        return [frames_dir / name for name in kept_file.read_text().split()]
    fr = sorted(frames_dir.glob("*.jpg"))
    if not fr:
        raise SystemExit(f"No frames in {frames_dir} — run the `frames` stage first.")
    return fr


def stage_views(args, fr: list[Path] | None = None) -> list[reproject.RigView]:
    fr = fr or load_kept_frames(args.out)
    rig = reproject.default_rig(args.yaw_step, tuple(args.pitches))
    print(f"[views] rendering {len(rig)} views/frame x {len(fr)} frames "
          f"({args.view_size}px, {args.fov} deg FOV)")
    reproject.render_views(fr, args.out / "images", args.view_size, args.fov, rig)
    return rig


def stage_sfm(args, views_per_frame: int):
    colmap.run_sfm(
        args.out,
        view_size=args.view_size,
        fov_deg=args.fov,
        views_per_frame=views_per_frame,
        matcher=args.matcher,
        use_gpu=not args.no_gpu,
    )


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("run", help="Full pipeline: frames -> views -> sfm -> train")
    p.add_argument("video", type=Path, help="Input 360 (equirectangular) video")
    for add in (add_common, add_frame_args, add_view_args, add_sfm_args, add_train_args):
        add(p)

    p = sub.add_parser("frames", help="Stage 1: extract frames from video")
    p.add_argument("video", type=Path)
    add_common(p)
    add_frame_args(p)

    p = sub.add_parser("views", help="Stage 2: reproject frames to pinhole views")
    add_common(p)
    add_view_args(p)

    p = sub.add_parser("sfm", help="Stage 3: COLMAP structure-from-motion")
    add_common(p)
    add_view_args(p)  # must match the values used in the views stage
    add_sfm_args(p)

    p = sub.add_parser("train", help="Stage 4: train the Gaussian splat")
    add_common(p)
    add_train_args(p)

    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    if args.cmd == "run":
        fr = stage_frames(args)
        rig = stage_views(args, fr)
        stage_sfm(args, views_per_frame=len(rig))
        train.train(args.out, args.trainer, args.inria_repo)
    elif args.cmd == "frames":
        stage_frames(args)
    elif args.cmd == "views":
        stage_views(args)
    elif args.cmd == "sfm":
        rig = reproject.default_rig(args.yaw_step, tuple(args.pitches))
        stage_sfm(args, views_per_frame=len(rig))
    elif args.cmd == "train":
        train.train(args.out, args.trainer, args.inria_repo)


if __name__ == "__main__":
    main()
