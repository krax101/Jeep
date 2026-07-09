# video360_to_splat

Turn 360° video (e.g. from an Insta360, GoPro Max, or Ricoh Theta) into a
**3D Gaussian splat**.

Gaussian splatting trainers can't consume equirectangular video directly —
they need pinhole images with known camera poses. This tool does the whole
conversion:

```
360 video ──ffmpeg──▶ equirect frames ──reproject──▶ pinhole view rig ──COLMAP──▶ posed dataset ──trainer──▶ .ply splat
   (1)                    (2)                             (3)                          (4)
```

1. **frames** — extract frames at a chosen rate, score them for sharpness
   (variance of Laplacian) and drop the blurriest.
2. **views** — resample each equirectangular frame into a rig of overlapping
   perspective views (default: 8 yaws × 3 pitches, 90° FOV each). The maps
   are precomputed once, so this is fast. Nadir/zenith are excluded so the
   tripod or your hand doesn't poison the reconstruction.
3. **sfm** — COLMAP feature extraction, matching, and mapping. Because the
   views are synthesized, the intrinsics are known *exactly* and are locked
   as a single shared PINHOLE camera, which makes SfM much more robust.
4. **train** — hand the standard `images/ + sparse/0/` dataset to
   nerfstudio (splatfacto), OpenSplat, or the Inria reference
   implementation — or just print the commands to run elsewhere.

## Requirements

- Python 3.10+, `pip install -r requirements.txt` (numpy, OpenCV)
- [ffmpeg](https://ffmpeg.org) on PATH (stages 1)
- [COLMAP](https://colmap.github.io) on PATH (stage 3)
- A CUDA GPU + one of the trainers below (stage 4, optional here — the
  dataset is portable, so you can train on another machine)

## Usage

Full pipeline:

```sh
python video360_to_splat.py run walkthrough.mp4 --out dataset/
```

Then train (needs a CUDA GPU):

```sh
pip install nerfstudio
ns-train splatfacto colmap --data dataset/
# or, if you built OpenSplat:
python video360_to_splat.py train --out dataset/ --trainer opensplat
```

View the result with the bundled viewer (see below), or in
[SuperSplat](https://playcanvas.com/supersplat/editor).

Stages can also run one at a time (`frames`, `views`, `sfm`, `train`), all
sharing the same `--out` directory — useful for re-running SfM with
different settings without re-rendering views. If you pass non-default
`--view-size`/`--fov` to `views`, pass the same values to `sfm` so the
locked intrinsics match.

### Key options

| Option | Default | Notes |
|---|---|---|
| `--fps` | 2 | Extraction rate. Raise for fast camera motion, lower for slow pans. |
| `--keep-sharpest` | 0.8 | Fraction of frames kept after blur filtering (1 = keep all). |
| `--view-size` | 1200 | Pinhole view resolution (px, square). |
| `--fov` | 90 | Horizontal FOV per view. |
| `--yaw-step` | 45 | Yaw spacing; with 90° FOV, 45° gives ~50% overlap. |
| `--pitches` | -30 0 30 | Pitch rings. Add ±60 for tall scenes. |
| `--matcher` | auto | `exhaustive` ≤ 500 images, else `sequential`. |
| `--trainer` | none | `nerfstudio`, `opensplat`, `inria`, or `none` (print commands). |

### Shooting tips

- Move **slowly and smoothly**; motion blur is the #1 cause of SfM failure.
- **Translate**, don't just rotate — parallax is what creates 3D. Walk a
  loop or lawnmower pattern through the scene.
- Lock exposure/white balance on the camera if you can.
- Stitch dual-fisheye footage to equirectangular (2:1 aspect) with the
  vendor app first; the tool warns if the input isn't 2:1.

## Viewer

`viewer/index.html` is a self-contained WebGL2 Gaussian splat viewer — one
file, no dependencies, works offline. Open it in a browser and drop in a
`.splat` or a trainer-output `.ply`, or serve it with a scene URL:

```sh
python -m http.server -d .          # from this directory
# then open http://localhost:8000/viewer/index.html?url=/path/to/scene.ply
```

Fly controls:

| Input | Action |
|---|---|
| click | capture the mouse |
| mouse | look around |
| `W` `A` `S` `D` | move forward / left / back / right |
| `Space` or `E` | move up |
| `Ctrl`, `C` or `Q` | move down (`C`/`Q` avoid the browser's `Ctrl+W`) |
| `Shift` | sprint (4x) |
| scroll wheel | adjust fly speed |
| `R` | reset camera · `Esc` releases the mouse |

It renders with the standard splatting approach: per-splat 3D covariance
packed into a texture, depth-sorted front-to-back in a web worker
(counting sort), and composited as instanced quads with a Gaussian falloff
shader.

The viewer has its own end-to-end test, run headlessly with Playwright:
`python viewer/make_test_splat.py` builds a synthetic scene with markers
at known positions, and `node viewer/test_viewer.mjs` checks projection
directions, occlusion order, `.ply` parsing, and every movement key
against rendered pixels.

## Verifying the geometry

`python -m splat360.selftest` renders views of a synthetic equirect image
with markers at known directions and checks each view centres on the right
marker (plus seam continuity across the ±180° wrap). No video needed.
