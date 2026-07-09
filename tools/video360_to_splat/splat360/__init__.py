"""video360_to_splat: turn 360-degree video into a 3D Gaussian splat dataset.

Pipeline stages:
  1. frames    - extract equirectangular frames from the video (ffmpeg)
  2. views     - reproject each frame into a rig of pinhole views
  3. sfm       - recover camera poses + sparse points with COLMAP
  4. train     - hand the dataset to a Gaussian splatting trainer
"""

__version__ = "0.1.0"
