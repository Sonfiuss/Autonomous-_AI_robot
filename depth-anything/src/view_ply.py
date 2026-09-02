#!/usr/bin/env python3
"""view_ply.py -- open a .ply point cloud (e.g. astra_map.ply) in an
interactive 3D window.

Controls (Open3D window):
    left-drag        rotate
    mouse wheel      zoom
    ctrl + drag      pan
    R                reset view
    +/-              point size
    Q or close       quit

------------------------------------------------------------------- usage -----
  python view_ply.py                          # default ../output/astra_map.ply
  python view_ply.py path\\to\\map.ply         # any ply
  python view_ply.py map.ply --voxel 0.02     # downsample huge clouds first
"""
import argparse
import sys
from pathlib import Path

_DEF_PLY = Path(__file__).resolve().parent.parent / "output" / "astra_map.ply"


def main():
    p = argparse.ArgumentParser(
        description="Interactive 3D viewer for .ply point clouds.")
    p.add_argument("ply", nargs="?", default=str(_DEF_PLY),
                   help="path to the .ply file (default: output/astra_map.ply)")
    p.add_argument("--voxel", type=float, default=0.0,
                   help="optional voxel size (m) to downsample before viewing "
                        "(useful if the cloud is huge and rendering is slow)")
    args = p.parse_args()

    path = Path(args.ply)
    if not path.exists():
        sys.exit(f"[view] file not found: {path}\n"
                 "       run astra_slam.py first, or pass the .ply path")

    import open3d as o3d
    pcd = o3d.io.read_point_cloud(str(path))
    if len(pcd.points) == 0:
        sys.exit(f"[view] {path} contains no points")

    if args.voxel > 0:
        before = len(pcd.points)
        pcd = pcd.voxel_down_sample(args.voxel)
        print(f"[view] downsampled {before} -> {len(pcd.points)} pts "
              f"(voxel {args.voxel} m)")

    print(f"[view] {path.name}: {len(pcd.points)} pts, "
          f"{'RGB colours' if pcd.has_colors() else 'NO colours (grey)'}")
    print("[view] left-drag rotate | wheel zoom | ctrl-drag pan | q quit")
    o3d.visualization.draw_geometries([pcd], window_name=f"view_ply - {path.name}",
                                      width=1280, height=720)


if __name__ == "__main__":
    sys.exit(main())
