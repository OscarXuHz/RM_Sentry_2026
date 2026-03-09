#!/usr/bin/env python3
"""Convert a 3D .pcd point cloud into occ/bev/occtopo PNGs for sentry_planning.

Outputs (8-bit PNG):
- occ.png: occupancy grid (0 free, 255 occupied)
- bev.png: height map encoded with height_bias/height_interval
- occtopo.png: skeleton of free space (centerline/topo map)
"""

import argparse
from pathlib import Path
import numpy as np

try:
    import open3d as o3d
except Exception as exc:  # pragma: no cover
    raise SystemExit("Missing dependency: open3d. Install with: pip install open3d") from exc

try:
    import cv2
except Exception as exc:  # pragma: no cover
    raise SystemExit("Missing dependency: opencv-python. Install with: pip install opencv-python") from exc

try:
    from skimage.morphology import skeletonize
except Exception as exc:  # pragma: no cover
    raise SystemExit("Missing dependency: scikit-image. Install with: pip install scikit-image") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert PCD to occ/bev/occtopo PNG maps.")
    parser.add_argument("--pcd", required=True, help="Input .pcd file path")
    parser.add_argument("--out", required=True, help="Output directory for PNGs")

    parser.add_argument("--resolution", type=float, default=0.1, help="Map resolution (m/pixel)")
    parser.add_argument("--map-x-size", type=float, default=50.0, help="Map size X in meters")
    parser.add_argument("--map-y-size", type=float, default=50.0, help="Map size Y in meters")
    parser.add_argument("--lower-x", type=float, default=0.0, help="Map lower bound X")
    parser.add_argument("--lower-y", type=float, default=0.0, help="Map lower bound Y")
    parser.add_argument(
        "--center",
        choices=["none", "centroid", "bbox"],
        default="none",
        help="Auto center map on pointcloud: 'centroid' or bounding 'bbox' (overrides lower-x/lower-y)",
    )

    parser.add_argument("--height-bias", type=float, default=-0.5, help="Height bias used by planner")
    parser.add_argument("--height-interval", type=float, default=2.0, help="Height interval used by planner")
    parser.add_argument(
        "--height-mode",
        choices=["min", "mean", "percentile"],
        default="min",
        help="How to compute per-cell height from point cloud",
    )
    parser.add_argument(
        "--height-percentile",
        type=float,
        default=10.0,
        help="Percentile for height-mode=percentile (e.g., 10 for near-ground)",
    )

    parser.add_argument(
        "--bev-brightness",
        type=float,
        default=1.0,
        help="Brightness multiplier applied to the generated BEV PNG (default 1.0)",
    )

    parser.add_argument(
        "--occ-z-min",
        type=float,
        default=None,
        help="Min Z to include in occupancy (meters). Default: no min filter.",
    )
    parser.add_argument(
        "--occ-z-max",
        type=float,
        default=None,
        help="Max Z to include in occupancy (meters). Default: no max filter.",
    )

    parser.add_argument(
        "--wall-height-threshold",
        type=float,
        default=0.5,
        help="Minimum per-cell height variation (m) to treat cell as a wall/obstacle (default 0.5).",
    )
    parser.add_argument(
        "--floor-percentile",
        type=float,
        default=10.0,
        help="Percentile used to estimate floor height for wall detection (default 10).",
    )

    return parser.parse_args()


def world_to_grid(x: np.ndarray, y: np.ndarray, lower_x: float, lower_y: float, res: float, rows: int, cols: int):
    col = ((x - lower_x) / res).astype(np.int32)
    row = (rows - 1 - ((y - lower_y) / res).astype(np.int32))
    mask = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
    return row, col, mask


def build_occ(points: np.ndarray, args: argparse.Namespace, rows: int, cols: int) -> np.ndarray:
    occ = np.zeros((rows, cols), dtype=np.uint8)
    # Filter points by Z range for occupancy processing
    pts = points
    if args.occ_z_min is not None:
        pts = pts[pts[:, 2] >= args.occ_z_min]
    if args.occ_z_max is not None:
        pts = pts[pts[:, 2] <= args.occ_z_max]

    if pts.size == 0:
        return occ

    # Per-cell min/max height
    z_min = np.full((rows, cols), np.inf, dtype=np.float32)
    z_max = np.full((rows, cols), -np.inf, dtype=np.float32)

    row, col, mask = world_to_grid(pts[:, 0], pts[:, 1], args.lower_x, args.lower_y, args.resolution, rows, cols)
    row = row[mask]
    col = col[mask]
    z = pts[mask, 2]

    for r, c, zz in zip(row, col, z):
        if zz < z_min[r, c]:
            z_min[r, c] = zz
        if zz > z_max[r, c]:
            z_max[r, c] = zz

    # Estimate global floor height from all points (not filtered) using percentile
    try:
        global_floor = float(np.percentile(points[:, 2], getattr(args, "floor_percentile", 10.0)))
    except Exception:
        global_floor = float(np.nanmin(points[:, 2])) if points.size else 0.0

    height_diff = z_max - z_min

    # A cell is considered a wall/obstacle if either:
    # - the per-cell height variation exceeds the wall-height threshold (vertical structure), or
    # - the cell's max height is significantly above the estimated floor (isolated tall obstacle)
    wall_mask = (height_diff >= getattr(args, "wall_height_threshold", 0.5)) & (z_min != np.inf)
    high_mask = (z_max >= (global_floor + getattr(args, "wall_height_threshold", 0.5))) & (z_max != -np.inf)

    occ_mask = wall_mask | high_mask
    occ[occ_mask] = 255
    return occ


def build_bev(points: np.ndarray, args: argparse.Namespace, rows: int, cols: int) -> np.ndarray:
    z_grid = np.full((rows, cols), np.nan, dtype=np.float32)

    row, col, mask = world_to_grid(points[:, 0], points[:, 1], args.lower_x, args.lower_y, args.resolution, rows, cols)
    row = row[mask]
    col = col[mask]
    z = points[mask, 2]

    if args.height_mode == "min":
        for r, c, zz in zip(row, col, z):
            if np.isnan(z_grid[r, c]) or zz < z_grid[r, c]:
                z_grid[r, c] = zz
    elif args.height_mode == "mean":
        sum_grid = np.zeros((rows, cols), dtype=np.float32)
        cnt_grid = np.zeros((rows, cols), dtype=np.int32)
        for r, c, zz in zip(row, col, z):
            sum_grid[r, c] += zz
            cnt_grid[r, c] += 1
        mask_cnt = cnt_grid > 0
        z_grid[mask_cnt] = sum_grid[mask_cnt] / cnt_grid[mask_cnt]
    else:  # percentile
        # Collect z values per cell (memory heavy for very dense maps)
        buckets = [[[] for _ in range(cols)] for _ in range(rows)]
        for r, c, zz in zip(row, col, z):
            buckets[r][c].append(float(zz))
        for r in range(rows):
            for c in range(cols):
                if buckets[r][c]:
                    z_grid[r, c] = np.percentile(buckets[r][c], args.height_percentile)

    # Fill empty cells with height_bias (planner expects valid heights everywhere)
    z_grid = np.where(np.isnan(z_grid), args.height_bias, z_grid)

    # Map height to 0..255 using planner formula: z = (bev * interval / 255) + bias
    bev = ((z_grid - args.height_bias) * 255.0 / args.height_interval)
    # Apply brightness multiplier (user can increase >1.0 to make map brighter)
    bev = bev * getattr(args, "bev_brightness", 1.0)
    bev = np.clip(bev, 0, 255).astype(np.uint8)
    return bev


def build_occtopo(occ: np.ndarray) -> np.ndarray:
    # Disabled Topo/skeleton generation disabled: return a blank map.
    # Original implementation (kept for reference):
    free = occ == 0
    skel = skeletonize(free).astype(np.uint8) * 255
    return skel
    #return np.zeros_like(occ, dtype=np.uint8)


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    pcd_path = Path(args.pcd)
    if not pcd_path.exists():
        raise SystemExit(f"PCD not found: {pcd_path}")

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(pcd.points)
    if points.size == 0:
        raise SystemExit("PCD has no points.")

    # Auto-center map based on the point cloud if requested
    if args.center != "none":
        min_x, min_y = points[:, 0].min(), points[:, 1].min()
        max_x, max_y = points[:, 0].max(), points[:, 1].max()
        if args.center == "centroid":
            cx, cy = points[:, 0].mean(), points[:, 1].mean()
        else:  # bbox center
            cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0

        # compute lower bounds so map is centered at (cx, cy)
        args.lower_x = cx - args.map_x_size / 2.0
        args.lower_y = cy - args.map_y_size / 2.0
        print(f"Auto-centered map: center=({cx:.3f},{cy:.3f}), lower_x={args.lower_x:.3f}, lower_y={args.lower_y:.3f}")

    rows = int(round(args.map_y_size / args.resolution))
    cols = int(round(args.map_x_size / args.resolution))

    occ = build_occ(points, args, rows, cols)
    bev = build_bev(points, args, rows, cols)
    occtopo = build_occtopo(occ)

    cv2.imwrite(str(out_dir / "occ.png"), occ)
    cv2.imwrite(str(out_dir / "bev.png"), bev)
    cv2.imwrite(str(out_dir / "occtopo.png"), occtopo)

    print("Wrote:", out_dir / "occ.png")
    print("Wrote:", out_dir / "bev.png")
    print("Wrote:", out_dir / "occtopo.png")


if __name__ == "__main__":
    main()
