#!/usr/bin/env python3
"""Generate Albert Park centerline + racing-line CSVs from real track data.

Source centerline: TUM racetrack-database ``Melbourne.csv`` (OpenStreetMap,
~5.28 km).  The path is rotated so the main straight is horizontal (eastbound
clockwise lap), resampled uniformly, then written to ``waypoints_f1.csv``.

Usage (from repo root):
    python3 src/AutoCarROS2/autocar_racing_line/scripts/generate_f1_circuit_waypoints.py
    python3 .../generate_f1_circuit_waypoints.py --update-world --plot
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
DATA_DIR = PKG_DIR / 'data'
REPO_ROOT = PKG_DIR.parents[2]
SOURCE_CSV = DATA_DIR / 'melbourne_source.csv'
WORLD_PATH = REPO_ROOT / 'src/AutoCarROS2/autocar_gazebo/worlds/race_f1_circuit.world'
META_PATH = DATA_DIR / 'f1_circuit_meta.json'
RACING_SCRIPT = SCRIPT_DIR / 'generate_racing_line.py'
TEMPLATE_WORLD = REPO_ROOT / 'src/AutoCarROS2/autocar_gazebo/worlds/race_oval.world'
DEFAULT_PLOT = REPO_ROOT / 'docs' / 'images' / 'racing_line_f1.png'

REAL_LENGTH_M = 5278.0
N_WAYPOINTS = 140
N_ROAD_POINTS = 420
ROAD_WIDTH_M = 16.0
SPAWN_BACK_OFFSET_M = 8.0

# Main-straight segment in the source CSV (low curvature, ~245 m).
STRAIGHT_ANCHOR = 1025
STRAIGHT_HEADING_DEG = 136.1306


def read_source(path: Path) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    with path.open() as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#'):
                continue
            pts.append((float(row[0]), float(row[1])))
    if len(pts) < 4:
        raise ValueError(f'Need at least 4 source points in {path}')
    return pts


def _rotate(points: list[tuple[float, float]], deg: float) -> list[tuple[float, float]]:
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def _translate(points: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in points]


def _chord_lengths(pts: list[tuple[float, float]]) -> list[float]:
    cum = [0.0]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
    return cum


def _point_at_arc(pts: list[tuple[float, float]], cum: list[float], s: float) -> tuple[float, float]:
    total = cum[-1]
    s = s % total
    lo, hi = 0, len(cum) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if cum[mid] < s:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    c0, c1 = cum[i - 1], cum[i]
    p0, p1 = pts[i - 1], pts[i]
    frac = (s - c0) / (c1 - c0) if c1 > c0 else 0.0
    return (
        p0[0] + frac * (p1[0] - p0[0]),
        p0[1] + frac * (p1[1] - p0[1]),
    )


def resample_uniform(pts: list[tuple[float, float]], n: int) -> list[tuple[float, float]]:
    closed = pts if pts[0] == pts[-1] else pts + [pts[0]]
    cum = _chord_lengths(closed)
    total = cum[-1]
    return [_point_at_arc(closed, cum, total * i / (n - 1)) for i in range(n)]


def scale_to_length(pts: list[tuple[float, float]], target: float) -> list[tuple[float, float]]:
    closed = pts if pts[0] == pts[-1] else pts + [pts[0]]
    length = _chord_lengths(closed)[-1]
    if length < 1e-6:
        return pts
    s = target / length
    return [(x * s, y * s) for x, y in pts]


def polyline_length(pts: list[tuple[float, float]]) -> float:
    closed = pts if pts[0] == pts[-1] else pts + [pts[0]]
    cum = _chord_lengths(closed)
    return cum[-1]


def prepare_albert_park(source: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Rotate, center, and reorder to clockwise lap starting on main straight."""
    # Align source straight heading (136 deg) with +X (eastbound).
    rotated = _rotate(source, -STRAIGHT_HEADING_DEG)

    cx = sum(p[0] for p in rotated) / len(rotated)
    cy = sum(p[1] for p in rotated) / len(rotated)
    centered = _translate(rotated, -cx, -cy)

    # Put the main straight on the bottom of the map (negative Y).
    straight_y = centered[STRAIGHT_ANCHOR][1]
    if straight_y > 0.0:
        centered = [(x, -y) for x, y in centered]

    # Start at the west end of the main straight, lap clockwise.
    n = len(centered)
    straight_idxs = list(range(980, n)) + list(range(0, 120))
    west_i = min(straight_idxs, key=lambda i: centered[i][0])
    ordered = centered[west_i:] + centered[:west_i]

    # Close tiny source gap.
    if math.hypot(ordered[0][0] - ordered[-1][0], ordered[0][1] - ordered[-1][1]) > 1e-3:
        ordered.append(ordered[0])
    return ordered


def write_csv(path: Path, pts: list[tuple[float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['X-axis', 'Y-axis'])
        for x, y in pts:
            w.writerow([f'{x:.6f}', f'{y:.6f}'])


def spawn_yaw_rad(x0: float, y0: float, x1: float, y1: float) -> float:
    """Yaw for Gazebo pose: yaw=0 faces world +Y."""
    dx, dy = x1 - x0, y1 - y0
    heading = math.atan2(dy, dx)
    return heading - math.pi / 2.0


def compute_spawn_pose(wp_pts: list[tuple[float, float]]) -> tuple[float, float, float]:
    """On-centerline pose slightly before waypoint 0, facing along the lap."""
    x0, y0 = wp_pts[0]
    x1, y1 = wp_pts[1]
    dx, dy = x1 - x0, y1 - y0
    seg_len = math.hypot(dx, dy) or 1.0
    back = min(SPAWN_BACK_OFFSET_M, 0.45 * seg_len)
    tx, ty = dx / seg_len, dy / seg_len
    return x0 - back * tx, y0 - back * ty, spawn_yaw_rad(x0, y0, x1, y1)


def update_world(path: Path, road_pts: list[tuple[float, float]], wp_pts: list[tuple[float, float]]) -> None:
    lines = [f'      <point>{x:.3f} {y:.3f} 0</point>' for x, y in road_pts]
    new_road = (
        '    <road name="track">\n'
        f'      <width>{ROAD_WIDTH_M:.1f}</width>\n'
        + '\n'.join(lines)
        + '\n    </road>'
    )
    text = path.read_text()
    text = re.sub(r'    <road name="track">.*?</road>', new_road, text, flags=re.DOTALL)

    wx, wy = wp_pts[0]
    sx, sy, yaw = compute_spawn_pose(wp_pts)
    spawn = f'<pose>{sx:.2f} {sy:.2f} 0.0 0 0 {yaw:.4f}</pose>'
    spawn_comment = (
        f'      <!-- Spawn on centerline {SPAWN_BACK_OFFSET_M:.0f} m before waypoint 0 '
        f'({wx:.2f}, {wy:.2f}).\n'
        f'           Eastbound clockwise lap; yaw={yaw:.4f} faces along-track. -->'
    )
    text = re.sub(
        r'<model name="autocar">\s*<!--.*?-->\s*<pose>[^<]+</pose>',
        f'<model name="autocar">\n{spawn_comment}\n      {spawn}',
        text,
        count=1,
        flags=re.DOTALL,
    )
    text = text.replace(
        'Elliptical race oval — same ~651 m centerline length as race_circuit.world',
        'Albert Park Grand Prix Circuit (real 5.278 km layout, clockwise)',
    )
    text = re.sub(
        r'    <!-- ===== Race-track.*?</road>',
        '    <!-- ===== Race-track (Albert Park) ===== -->\n' + new_road,
        text,
        flags=re.DOTALL,
    )
    text = re.sub(
        r'\n\s*<!-- ===== Guardrails.*?</model>\s*(?=\n\s*</world>)',
        '\n',
        text,
        flags=re.DOTALL,
    )
    text = re.sub(
        r'\n\s*<model name="track_outer_guardrail">.*?</model>\s*(?=\n\s*</world>)',
        '\n',
        text,
        flags=re.DOTALL,
    )
    path.write_text(text)


def ensure_world_template() -> None:
    if WORLD_PATH.is_file():
        return
    if not TEMPLATE_WORLD.is_file():
        raise FileNotFoundError(f'Missing template world: {TEMPLATE_WORLD}')
    text = TEMPLATE_WORLD.read_text()
    text = text.replace(
        'Elliptical race oval — same ~651 m centerline length as race_circuit.world',
        'Albert Park Grand Prix Circuit (real 5.278 km layout, clockwise)',
    )
    text = re.sub(
        r'    <!-- ===== Guardrails.*?</model>\s*\n\s*</world>',
        '  </world>',
        text,
        flags=re.DOTALL,
    )
    WORLD_PATH.write_text(text)


def write_meta(wp_pts: list[tuple[float, float]]) -> None:
    fx, fy = wp_pts[0]
    sx, sy, yaw = compute_spawn_pose(wp_pts)
    meta = {
        'finish_line': {
            'mode': 'pos_x',
            'line_x': fx,
            'y_center': fy,
            'y_half_width': ROAD_WIDTH_M,
        },
        'spawn': {
            'x': sx,
            'y': sy,
            'yaw': yaw,
        },
    }
    META_PATH.write_text(json.dumps(meta, indent=2) + '\n')


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--update-world', action='store_true')
    p.add_argument(
        '--target-length',
        type=float,
        default=REAL_LENGTH_M,
        help='Scale centerline to this length in metres (default: real 5278 m).',
    )
    p.add_argument('--waypoints', type=int, default=N_WAYPOINTS)
    p.add_argument('--road-points', type=int, default=N_ROAD_POINTS)
    p.add_argument(
        '--plot',
        nargs='?',
        const=DEFAULT_PLOT,
        default=None,
        type=Path,
        help='Write comparison PNG (default: docs/images/racing_line_f1.png)',
    )
    args = p.parse_args()

    if not SOURCE_CSV.is_file():
        raise FileNotFoundError(
            f'Missing {SOURCE_CSV}. Download Melbourne.csv from '
            'https://github.com/TUMFTM/racetrack-database')

    prepared = prepare_albert_park(read_source(SOURCE_CSV))
    if args.target_length > 0:
        prepared = scale_to_length(prepared, args.target_length)

    waypoints = resample_uniform(prepared, args.waypoints)
    road = resample_uniform(prepared, args.road_points)
    road.append(road[0])

    centerline_csv = DATA_DIR / 'waypoints_f1.csv'
    racing_csv = DATA_DIR / 'waypoints_f1_racing.csv'
    write_csv(centerline_csv, waypoints)
    write_meta(waypoints)
    print(f'Wrote {len(waypoints)} centerline points to {centerline_csv}')
    print(f'  polyline length {polyline_length(waypoints):.3f} m (target {args.target_length:.1f} m)')
    print(f'  start/finish at ({waypoints[0][0]:.2f}, {waypoints[0][1]:.2f})')

    if args.update_world:
        ensure_world_template()
        update_world(WORLD_PATH, road, waypoints)
        print(f'Updated {WORLD_PATH}')

    cmd = [sys.executable, str(RACING_SCRIPT), '--input', str(centerline_csv), '--output', str(racing_csv)]
    if args.plot is not None:
        cmd.extend(['--plot', str(args.plot)])
    subprocess.run(cmd, check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
