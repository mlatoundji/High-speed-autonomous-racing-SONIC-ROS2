#!/usr/bin/env python3
"""Generate oval centerline + racing-line CSVs from the ellipse in race_oval.world.

Centerline: 46 waypoints, uniform arc length (CCW from +X), perimeter matched to
race_circuit (~650.85 m). Racing line: same pipeline as generate_racing_line.py.

Usage (from repo root):
    python3 src/AutoCarROS2/autocar_racing_line/scripts/generate_oval_waypoints.py
    python3 .../generate_oval_waypoints.py --update-world
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
DATA_DIR = PKG_DIR / 'data'
REPO_ROOT = PKG_DIR.parents[2]
WORLD_PATH = REPO_ROOT / 'src/AutoCarROS2/autocar_gazebo/worlds/race_oval.world'
RACING_SCRIPT = SCRIPT_DIR / 'generate_racing_line.py'

TARGET_LENGTH_M = 650.849
AXIS_RATIO = 1.6
N_WAYPOINTS = 46


def ellipse_perimeter_ramanujan(a: float, b: float) -> float:
    h = ((a - b) / (a + b)) ** 2
    return math.pi * (a + b) * (1 + 3 * h / (10 + math.sqrt(4 - 3 * h)))


def solve_semi_axes(target: float, ratio: float) -> tuple[float, float]:
    lo, hi = 1.0, 500.0
    for _ in range(80):
        mid = (lo + hi) / 2
        a = mid
        b = a / ratio
        if ellipse_perimeter_ramanujan(a, b) < target:
            lo = mid
        else:
            hi = mid
    a = (lo + hi) / 2
    return a, a / ratio


def build_arc_length_table(a: float, b: float, n_samples: int = 20000):
    ts = [2 * math.pi * i / n_samples for i in range(n_samples + 1)]

    def point(t):
        return a * math.cos(t), b * math.sin(t)

    pts = [point(t) for t in ts]
    seg_lens = [
        math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        for i in range(n_samples)
    ]
    cum = [0.0]
    for length in seg_lens:
        cum.append(cum[-1] + length)
    return ts, cum, cum[-1], point


def centerline_waypoints(a: float, b: float, n: int) -> list[tuple[float, float]]:
    ts_tab, cum, total, point = build_arc_length_table(a, b)

    def t_at_arc(s: float) -> float:
        s = s % total
        lo_i, hi_i = 0, len(cum) - 1
        while lo_i < hi_i:
            mid = (lo_i + hi_i) // 2
            if cum[mid] < s:
                lo_i = mid + 1
            else:
                hi_i = mid
        i = max(1, lo_i)
        t0, t1 = ts_tab[i - 1], ts_tab[i]
        c0, c1 = cum[i - 1], cum[i]
        frac = (s - c0) / (c1 - c0) if c1 > c0 else 0.0
        return t0 + frac * (t1 - t0)

    out = []
    for i in range(n):
        s = total * i / (n - 1) if i < n - 1 else 0.0
        out.append(point(t_at_arc(s)))
    return out


def write_csv(path: Path, pts: list[tuple[float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['X-axis', 'Y-axis'])
        for x, y in pts:
            w.writerow([f'{x:.6f}', f'{y:.6f}'])


def update_world_road(path: Path, pts: list[tuple[float, float]]) -> None:
    lines = [f'      <point>{x:.3f} {y:.3f} 0</point>' for x, y in pts]
    new_road = (
        '    <road name="track">\n      <width>16.0</width>\n'
        + '\n'.join(lines)
        + '\n    </road>'
    )
    text = path.read_text()
    text = re.sub(r'    <road name="track">.*?</road>', new_road, text, flags=re.DOTALL)
    text = re.sub(
        r'(<pose>)[\d.-]+(\s+-20\.0\s+0\.0\s+0\s+0\s+0</pose>)',
        rf'\g<1>{pts[0][0]:.2f}\2',
        text,
        count=1,
    )
    path.write_text(text)


def polyline_length(pts: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        for i in range(len(pts) - 1)
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        '--update-world',
        action='store_true',
        help='Sync <road> points and spawn pose in race_oval.world',
    )
    p.add_argument(
        '--plot',
        nargs='?',
        const=REPO_ROOT / 'docs' / 'images' / 'racing_line_oval.png',
        default=None,
        type=Path,
        help='Write comparison PNG (default: docs/images/racing_line_oval.png)',
    )
    args = p.parse_args()

    a, b = solve_semi_axes(TARGET_LENGTH_M, AXIS_RATIO)
    pts = centerline_waypoints(a, b, N_WAYPOINTS)
    centerline_csv = DATA_DIR / 'waypoints_oval.csv'
    racing_csv = DATA_DIR / 'waypoints_oval_racing.csv'

    write_csv(centerline_csv, pts)
    print(f'Wrote {N_WAYPOINTS} centerline points to {centerline_csv}')
    print(f'  semi-axes a={a:.3f} m b={b:.3f} m (ratio {AXIS_RATIO})')
    print(f'  polyline length {polyline_length(pts):.3f} m (target {TARGET_LENGTH_M:.3f} m)')

    if args.update_world and WORLD_PATH.is_file():
        update_world_road(WORLD_PATH, pts)
        print(f'Updated road + spawn in {WORLD_PATH}')

    cmd = [
        sys.executable,
        str(RACING_SCRIPT),
        '--input', str(centerline_csv),
        '--output', str(racing_csv),
    ]
    if args.plot is not None:
        cmd.extend(['--plot', str(args.plot)])
    subprocess.run(cmd, check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
