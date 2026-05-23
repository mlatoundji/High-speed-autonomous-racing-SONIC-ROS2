#!/usr/bin/env python3
"""Generate a racing line from the centerline waypoints.

Approach (deliberately simple, "inside-the-turn" heuristic):
  1. Read the centerline waypoints (closed loop).
  2. Compute per-waypoint curvature and unit left-normal.
  3. Smooth the curvature so the new line transitions cleanly between
     turns and straights.
  4. Scale the smoothed curvature so the tightest turn gets exactly
     `target_offset` metres of lateral displacement, then offset every
     waypoint along its left-normal in proportion to its (signed)
     curvature: positive curvature (CCW turn) -> the inside is left,
     so we move left; negative curvature (CW) -> we move right. The
     formula is `offset = +alpha * kappa_smoothed` where alpha is the
     scaling factor.
  5. Clip the offset to `max_offset` so the new line never gets within
     `track_half_width - max_offset` of the track edge.
  6. Smooth the offset itself once more to avoid sharp jumps between
     adjacent waypoints.
  7. Write the racing line CSV in the same format as `waypoints.csv`
     so the global planner can load it interchangeably.

This is NOT a full min-curvature QP, but on the 46-point closed loop
of race_circuit.world it produces a sensible "cut the apex" line
which should beat the centerline lap time without violating the road
boundaries.

Usage:
    python3 scripts/generate_racing_line.py
    python3 scripts/generate_racing_line.py --target-offset 5.0 --plot docs/figures/racing_line.png
"""

import argparse
import csv
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IN = REPO_ROOT / 'src' / 'AutoCarROS2' / 'autocar_nav' / 'data' / 'waypoints.csv'
DEFAULT_OUT = REPO_ROOT / 'src' / 'AutoCarROS2' / 'autocar_nav' / 'data' / 'waypoints_racing.csv'
DEFAULT_PLOT = REPO_ROOT / 'docs' / 'figures' / 'racing_line.png'

# race_circuit.world is 16 m wide, so the centerline-to-edge distance
# is 8 m. We keep a safety margin so a 2 m wide car doesn't kiss the
# hay bales on the outside or run off into the infield on the inside.
TRACK_HALF_WIDTH_M = 8.0
DEFAULT_MAX_OFFSET_M = 6.0


def read_waypoints(path: Path):
    xs, ys = [], []
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['X-axis']))
            ys.append(float(row['Y-axis']))
    return np.asarray(xs), np.asarray(ys)


def write_waypoints(path: Path, xs, ys):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['X-axis', 'Y-axis'])
        for x, y in zip(xs, ys):
            w.writerow([f'{x:.6f}', f'{y:.6f}'])


def closed_tangent_normal(xs, ys):
    """Per-point unit tangent and unit left-normal, assuming a closed loop."""
    dx = np.roll(xs, -1) - np.roll(xs, 1)
    dy = np.roll(ys, -1) - np.roll(ys, 1)
    n = np.hypot(dx, dy)
    n[n < 1e-9] = 1.0
    tx, ty = dx / n, dy / n
    # Left normal in 2D = rotate tangent +90 deg (CCW)
    nx_l, ny_l = -ty, tx
    return tx, ty, nx_l, ny_l


def closed_curvature(xs, ys):
    """Signed curvature at each point, closed loop, via three-point formula."""
    x_prev = np.roll(xs, 1)
    x_next = np.roll(xs, -1)
    y_prev = np.roll(ys, 1)
    y_next = np.roll(ys, -1)

    dx1 = xs - x_prev
    dy1 = ys - y_prev
    dx2 = x_next - xs
    dy2 = y_next - ys

    cross = dx1 * dy2 - dy1 * dx2
    n1 = np.hypot(dx1, dy1)
    n2 = np.hypot(dx2, dy2)
    chord = np.hypot(x_next - x_prev, y_next - y_prev)
    denom = n1 * n2 * chord
    denom[denom < 1e-9] = 1.0
    return 2.0 * cross / denom


def laplacian_smooth(a, iterations, closed=True):
    a = a.copy().astype(float)
    for _ in range(iterations):
        if closed:
            prev = np.roll(a, 1)
            nxt = np.roll(a, -1)
        else:
            prev = np.concatenate([[a[0]], a[:-1]])
            nxt = np.concatenate([a[1:], [a[-1]]])
        a = 0.25 * prev + 0.5 * a + 0.25 * nxt
    return a


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, default=DEFAULT_IN)
    p.add_argument('--output', type=Path, default=DEFAULT_OUT)
    p.add_argument('--target-offset', type=float, default=4.0,
                   help='Max lateral displacement (m) at the tightest turn')
    p.add_argument('--max-offset', type=float, default=DEFAULT_MAX_OFFSET_M,
                   help='Hard clip on lateral offset (m), for safety margin')
    p.add_argument('--smoothing', type=int, default=30,
                   help='Laplacian smoothing iterations (applied twice)')
    p.add_argument('--plot', type=Path, default=DEFAULT_PLOT,
                   help='PNG comparison plot path. Pass empty string to skip.')
    args = p.parse_args()

    xs, ys = read_waypoints(args.input)
    n = len(xs)
    print(f'Loaded {n} centerline waypoints from {args.input}')

    _, _, nx_l, ny_l = closed_tangent_normal(xs, ys)
    kappa = closed_curvature(xs, ys)
    kappa_s = laplacian_smooth(kappa, iterations=args.smoothing, closed=True)

    max_k = float(np.max(np.abs(kappa_s)))
    if max_k < 1e-9:
        print('Curvature is ~ 0 everywhere -> racing line == centerline.')
        offset = np.zeros_like(kappa_s)
    else:
        # Inside-the-turn: positive curvature (CCW) -> move +left = inside;
        # negative -> move right. Scale so the tightest turn hits target_offset.
        offset = kappa_s * (args.target_offset / max_k)

    offset = laplacian_smooth(offset, iterations=args.smoothing, closed=True)
    offset = np.clip(offset, -args.max_offset, args.max_offset)

    new_xs = xs + offset * nx_l
    new_ys = ys + offset * ny_l

    write_waypoints(args.output, new_xs, new_ys)
    print(f'Wrote {n} racing-line waypoints to {args.output}')
    print(f'Lateral offset stats: min={offset.min():+.2f} m, '
          f'max={offset.max():+.2f} m, mean abs={np.mean(np.abs(offset)):.2f} m')
    print(f'Curvature stats: peak {max_k:.4f} 1/m (turning radius {1.0/max_k:.1f} m)')

    if str(args.plot):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            # Catch BOTH ImportError and the numpy/matplotlib ABI mismatch
            # that bites some apt-installed pairs ("_ARRAY_API not found").
            print(f'matplotlib unavailable ({type(e).__name__}: {e}); skipping plot.')
            return

        # Close the loops for visual continuity
        xs_c = np.append(xs, xs[0])
        ys_c = np.append(ys, ys[0])
        nxs_c = np.append(new_xs, new_xs[0])
        nys_c = np.append(new_ys, new_ys[0])

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(xs_c, ys_c, 'b.-', label='Centerline (waypoints.csv)',
                alpha=0.7, markersize=5, linewidth=1.2)
        ax.plot(nxs_c, nys_c, 'r.-', label='Racing line (waypoints_racing.csv)',
                alpha=0.85, markersize=5, linewidth=1.2)
        # Sketch the road edges using the left-normal of the centerline.
        edge_l_x = xs + TRACK_HALF_WIDTH_M * nx_l
        edge_l_y = ys + TRACK_HALF_WIDTH_M * ny_l
        edge_r_x = xs - TRACK_HALF_WIDTH_M * nx_l
        edge_r_y = ys - TRACK_HALF_WIDTH_M * ny_l
        ax.plot(np.append(edge_l_x, edge_l_x[0]), np.append(edge_l_y, edge_l_y[0]),
                'k--', alpha=0.3, linewidth=0.8, label='Track edges')
        ax.plot(np.append(edge_r_x, edge_r_x[0]), np.append(edge_r_y, edge_r_y[0]),
                'k--', alpha=0.3, linewidth=0.8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Racing line vs centerline\n'
                     f'(target_offset={args.target_offset} m, max_offset={args.max_offset} m, '
                     f'smoothing={args.smoothing} iter)')
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        plt.close(fig)
        print(f'Saved comparison plot to {args.plot}')


if __name__ == '__main__':
    main()
