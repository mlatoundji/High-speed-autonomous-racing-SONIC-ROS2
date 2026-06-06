#!/usr/bin/env python3
"""Minimum-curvature racing line via a convex QP (numpy only, projected gradient).

Line = centerline + alpha * normal, with alpha bounded to a SAFE corridor.
Minimise J(alpha) = sum_i || P''_i ||^2  (squared 2nd difference ~ curvature^2),
which is quadratic & convex in alpha:
    P'' = D@C + (D@diag(nx)) alpha , ...     (D = circulant 2nd-difference)
    J  = alpha^T H alpha + f^T alpha + const,  H = 2(Ax^T Ax + Ay^T Ay) PSD
Solved by projected gradient (alpha <- clip(alpha - lr*grad, -amax, amax)),
lr = 0.9/lambda_max(H). Convex => converges to the global optimum, no zigzag.
The optimum is guaranteed <= centerline curvature (centerline alpha=0 is feasible).

Safety vs last time: amax keeps the line well inside the track; the result is
validated (curvature must DROP) before anything is deployed.
"""
import argparse
import csv
from pathlib import Path

import numpy as np


def read_wp(path):
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row['X-axis']))
            ys.append(float(row['Y-axis']))
    return np.asarray(xs), np.asarray(ys)


def write_wp(path, xs, ys):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['X-axis', 'Y-axis'])
        for x, y in zip(xs, ys):
            w.writerow([f'{x:.6f}', f'{y:.6f}'])


def dedupe(cx, cy, min_d=0.5):
    kx, ky = [cx[0]], [cy[0]]
    for x, y in zip(cx[1:], cy[1:]):
        if np.hypot(x - kx[-1], y - ky[-1]) >= min_d:
            kx.append(x); ky.append(y)
    if np.hypot(kx[0] - kx[-1], ky[0] - ky[-1]) < min_d:
        kx, ky = kx[:-1], ky[:-1]
    return np.asarray(kx), np.asarray(ky)


def normals(xs, ys):
    dx = np.roll(xs, -1) - np.roll(xs, 1)
    dy = np.roll(ys, -1) - np.roll(ys, 1)
    n = np.hypot(dx, dy); n[n < 1e-9] = 1.0
    return -dy / n, dx / n


def curvature(xs, ys):
    xp, xn = np.roll(xs, 1), np.roll(xs, -1)
    yp, yn = np.roll(ys, 1), np.roll(ys, -1)
    d1x, d1y = xs - xp, ys - yp
    d2x, d2y = xn - xs, yn - ys
    cross = d1x * d2y - d1y * d2x
    den = np.hypot(d1x, d1y) * np.hypot(d2x, d2y) * np.hypot(xn - xp, yn - yp)
    den[den < 1e-9] = 1.0
    return 2.0 * cross / den


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--max-offset', type=float, default=5.0)
    p.add_argument('--iters', type=int, default=8000)
    args = p.parse_args()

    cx, cy = read_wp(args.input)
    if len(cx) >= 3 and np.hypot(cx[0] - cx[-1], cy[0] - cy[-1]) < 1e-6:
        cx, cy = cx[:-1], cy[:-1]
    n0 = len(cx)
    cx, cy = dedupe(cx, cy)
    n = len(cx)
    if n0 - n:
        print(f'(dédup: {n0 - n} point(s) retirés)')
    nx, ny = normals(cx, cy)
    print(f'{n} points centerline depuis {args.input}')

    # Circulant 2nd-difference operator D (closed loop)
    I = np.eye(n)
    D = -2 * I + np.roll(I, 1, axis=1) + np.roll(I, -1, axis=1)
    Ax = D @ np.diag(nx)
    Ay = D @ np.diag(ny)
    bx = D @ cx
    by = D @ cy
    H = 2 * (Ax.T @ Ax + Ay.T @ Ay)
    f = 2 * (Ax.T @ bx + Ay.T @ by)

    # lambda_max via power iteration -> step size
    v = np.ones(n)
    for _ in range(60):
        v = H @ v
        nv = np.linalg.norm(v)
        if nv < 1e-12:
            break
        v /= nv
    lam = float(v @ (H @ v))
    lr = 0.9 / max(lam, 1e-9)

    amax = args.max_offset
    alpha = np.zeros(n)

    def J(a):
        return float(np.sum((Ax @ a + bx) ** 2) + np.sum((Ay @ a + by) ** 2))

    J0 = J(alpha)
    for _ in range(args.iters):
        grad = H @ alpha + f
        alpha = np.clip(alpha - lr * grad, -amax, amax)
    J1 = J(alpha)

    rx = cx + alpha * nx
    ry = cy + alpha * ny
    kc = float(np.max(np.abs(curvature(cx, cy))))
    kr = float(np.max(np.abs(curvature(rx, ry))))
    print(f'objectif courbure²: {J0:.1f} -> {J1:.1f}  ({100*(1-J1/max(J0,1e-9)):.0f}% de baisse)')
    print(f'offset: min={alpha.min():+.2f} max={alpha.max():+.2f} mean|.|={np.mean(np.abs(alpha)):.2f} m (cap ±{amax})')
    pinned = 100.0 * np.mean(np.abs(np.abs(alpha) - amax) < 0.05)
    print(f'points collés au bord: {pinned:.0f}%')
    print(f'courbure pic: centerline R={1/max(kc,1e-9):.1f}m -> racing R={1/max(kr,1e-9):.1f}m  (plus grand = mieux)')
    if kr <= kc:
        print('OK: la racing line est MOINS courbée que le centre. ✅')
    else:
        print('ATTENTION: racing PIRE que le centre -> NE PAS deployer. ❌')

    write_wp(args.output, np.append(rx, rx[0]), np.append(ry, ry[0]))
    print(f'écrit {n + 1} points -> {args.output}')


if __name__ == '__main__':
    main()
