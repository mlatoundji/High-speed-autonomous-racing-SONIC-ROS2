#!/usr/bin/env python3
"""Lisse une racing line pour enlever les kinks ponctuels + espacement uniforme,
SANS sortir de la piste. Valide la courbure AVANT/APRES (rien n'est deploye tant
que la courbure n'a pas baisse et que la ligne reste dans le couloir).

Usage:
  python3 smooth_racing_line.py --input waypoints_f1_racing.csv \
      --centerline waypoints_f1.csv --output waypoints_f1_racing_smooth.csv \
      --alpha 0.3 --iters 5 --max-dev 1.5 --half-width 8.0
"""
import argparse
import csv
import math

import numpy as np


def read_xy(p):
    xs, ys = [], []
    for r in csv.DictReader(open(p)):
        xs.append(float(r['X-axis'])); ys.append(float(r['Y-axis']))
    xs, ys = np.asarray(xs), np.asarray(ys)
    if len(xs) >= 3 and np.hypot(xs[0]-xs[-1], ys[0]-ys[-1]) < 1e-6:
        xs, ys = xs[:-1], ys[:-1]
    return xs, ys


def curvature(xs, ys):
    xp, xn = np.roll(xs, 1), np.roll(xs, -1)
    yp, yn = np.roll(ys, 1), np.roll(ys, -1)
    d1x, d1y = xs-xp, ys-yp
    d2x, d2y = xn-xs, yn-ys
    cr = d1x*d2y - d1y*d2x
    den = np.hypot(d1x, d1y)*np.hypot(d2x, d2y)*np.hypot(xn-xp, yn-yp)
    den[den < 1e-9] = 1.0
    return 2*cr/den


def resample_uniform(xs, ys, n):
    pts = np.column_stack([xs, ys])
    pts = np.vstack([pts, pts[0]])  # close
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    cum = np.concatenate([[0], np.cumsum(seg)])
    total = cum[-1]
    s = np.linspace(0, total, n, endpoint=False)
    rx = np.interp(s, cum, pts[:, 0])
    ry = np.interp(s, cum, pts[:, 1])
    return rx, ry


def stats(xs, ys, label):
    k = np.abs(curvature(xs, ys))
    sp = np.hypot(np.roll(xs, -1)-xs, np.roll(ys, -1)-ys)
    rmin = 1/max(k.max(), 1e-9)
    print(f'{label}: R_mini={rmin:.2f}m  courbure^2={np.sum(k**2):.4f}  '
          f'espac[{sp.min():.2f}-{sp.max():.2f}]')
    return rmin


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', required=True)
    p.add_argument('--centerline', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--alpha', type=float, default=0.3)
    p.add_argument('--iters', type=int, default=5)
    p.add_argument('--max-dev', type=float, default=1.5, help='deplacement max vs ligne origine (m)')
    p.add_argument('--half-width', type=float, default=8.0, help='demi-largeur piste (m)')
    a = p.parse_args()

    ox, oy = read_xy(a.input)
    cx, cy = read_xy(a.centerline)
    n = len(ox)
    print(f'{n} points')
    r0 = stats(ox, oy, 'AVANT ')

    xs, oy0 = ox.copy(), oy.copy()
    ys = oy.copy()
    for _ in range(a.iters):
        xp, xn = np.roll(xs, 1), np.roll(xs, -1)
        yp, yn = np.roll(ys, 1), np.roll(ys, -1)
        xs = xs + a.alpha * (0.5*(xp+xn) - xs)
        ys = ys + a.alpha * (0.5*(yp+yn) - ys)
        # contrainte: ne pas s'eloigner de la ligne d'origine de > max_dev
        dx, dy = xs-ox, ys-oy
        d = np.hypot(dx, dy)
        over = d > a.max_dev
        xs[over] = ox[over] + dx[over]/d[over]*a.max_dev
        ys[over] = oy[over] + dy[over]/d[over]*a.max_dev

    xs, ys = resample_uniform(xs, ys, n)
    r1 = stats(xs, ys, 'APRES ')

    # garde-fou piste: distance max au centerline le plus proche
    maxoff = 0.0
    for x, y in zip(xs, ys):
        maxoff = max(maxoff, np.min(np.hypot(cx-x, cy-y)))
    print(f'offset max vs centerline = {maxoff:.2f} m (limite piste = {a.half_width:.1f} m) '
          f'-> {"OK dans la piste" if maxoff < a.half_width else "DEHORS !!"}')

    # Deploiement: on VEUT R_mini plus GRAND (virage moins serre) et rester en piste.
    if r1 <= r0 + 0.05:
        print(f'\n>>> R_mini n a pas augmente ({r0:.2f}->{r1:.2f}) — lissage inutile, NE PAS deployer')
    elif maxoff >= a.half_width:
        print('\n>>> SORT DE LA PISTE — NE PAS deployer')
    else:
        with open(a.output, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['X-axis', 'Y-axis'])
            for x, y in zip(xs, ys):
                w.writerow([f'{x:.6f}', f'{y:.6f}'])
            w.writerow([f'{xs[0]:.6f}', f'{ys[0]:.6f}'])  # close loop
        print(f'\n>>> OK: R_mini {r0:.2f}->{r1:.2f}m, dans la piste. Ecrit -> {a.output}')


if __name__ == '__main__':
    main()
