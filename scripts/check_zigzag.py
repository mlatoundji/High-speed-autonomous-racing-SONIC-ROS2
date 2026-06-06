#!/usr/bin/env python3
"""Detect steering zigzag on straights: steering activity while going fast."""
import csv
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/diag_metrics.csv'
FAST = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0  # m/s = straight

rows = []
with open(PATH) as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k: float(v) for k, v in r.items()})
        except ValueError:
            pass

fast = [r for r in rows if r['speed_mps'] >= FAST]
slow = [r for r in rows if r['speed_mps'] < FAST]
print(f'{len(rows)} ech. | rapides(>={FAST}m/s)={len(fast)} ({100*len(fast)/len(rows):.0f}%) | lents={len(slow)}')


def sign_changes(data):
    sc = 0
    prev = 0.0
    for r in data:
        s = r['steer_cmd_rad']
        if prev != 0 and ((s > 0.02 and prev < -0.02) or (s < -0.02 and prev > 0.02)):
            sc += 1
        if abs(s) > 0.02:
            prev = s
    return sc


def stats(data, label):
    if not data:
        print(f'{label}: (vide)')
        return
    n = len(data)
    asr = sum(abs(r['steer_rate_radps']) for r in data) / n
    asc = sum(abs(r['steer_cmd_rad']) for r in data) / n
    active = 100.0 * sum(1 for r in data if abs(r['steer_rate_radps']) > 2.0) / n
    sc = sign_changes(data)
    dur = data[-1]['t_s'] - data[0]['t_s'] if n > 1 else 1
    print(f'{label}:')
    print(f'   |steer_cmd| moy = {asc:.3f} rad   |steer_rate| moy = {asr:.2f} rad/s')
    print(f'   braquage actif (|rate|>2) = {active:.0f}% du temps')
    print(f'   changements de signe du braquage = {sc}  (gauche<->droite)')


print('\n--- SUR LES DROITES (rapide) : zigzag ? ---')
stats(fast, 'droites')
print('\n--- en virage (lent), pour comparer ---')
stats(slow, 'virages')
