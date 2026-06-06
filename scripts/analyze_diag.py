#!/usr/bin/env python3
"""Analyse diag_metrics.csv and ATTRIBUTE the speed loss to each parameter.

Two distinct speed-reduction mechanisms exist:
  A) Velocity planner braking BEFORE the corner (curvature-based)
        -> target_vel drops below cruise_velocity
        -> driven by: max_lateral_accel, curvature_lookahead, min_curvature
  B) Tracker "slow down when far from path" DURING the corner
        -> throttle_cmd drops below target_vel (err_scale crush)
        -> driven by: lateral_soft, heading_soft  (root cause = tracking error)

Plus: steering saturation (steering_limits) and rate-limit (steering_rate_limit).

Usage: python3 scripts/analyze_diag.py [csv] [navigation_params.yaml]
"""
import csv
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/diag_metrics.csv'
YAML = sys.argv[2] if len(sys.argv) > 2 else (
    'install/autocar_nav_pure_pursuit/share/autocar_nav_pure_pursuit/config/navigation_params.yaml')


def load_params(path):
    try:
        import yaml
        d = yaml.safe_load(open(path))
        lp = d['local_planner']['ros__parameters']
        pt = d['path_tracker']['ros__parameters']
        return {**lp, **pt}
    except Exception as e:
        print(f'(params yaml non lus: {e})')
        return {}


P = load_params(YAML)
rows = []
with open(CSV) as f:
    for r in csv.DictReader(f):
        try:
            rows.append({k: float(v) for k, v in r.items()})
        except ValueError:
            pass
n = len(rows)
if n == 0:
    print('CSV vide:', CSV)
    sys.exit(1)


def g(k, d=None):
    return P.get(k, d)


cruise = float(g('cruise_velocity', 8.0))
lat_soft = float(g('lateral_soft', 4.0))
steer_cap = float(g('steering_limits', 0.95))
rate_cap = float(g('steering_rate_limit', 0.0))


def pct(cond, data=rows):
    return 100.0 * sum(1 for r in data if cond(r)) / len(data)


print('=== PARAMETRES ACTIFS (', YAML.split('/')[-1], ') ===')
print(f"  Planner : cruise_velocity={g('cruise_velocity')}  max_lateral_accel={g('max_lateral_accel')}"
      f"  min_curvature={g('min_curvature')}  curvature_lookahead={g('curvature_lookahead')}"
      f"  accel/decel={g('accel_rate')}/{g('decel_rate')}")
print(f"  Tracker : lookahead gain/min/max={g('lookahead_gain')}/{g('lookahead_min')}/{g('lookahead_max')}"
      f"  steering_limits={g('steering_limits')}  steering_rate_limit={g('steering_rate_limit')}")
print(f"            lateral_soft={g('lateral_soft')}  heading_soft={g('heading_soft')}  velocity_gain={g('velocity_gain')}")

spd = [r['speed_mps'] for r in rows]
dur = rows[-1]['t_s'] - rows[0]['t_s']
print(f'\n=== VITESSE ({n} ech., {dur:.0f}s) ===')
print(f'  min={min(spd):.2f}  moy={sum(spd)/n:.2f}  max={max(spd):.2f} m/s')
print(f'  temps quasi a l arret (<0.5 m/s) : {pct(lambda r: r["speed_mps"]<0.5):.0f}%')
print(f'  temps lent (<3 m/s)              : {pct(lambda r: r["speed_mps"]<3.0):.0f}%')

# Mechanism A: planner braking (target_vel < cruise)
print('\n=== A) FREIN DU PLANNER (avant le virage) ===')
print(f'  target_vel min={min(r["target_vel_mps"] for r in rows):.2f}  moy={sum(r["target_vel_mps"] for r in rows)/n:.2f} m/s  (cruise={cruise})')
print(f'  temps ou target_vel < 90% cruise (=freinage courbure) : {pct(lambda r: r["target_vel_mps"] < 0.9*cruise):.0f}%')
print(f'  -> piloté par: max_lateral_accel, curvature_lookahead, min_curvature')

# Mechanism B: lateral_soft crush (throttle << target_vel)
def ratio(r):
    return r['throttle_cmd'] / r['target_vel_mps'] if r['target_vel_mps'] > 0.3 else 1.0
crush = [r for r in rows if r['target_vel_mps'] > 0.3 and ratio(r) < 0.5]
print('\n=== B) FREIN "LOIN DE LA LIGNE" (lateral_soft, dans le virage) ===')
print(f'  temps ou throttle < 50% de target_vel (vitesse écrasée) : {100.0*len(crush)/n:.0f}%')
print(f'  temps ou |lateral_error| > lateral_soft ({lat_soft}) : {pct(lambda r: abs(r["lateral_error_m"])>lat_soft):.0f}%')
print(f'  |lateral_error| moy={sum(abs(r["lateral_error_m"]) for r in rows)/n:.2f}  max={max(abs(r["lateral_error_m"]) for r in rows):.2f} m')
print(f'  -> piloté par: lateral_soft / heading_soft (et racine = erreur de suivi)')

# Steering
print('\n=== C) BRAQUAGE ===')
print(f'  temps saturé (|steer|>=0.83) : {pct(lambda r: abs(r["steer_cmd_rad"])>=0.83):.0f}%')
print(f'  temps bridé (|rate|>=0.95*cap {rate_cap}) : {pct(lambda r: rate_cap>0 and abs(r["steer_rate_radps"])>=0.95*rate_cap):.0f}%')
print(f'  -> piloté par: steering_limits, steering_rate_limit, lookahead_*')

# Verdict
print('\n=== VERDICT (quel param changer dans le yaml f1) ===')
a = pct(lambda r: r['target_vel_mps'] < 0.9*cruise)
b = 100.0*len(crush)/n
off = pct(lambda r: abs(r['lateral_error_m'])>lat_soft)
sat = pct(lambda r: abs(r['steer_cmd_rad'])>=0.83)
if a >= 25:
    print(f'  A) planner freine {a:.0f}% du temps -> si trop tôt/trop fort:')
    print('     . ↑ max_lateral_accel (virages plus rapides)')
    print('     . ↓ curvature_lookahead (freine moins tôt)')
if b >= 15 or off >= 15:
    print(f'  B) vitesse écrasée par lateral_soft {b:.0f}% (off-line {off:.0f}%) -> la voiture part large:')
    print('     . racine: réduire l erreur de suivi (lookahead, ou racing line plus suivable)')
    print('     . palliatif: ↑ lateral_soft (moins brider) — mais traite le symptôme')
if sat >= 25:
    print(f'  C) braquage saturé {sat:.0f}% -> virage trop serré (ligne + large / ↑ max_steer)')
if a < 25 and b < 15 and off < 15 and sat < 25:
    print('  Aucun mécanisme dominant -> regarder le time-series brut.')
