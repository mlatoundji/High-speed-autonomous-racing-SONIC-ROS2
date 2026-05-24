"""Compare lap metrics against docs/BASELINE.md targets."""

import csv
from pathlib import Path

# Frozen Stanley baseline (docs/baseline_lap_times.csv, lap 1).
BASELINE_LAP_S = 190.900
BASELINE_AVG_SPEED_MPS = 3.419
BASELINE_MAX_SPEED_MPS = 5.847
BASELINE_DISTANCE_M = 652.66

# Roadmap targets from docs/BASELINE.md.
TARGET_PP_LAP_S = 120.0
TARGET_RACING_LAP_S = 90.0


def load_baseline_csv(path):
    """Load the first data row from a baseline_lap_times-style CSV."""
    path = Path(path)
    if not path.is_file():
        return None

    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            return {
                'duration_s': float(row['duration_s']),
                'avg_speed_mps': float(row['avg_speed_mps']),
                'max_speed_mps': float(row['max_speed_mps']),
                'distance_m': float(row['distance_m']),
                'session_id': row.get('session_id', ''),
                'lap_number': int(row.get('lap_number', 1)),
            }
    return None


def default_baseline_metrics(csv_path=None):
    """Return baseline dict from CSV if present, else documented constants."""
    if csv_path is not None:
        loaded = load_baseline_csv(csv_path)
        if loaded is not None:
            return loaded

    return {
        'duration_s': BASELINE_LAP_S,
        'avg_speed_mps': BASELINE_AVG_SPEED_MPS,
        'max_speed_mps': BASELINE_MAX_SPEED_MPS,
        'distance_m': BASELINE_DISTANCE_M,
        'session_id': 'baseline',
        'lap_number': 1,
    }


def _delta_str(value, reference, unit, lower_is_better=True):
    delta = value - reference
    if abs(delta) < 0.05:
        return f'{value:.2f} {unit} (≈ baseline)'
    if lower_is_better:
        tag = 'faster' if delta < 0 else 'slower'
    else:
        tag = 'higher' if delta > 0 else 'lower'
    return f'{value:.2f} {unit} ({delta:+.2f} {unit} vs baseline, {tag})'


def format_lap_comparison(duration_s, avg_speed_mps, max_speed_mps, distance_m, baseline):
    """Human-readable multi-line comparison for log output."""
    b = baseline
    lines = [
        '── Baseline comparison (docs/BASELINE.md) ──',
        f'  Lap time:   {_delta_str(duration_s, b["duration_s"], "s")}',
        f'  Avg speed:  {_delta_str(avg_speed_mps, b["avg_speed_mps"], "m/s", lower_is_better=False)}',
        f'  Peak speed: {_delta_str(max_speed_mps, b["max_speed_mps"], "m/s", lower_is_better=False)}',
        f'  Distance:   {_delta_str(distance_m, b["distance_m"], "m", lower_is_better=False)}',
    ]

    if duration_s < b['duration_s']:
        lines.append(f'  ✓ Beat Stanley baseline ({b["duration_s"]:.2f} s) by {b["duration_s"] - duration_s:.2f} s')
    else:
        lines.append(f'  ✗ Above Stanley baseline ({b["duration_s"]:.2f} s) by {duration_s - b["duration_s"]:.2f} s')

    if duration_s < TARGET_PP_LAP_S:
        lines.append(f'  ✓ Pure Pursuit roadmap target (< {TARGET_PP_LAP_S:.0f} s)')
    else:
        lines.append(f'  · PP target (< {TARGET_PP_LAP_S:.0f} s): {duration_s - TARGET_PP_LAP_S:.2f} s to go')

    if duration_s < TARGET_RACING_LAP_S:
        lines.append(f'  ✓ Racing-line roadmap target (< {TARGET_RACING_LAP_S:.0f} s)')
    else:
        lines.append(f'  · Racing target (< {TARGET_RACING_LAP_S:.0f} s): {duration_s - TARGET_RACING_LAP_S:.2f} s to go')

    return '\n'.join(lines)


def compare_sessions_table(rows, baseline):
    """Build printable rows for the offline compare script."""
    header = (
        'session', 'lap', 'duration_s', 'Δ baseline', 'avg_mps', 'max_mps',
        'dist_m', 'vs PP', 'vs racing',
    )
    table = [header]
    b_time = baseline['duration_s']

    for row in rows:
        duration = float(row['duration_s'])
        delta = duration - b_time
        vs_pp = 'OK' if duration < TARGET_PP_LAP_S else f'+{duration - TARGET_PP_LAP_S:.1f}s'
        vs_race = 'OK' if duration < TARGET_RACING_LAP_S else f'+{duration - TARGET_RACING_LAP_S:.1f}s'
        table.append((
            row.get('session_id', '')[:19],
            row.get('lap_number', ''),
            f'{duration:.3f}',
            f'{delta:+.3f}',
            row.get('avg_speed_mps', ''),
            row.get('max_speed_mps', ''),
            row.get('distance_m', ''),
            vs_pp,
            vs_race,
        ))
    return table
