#!/usr/bin/env python3
"""Compare logged laps against docs/BASELINE.md / baseline_lap_times.csv.

Usage:
  python3 scripts/compare_lap_times.py
  python3 scripts/compare_lap_times.py --csv ~/.ros/autocar_lap_times.csv
  python3 scripts/compare_lap_times.py --session latest
  python3 scripts/compare_lap_times.py --baseline docs/baseline_lap_times.csv
"""

import argparse
import csv
import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

_BASELINE_COMPARE_PATH = (
    REPO_ROOT / 'src' / 'AutoCarROS2' / 'autocar_nav' / 'autocar_nav' / 'baseline_compare.py'
)
_spec = importlib.util.spec_from_file_location('baseline_compare', _BASELINE_COMPARE_PATH)
_bc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_bc)
compare_sessions_table = _bc.compare_sessions_table
default_baseline_metrics = _bc.default_baseline_metrics
format_lap_comparison = _bc.format_lap_comparison


def load_lap_csv(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f'Lap log not found: {path}')

    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        return list(reader)


def filter_session(rows, session_arg):
    if not rows:
        return rows

    if session_arg in (None, 'all'):
        return rows

    sessions = sorted({r['session_id'] for r in rows})
    if session_arg == 'latest':
        target = sessions[-1]
    else:
        target = session_arg

    return [r for r in rows if r['session_id'] == target]


def print_table(table):
    widths = [max(len(str(row[i])) for row in table) for i in range(len(table[0]))]
    for row in table:
        line = '  '.join(str(cell).ljust(widths[i]) for i, cell in enumerate(row))
        print(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--csv',
        default=str(Path.home() / '.ros' / 'autocar_lap_times.csv'),
        help='Cumulative lap log (default: ~/.ros/autocar_lap_times.csv)',
    )
    parser.add_argument(
        '--baseline',
        default=str(REPO_ROOT / 'docs' / 'baseline_lap_times.csv'),
        help='Frozen baseline CSV (default: docs/baseline_lap_times.csv)',
    )
    parser.add_argument(
        '--session',
        default='latest',
        help='Session id to show, "latest", or "all" (default: latest)',
    )
    parser.add_argument(
        '--detail',
        action='store_true',
        help='Print full comparison block for each lap in the selection',
    )
    args = parser.parse_args()

    baseline = default_baseline_metrics(args.baseline)
    print(f'Baseline reference: {baseline["duration_s"]:.3f} s  '
          f'(avg {baseline["avg_speed_mps"]:.3f} m/s, '
          f'max {baseline["max_speed_mps"]:.3f} m/s, '
          f'dist {baseline["distance_m"]:.2f} m)')
    print(f'  Source: {args.baseline}\n')

    try:
        rows = load_lap_csv(args.csv)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        print('Run a race launch first (e.g. race_mpc_launch.py) to record laps.', file=sys.stderr)
        return 1

    if not rows:
        print(f'No laps in {args.csv}')
        return 0

    selected = filter_session(rows, args.session)
    if not selected:
        print(f'No rows for session={args.session!r}')
        return 1

    if args.session == 'all':
        print(f'All sessions ({len(selected)} laps):\n')
    else:
        sid = selected[0]['session_id']
        stacks = sorted({r.get('stack', '?') for r in selected})
        print(f'Session {sid}  stacks={stacks}  ({len(selected)} lap(s)):\n')

    print_table(compare_sessions_table(selected, baseline))

    if args.detail:
        print()
        for row in selected:
            lap = int(row['lap_number'])
            stack = row.get('stack', '?')
            print(f'--- Lap {lap} ({stack}) ---')
            print(format_lap_comparison(
                float(row['duration_s']),
                float(row['avg_speed_mps']),
                float(row['max_speed_mps']),
                float(row['distance_m']),
                baseline,
            ))
            print()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
