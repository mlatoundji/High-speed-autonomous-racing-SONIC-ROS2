#!/usr/bin/env python3
"""Experimental harness for the SONIC-ROS2 racing project.

Reads a matrix of (controller, profile, latency_ms, odom_noise_std, n_laps)
configurations, launches the race simulation once per row, waits for the
expected number of lap rows to appear in the metrics CSV, then aggregates
the results into a summary file and produces a few plots.

Does NOT modify the running CSV; the lap_timer node is the sole writer.

Usage:
    python3 scripts/bench.py --matrix scripts/matrices/quick.yaml
    python3 scripts/bench.py --quick            # builtin 1-config smoke test
    python3 scripts/bench.py --matrix ... --dry-run

The script expects to run from a shell where ROS 2 is on disk (it sources
/opt/ros/humble/setup.bash and the local install itself for each child).
"""

import argparse
import csv
import json
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = Path.home() / '.ros' / 'autocar_lap_times.csv'
SNAPSHOTS_DIR = REPO_ROOT / 'docs' / 'snapshots'
FIGURES_DIR = REPO_ROOT / 'docs' / 'figures'

# Default per-lap timeout. If a lap takes more than this in wall time, we
# assume the run is broken (controller diverged, Gazebo froze, etc.) and abort.
LAP_TIMEOUT_S = 600.0

# Headroom after killing a run before launching the next. Gazebo can take a
# couple of seconds to free its ports.
COOLDOWN_S = 3.0

BUILTIN_QUICK_MATRIX = [
    # 1 config, 1 lap, no warmup discard. Just proves bench.py can drive
    # a run end-to-end and produce an aggregated row.
    {'controller': 'stanley', 'profile': 'default',
     'latency_ms': 0, 'odom_noise_std': 0.0, 'n_laps': 1, 'n_warmup': 0},
]


# ----------------------------------------------------------------------
# Matrix loading
# ----------------------------------------------------------------------
def load_matrix(path: Path):
    if not path.exists():
        sys.exit(f'matrix file not found: {path}')
    text = path.read_text()
    if path.suffix.lower() in ('.yaml', '.yml'):
        try:
            import yaml
        except ImportError:
            sys.exit('PyYAML not installed (pip install pyyaml) -- or use a .json matrix.')
        return yaml.safe_load(text)
    if path.suffix.lower() == '.json':
        return json.loads(text)
    sys.exit(f'unsupported matrix extension: {path.suffix}')


def normalize_config(cfg: dict, default_n_warmup: int) -> dict:
    n_laps = int(cfg.get('n_laps', 3))
    # Per-row override of n_warmup if specified, otherwise CLI default.
    # Cap so at least one measured lap survives, otherwise the run produces
    # no aggregated stats and we waste 8 minutes.
    requested_warmup = int(cfg.get('n_warmup', default_n_warmup))
    n_warmup = max(0, min(requested_warmup, n_laps - 1))
    if n_warmup != requested_warmup:
        print(f'[bench] note: n_warmup capped from {requested_warmup} to {n_warmup} (n_laps={n_laps})')
    return {
        'controller': str(cfg.get('controller', 'stanley')),
        'profile': str(cfg.get('profile', 'default')),
        'latency_ms': int(cfg.get('latency_ms', 0)),
        'odom_noise_std': float(cfg.get('odom_noise_std', 0.0)),
        'n_laps': n_laps,
        'n_warmup': n_warmup,
    }


# ----------------------------------------------------------------------
# Process control
# ----------------------------------------------------------------------
def kill_sim():
    for cmd in (
        ['killall', '-9', 'gzserver'],
        ['killall', '-9', 'gzclient'],
        ['killall', '-9', 'gazebo'],
        ['pkill', '-9', '-f', 'ros2 launch'],
    ):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def launch_command(cfg: dict) -> str:
    return (
        'source /opt/ros/humble/setup.bash && '
        'source install/setup.bash && '
        'ros2 launch launches race_launch.py '
        f'controller:={cfg["controller"]} '
        f'profile:={cfg["profile"]} '
        f'latency_ms:={cfg["latency_ms"]} '
        f'odom_noise_std:={cfg["odom_noise_std"]}'
    )


def count_csv_data_rows() -> int:
    if not CSV_PATH.exists():
        return 0
    with CSV_PATH.open() as f:
        return max(0, sum(1 for _ in f) - 1)  # minus header


def read_session_rows(session_id: str) -> list:
    if not CSV_PATH.exists():
        return []
    with CSV_PATH.open(newline='') as f:
        reader = csv.DictReader(f)
        return [r for r in reader if r.get('session_id') == session_id]


def read_latest_session_id_after(min_data_rows: int):
    """Return the session_id of the most recent row whose index >= min_data_rows."""
    if not CSV_PATH.exists():
        return None
    with CSV_PATH.open(newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if len(rows) <= min_data_rows:
        return None
    return rows[-1].get('session_id')


# ----------------------------------------------------------------------
# One run
# ----------------------------------------------------------------------
def run_one(cfg: dict) -> dict:
    kill_sim()
    time.sleep(COOLDOWN_S)

    rows_before = count_csv_data_rows()
    expected_new = cfg['n_laps']
    deadline = time.time() + LAP_TIMEOUT_S * expected_new

    cmd = launch_command(cfg)
    proc = subprocess.Popen(
        ['bash', '-lc', cmd],
        cwd=REPO_ROOT,
        preexec_fn=os.setsid,  # own process group so we can SIGTERM everything
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    print(f'[bench] launched pid={proc.pid} -- waiting for {expected_new} lap(s)...')

    timed_out = False
    while True:
        time.sleep(2.0)
        if proc.poll() is not None:
            print(f'[bench]   launch process exited unexpectedly (code {proc.returncode}).')
            break
        current = count_csv_data_rows()
        if current >= rows_before + expected_new:
            print(f'[bench]   {current - rows_before} lap(s) recorded, stopping run.')
            break
        if time.time() > deadline:
            print('[bench]   TIMEOUT, killing run.')
            timed_out = True
            break

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    kill_sim()

    session_id = read_latest_session_id_after(rows_before)
    rows = read_session_rows(session_id) if session_id else []
    return {'config': cfg, 'session_id': session_id, 'rows': rows, 'timed_out': timed_out}


# ----------------------------------------------------------------------
# Aggregation
# ----------------------------------------------------------------------
def aggregate(results) -> list:
    table = []
    for r in results:
        cfg = r['config']
        rows = r['rows'][cfg['n_warmup']:]  # drop the per-config n_warmup laps
        durations = [float(x['duration_s']) for x in rows]

        row = {
            'controller': cfg['controller'],
            'profile': cfg['profile'],
            'latency_ms': cfg['latency_ms'],
            'odom_noise_std': cfg['odom_noise_std'],
            'session_id': r['session_id'],
            'n_laps_recorded': len(r['rows']),
            'n_measured': len(rows),
            'timed_out': r['timed_out'],
        }
        if durations:
            row.update({
                'median_lap_s': statistics.median(durations),
                'std_lap_s': statistics.stdev(durations) if len(durations) > 1 else 0.0,
                'min_lap_s': min(durations),
                'max_lap_s': max(durations),
                'median_lat_rms': statistics.median(float(x['lateral_error_rms']) for x in rows),
                'max_lat_max': max(float(x['lateral_error_max']) for x in rows),
                'sum_offtrack': sum(int(x['offtrack_events']) for x in rows),
            })
        else:
            row.update({
                'median_lap_s': None, 'std_lap_s': None,
                'min_lap_s': None, 'max_lap_s': None,
                'median_lat_rms': None, 'max_lat_max': None,
                'sum_offtrack': None,
            })
        table.append(row)
    return table


def write_summary(table, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not table:
        out_path.write_text('')
        return
    fieldnames = list(table[0].keys())
    with out_path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in table:
            w.writerow(row)


def plot_summary(table, out_dir: Path, tag: str):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('[bench] matplotlib not available, skipping plots.')
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    valid = [r for r in table if r.get('median_lap_s') is not None]
    if not valid:
        print('[bench] no valid rows, skipping plots.')
        return

    labels = [f"{r['controller']}/{r['profile']}" for r in valid]
    medians = [r['median_lap_s'] for r in valid]
    stds = [r['std_lap_s'] for r in valid]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(range(len(labels)), medians, yerr=stds, capsize=5)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.set_ylabel('Lap time (s) -- median +/- std')
    ax.set_title(f'bench {tag}: lap time by controller/profile')
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f'bench_{tag}_lap_by_config.png', dpi=120)
    plt.close(fig)

    # If the matrix varies latency, scatter that.
    latencies = sorted({r['latency_ms'] for r in valid})
    if len(latencies) > 1:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for ctrl in sorted({r['controller'] for r in valid}):
            xs, ys = [], []
            for r in valid:
                if r['controller'] == ctrl:
                    xs.append(r['latency_ms'])
                    ys.append(r['median_lap_s'])
            ax.plot(xs, ys, 'o-', label=ctrl)
        ax.set_xlabel('Artificial latency (ms)')
        ax.set_ylabel('Median lap time (s)')
        ax.set_title(f'bench {tag}: lap time vs latency')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f'bench_{tag}_lap_vs_latency.png', dpi=120)
        plt.close(fig)

    noises = sorted({r['odom_noise_std'] for r in valid})
    if len(noises) > 1:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for ctrl in sorted({r['controller'] for r in valid}):
            xs, ys = [], []
            for r in valid:
                if r['controller'] == ctrl:
                    xs.append(r['odom_noise_std'])
                    ys.append(r['median_lap_s'])
            ax.plot(xs, ys, 'o-', label=ctrl)
        ax.set_xlabel('Odometry noise std (m)')
        ax.set_ylabel('Median lap time (s)')
        ax.set_title(f'bench {tag}: lap time vs odom noise')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f'bench_{tag}_lap_vs_odom_noise.png', dpi=120)
        plt.close(fig)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--matrix', type=Path, help='YAML or JSON file with the matrix')
    ap.add_argument('--quick', action='store_true', help='Run the builtin 1-config smoke test')
    ap.add_argument('--n-warmup', type=int, default=1, help='Laps dropped at the start of each run')
    ap.add_argument('--dry-run', action='store_true', help='Print resolved matrix and exit')
    args = ap.parse_args()

    if args.matrix:
        raw = load_matrix(args.matrix)
    elif args.quick:
        raw = BUILTIN_QUICK_MATRIX
    else:
        sys.exit('pick one of --matrix FILE or --quick')

    matrix = [normalize_config(c, default_n_warmup=args.n_warmup) for c in raw]

    print(f'[bench] matrix ({len(matrix)} run(s)):')
    for c in matrix:
        print(f'  {c}')
    if args.dry_run:
        return

    if not CSV_PATH.parent.exists():
        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)

    tag = datetime.now().strftime('%Y%m%dT%H%M%S')
    print(f'[bench] starting, tag={tag}')

    results = []
    t0 = time.time()
    for i, cfg in enumerate(matrix, 1):
        print(f'[bench] === run {i}/{len(matrix)}: {cfg} ===')
        results.append(run_one(cfg))

    table = aggregate(results)

    summary_path = SNAPSHOTS_DIR / f'results_{tag}.csv'
    write_summary(table, summary_path)
    plot_summary(table, FIGURES_DIR, tag)

    elapsed = time.time() - t0
    print(f'[bench] DONE in {elapsed:.0f} s. Summary: {summary_path}')
    for row in table:
        print(f'  {row}')


if __name__ == '__main__':
    main()
