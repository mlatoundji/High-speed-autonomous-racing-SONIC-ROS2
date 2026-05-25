#!/usr/bin/env python3
"""Batch race benchmarks: run configs from ``scripts/configs/``, write to ``results/benchmark_<timestamp>/``.

Examples:
    python3 scripts/benchmark.py --smoke
    python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml
    python3 scripts/benchmark.py --config r4_latency_sweep.yaml --dry-run
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import FrozenSet, Optional

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_SCRIPTS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = Path(
    os.environ.get('AUTOCAR_REPO_ROOT', _SCRIPTS_DIR.parent)
).expanduser().resolve()

CONFIGS_DIR = _SCRIPTS_DIR / 'configs'

STACK_LAUNCH_FILES = {
    'stanley': 'race_launch.py',
    'mpc': 'race_mpc_launch.py',
    'pure_pursuit': 'race_pure_pursuit_launch.py',
}

KNOWN_STACKS: FrozenSet[str] = frozenset(STACK_LAUNCH_FILES)
VALID_LINES: FrozenSet[str] = frozenset({'centerline', 'racing'})

LAP_TIMEOUT_S = 600.0
SIM_COOLDOWN_S = 5.0
LAUNCH_SHUTDOWN_TIMEOUT_S = 20.0
STALE_SIM_TERM_WAIT_S = 3.0
_STALE_SIM_PROCS = ('gzserver', 'gzclient', 'rviz2')

SMOKE_RUN = {
    'stack': 'stanley',
    'profile': 'default',
    'line': 'centerline',
    'latency_ms': 0,
    'odom_noise_std': 0.0,
    'lap_count': 1,
    'warmup_laps': 0,
}


def results_dir() -> Path:
    env = os.environ.get('AUTOCAR_RESULTS_DIR')
    if env:
        return Path(env).expanduser().resolve()
    return _REPO_ROOT / 'results'


def resolve_config_path(config_path: Path) -> Path:
    """Resolve config path: repo-relative, or under ``scripts/configs/``."""
    if config_path.is_absolute():
        return config_path.resolve()
    for candidate in (
        _REPO_ROOT / config_path,
        CONFIGS_DIR / config_path,
        CONFIGS_DIR / config_path.name,
    ):
        if candidate.exists():
            return candidate.resolve()
    return (_REPO_ROOT / config_path).resolve()


# ---------------------------------------------------------------------------
# Config model
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class BenchmarkRun:
    stack: str
    profile: str
    line: str
    latency_ms: int
    odom_noise_std: float
    lap_count: int
    warmup_laps: int

    def as_dict(self) -> dict:
        return {
            'stack': self.stack,
            'profile': self.profile,
            'line': self.line,
            'latency_ms': self.latency_ms,
            'odom_noise_std': self.odom_noise_std,
            'lap_count': self.lap_count,
            'warmup_laps': self.warmup_laps,
        }


def _log(msg: str) -> None:
    print(f'[benchmark] {msg}')


def load_config_file(path: Path) -> list:
    if not path.exists():
        sys.exit(f'config file not found: {path}')
    text = path.read_text(encoding='utf-8')
    suffix = path.suffix.lower()
    if suffix in ('.yaml', '.yml'):
        try:
            import yaml
        except ImportError:
            sys.exit('PyYAML required: pip install pyyaml')
        data = yaml.safe_load(text)
    elif suffix == '.json':
        data = json.loads(text)
    else:
        sys.exit(f'unsupported config extension: {path.suffix}')
    if not isinstance(data, list):
        sys.exit(f'config must be a list of runs: {path}')
    return data


def normalize_run(raw: dict, default_warmup_laps: int) -> BenchmarkRun:
    stack = str(raw.get('stack', 'stanley'))
    if stack not in KNOWN_STACKS:
        sys.exit(f'unknown stack {stack!r}; use one of: {sorted(KNOWN_STACKS)}')

    lap_count = int(raw.get('lap_count', 3))
    requested_warmup = int(raw.get('warmup_laps', default_warmup_laps))
    warmup_laps = max(0, min(requested_warmup, lap_count - 1))
    if warmup_laps != requested_warmup:
        _log(f'warmup_laps capped from {requested_warmup} to {warmup_laps} (lap_count={lap_count})')

    line = str(raw.get('line', 'centerline'))
    if line not in VALID_LINES:
        sys.exit(f'unknown line {line!r}; use centerline or racing')

    return BenchmarkRun(
        stack=stack,
        profile=str(raw.get('profile', 'default')),
        line=line,
        latency_ms=int(raw.get('latency_ms', 0)),
        odom_noise_std=float(raw.get('odom_noise_std', 0.0)),
        lap_count=lap_count,
        warmup_laps=warmup_laps,
    )


def load_runs(path: Path, default_warmup_laps: int = 1) -> list[BenchmarkRun]:
    return [
        normalize_run(entry, default_warmup_laps)
        for entry in load_config_file(path)
    ]


def config_source_path(args: argparse.Namespace) -> Path:
    if args.smoke:
        return CONFIGS_DIR / 'smoke.yaml'
    return resolve_config_path(args.config)


def config_source_label(path: Path) -> str:
    try:
        return str(path.relative_to(_REPO_ROOT))
    except ValueError:
        return str(path)


def write_benchmark_config(
    out_dir: Path,
    tag: str,
    runs: list[BenchmarkRun],
    *,
    config_source: Path,
    default_warmup_laps: int,
    smoke: bool,
) -> Path:
    import yaml

    dest = out_dir / 'config.yaml'
    payload = {
        'benchmark': {
            'directory': f'benchmark_{tag}',
            'started_at': datetime.now().isoformat(timespec='seconds'),
            'smoke': smoke,
            'config_source': config_source_label(config_source),
            'default_warmup_laps': default_warmup_laps,
        },
        'runs': [run.as_dict() for run in runs],
    }
    with dest.open('w', encoding='utf-8') as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
    return dest


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def _sim_process_running(name: str) -> bool:
    result = subprocess.run(
        ['pgrep', '-x', name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def terminate_launch_process(proc: subprocess.Popen) -> None:
    """Shut down ``ros2 launch`` and its children (Gazebo, RViz, nodes) gracefully."""
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    for sig, label in (
        (signal.SIGINT, 'SIGINT'),
        (signal.SIGTERM, 'SIGTERM'),
    ):
        if proc.poll() is not None:
            return
        _log(f'sending {label} to launch process group')
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=LAUNCH_SHUTDOWN_TIMEOUT_S)
            _log(f'launch exited (code {proc.returncode})')
            return
        except subprocess.TimeoutExpired:
            continue

    if proc.poll() is None:
        _log('launch still running; sending SIGKILL')
        try:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def cleanup_stale_sim_processes() -> None:
    """Remove orphaned Gazebo / RViz after launch exit.

    Uses SIGTERM first so the next ``ros2 launch`` (which also killall's gz)
    does not fight frozen ``-9`` windows. SIGKILL only if still alive.
    """
    for name in _STALE_SIM_PROCS:
        if not _sim_process_running(name):
            continue
        subprocess.run(
            ['killall', name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    time.sleep(STALE_SIM_TERM_WAIT_S)

    for name in _STALE_SIM_PROCS:
        if not _sim_process_running(name):
            continue
        _log(f'{name} still running; sending SIGKILL')
        subprocess.run(
            ['killall', '-9', name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def run_dir_prefix(run: BenchmarkRun) -> str:
    if run.line == 'racing':
        return f'{run.stack}_{run.line}_'
    return f'{run.stack}_'


def list_run_dir_names(results_root: Path) -> set:
    if not results_root.exists():
        return set()
    return {p.name for p in results_root.iterdir() if p.is_dir()}


def find_new_run_dir(before: set, run: BenchmarkRun, results_root: Path) -> Optional[str]:
    prefix = run_dir_prefix(run)
    after = list_run_dir_names(results_root) - before
    candidates = sorted((n for n in after if n.startswith(prefix)), reverse=True)
    return candidates[0] if candidates else None


def count_data_rows(csv_path: Path) -> int:
    if not csv_path.exists():
        return 0
    with csv_path.open(encoding='utf-8') as f:
        return max(0, sum(1 for _ in f) - 1)


def read_lap_rows(csv_path: Path) -> list:
    if not csv_path.exists():
        return []
    with csv_path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def build_launch_command(run: BenchmarkRun) -> str:
    launch_file = STACK_LAUNCH_FILES[run.stack]
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    return (
        f'source /opt/ros/{ros_distro}/setup.bash && '
        'source install/setup.bash && '
        f'ros2 launch launches {launch_file} '
        f'profile:={run.profile} '
        f'line:={run.line} '
        f'latency_ms:={run.latency_ms} '
        f'odom_noise_std:={run.odom_noise_std}'
    )


def execute_run(run: BenchmarkRun, results_root: Path) -> dict:
    if any(_sim_process_running(name) for name in _STALE_SIM_PROCS):
        _log('stale Gazebo/RViz detected before launch; cleaning up')
        cleanup_stale_sim_processes()
        time.sleep(SIM_COOLDOWN_S)

    dirs_before = list_run_dir_names(results_root)
    expected_laps = run.lap_count
    deadline = time.time() + LAP_TIMEOUT_S * expected_laps

    proc = subprocess.Popen(
        ['bash', '-lc', build_launch_command(run)],
        cwd=_REPO_ROOT,
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    _log(f'pid={proc.pid} waiting for {expected_laps} lap(s)...')

    run_dir_name = None
    csv_path = None
    rows_before = 0
    timed_out = False

    while True:
        time.sleep(2.0)
        if proc.poll() is not None:
            _log(f'launch exited (code {proc.returncode})')
            break

        if run_dir_name is None:
            run_dir_name = find_new_run_dir(dirs_before, run, results_root)
            if run_dir_name:
                csv_path = results_root / run_dir_name / 'lap_times.csv'
                rows_before = count_data_rows(csv_path)
                _log(f'tracking {csv_path}')

        if csv_path is not None:
            current = count_data_rows(csv_path)
            if current >= rows_before + expected_laps:
                _log(f'{current - rows_before} lap(s) recorded')
                break

        if time.time() > deadline:
            _log('TIMEOUT')
            timed_out = True
            break

    terminate_launch_process(proc)
    cleanup_stale_sim_processes()
    time.sleep(SIM_COOLDOWN_S)

    rows = []
    session_id = None
    if run_dir_name and csv_path and csv_path.exists():
        rows = read_lap_rows(csv_path)
        if rows:
            session_id = rows[-1].get('session_id')

    return {
        'run': run,
        'session_id': session_id,
        'run_dir': run_dir_name,
        'rows': rows,
        'timed_out': timed_out,
    }


# ---------------------------------------------------------------------------
# Aggregation & plots
# ---------------------------------------------------------------------------

def _float_field(row: dict, key: str, default: float = 0.0) -> float:
    value = row.get(key, '')
    if value in ('', None):
        return default
    return float(value)


def summarize_results(results: list) -> list:
    table = []
    for entry in results:
        run = entry['run']
        rows = entry['rows'][run.warmup_laps:]
        durations = [
            _float_field(x, 'duration_s')
            for x in rows
            if x.get('duration_s')
        ]

        summary = {
            'stack': run.stack,
            'profile': run.profile,
            'line': run.line,
            'latency_ms': run.latency_ms,
            'odom_noise_std': run.odom_noise_std,
            'session_id': entry['session_id'],
            'run_dir': entry['run_dir'],
            'lap_count': run.lap_count,
            'warmup_laps': run.warmup_laps,
            'laps_recorded': len(entry['rows']),
            'laps_measured': len(rows),
            'timed_out': entry['timed_out'],
        }
        if durations:
            summary.update({
                'median_lap_s': statistics.median(durations),
                'std_lap_s': statistics.stdev(durations) if len(durations) > 1 else 0.0,
                'min_lap_s': min(durations),
                'max_lap_s': max(durations),
                'median_lateral_error_rms': statistics.median(
                    _float_field(x, 'lateral_error_rms') for x in rows),
                'max_lateral_error_max': max(
                    _float_field(x, 'lateral_error_max') for x in rows),
                'offtrack_events_total': sum(
                    int(_float_field(x, 'offtrack_events')) for x in rows),
            })
        else:
            summary.update({
                'median_lap_s': None,
                'std_lap_s': None,
                'min_lap_s': None,
                'max_lap_s': None,
                'median_lateral_error_rms': None,
                'max_lateral_error_max': None,
                'offtrack_events_total': None,
            })
        table.append(summary)
    return table


def write_summary_csv(table: list, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not table:
        path.write_text('', encoding='utf-8')
        return
    fieldnames = list(table[0].keys())
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in table:
            writer.writerow(row)


def plot_summary(table: list, out_dir: Path, tag: str) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        _log('matplotlib not available, skipping plots')
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    valid = [r for r in table if r.get('median_lap_s') is not None]
    if not valid:
        _log('no valid rows, skipping plots')
        return

    labels = [f"{r['stack']}/{r['profile']}" for r in valid]
    medians = [r['median_lap_s'] for r in valid]
    stds = [r['std_lap_s'] for r in valid]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(range(len(labels)), medians, yerr=stds, capsize=5)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.set_ylabel('Lap time (s)')
    ax.set_title(f'benchmark {tag}: median lap time')
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f'lap_by_stack.png', dpi=120)
    plt.close(fig)

    latencies = sorted({r['latency_ms'] for r in valid})
    if len(latencies) > 1:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for stack in sorted({r['stack'] for r in valid}):
            xs, ys = [], []
            for r in valid:
                if r['stack'] == stack:
                    xs.append(r['latency_ms'])
                    ys.append(r['median_lap_s'])
            ax.plot(xs, ys, 'o-', label=stack)
        ax.set_xlabel('latency_ms')
        ax.set_ylabel('Median lap time (s)')
        ax.set_title(f'benchmark {tag}: lap time vs latency')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f'lap_vs_latency.png', dpi=120)
        plt.close(fig)

    noises = sorted({r['odom_noise_std'] for r in valid})
    if len(noises) > 1:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for stack in sorted({r['stack'] for r in valid}):
            xs, ys = [], []
            for r in valid:
                if r['stack'] == stack:
                    xs.append(r['odom_noise_std'])
                    ys.append(r['median_lap_s'])
            ax.plot(xs, ys, 'o-', label=stack)
        ax.set_xlabel('odom_noise_std')
        ax.set_ylabel('Median lap time (s)')
        ax.set_title(f'benchmark {tag}: lap time vs odom noise')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / f'lap_vs_odom_noise.png', dpi=120)
        plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog='benchmark',
        description='Run race benchmark configs and write results/benchmark_<timestamp>/',
    )
    ap.add_argument(
        '--config', '-c',
        type=Path,
        metavar='FILE',
        help='YAML/JSON config (e.g. scripts/configs/r4_latency_sweep.yaml or r4_latency_sweep.yaml)',
    )
    ap.add_argument(
        '--smoke',
        action='store_true',
        help='Single-lap Stanley smoke test (same as scripts/configs/smoke.yaml)',
    )
    ap.add_argument(
        '--warmup-laps',
        type=int,
        default=1,
        help='Default warmup laps when omitted from config (default: 1)',
    )
    ap.add_argument('--dry-run', action='store_true', help='Print resolved runs and exit (no config.yaml written)')
    ap.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='Output directory (default: results/benchmark_<timestamp>/)',
    )
    ap.add_argument('--no-plots', action='store_true', help='Skip matplotlib figures')
    return ap


def resolve_runs(args: argparse.Namespace) -> list[BenchmarkRun]:
    if args.smoke:
        return [normalize_run(SMOKE_RUN, args.warmup_laps)]

    if args.config is None:
        build_parser().error('one of --config or --smoke is required')

    return load_runs(resolve_config_path(args.config), default_warmup_laps=args.warmup_laps)


def main(argv: Optional[list] = None) -> None:
    args = build_parser().parse_args(argv)

    runs = resolve_runs(args)
    _log(f'{len(runs)} run(s):')
    for run in runs:
        _log(f'  {run.as_dict()}')
    if args.dry_run:
        return

    root = results_dir()
    root.mkdir(parents=True, exist_ok=True)

    tag = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')
    out_dir = args.output_dir or (root / f'benchmark_{tag}')
    out_dir.mkdir(parents=True, exist_ok=True)

    config_path = write_benchmark_config(
        out_dir,
        tag,
        runs,
        config_source=config_source_path(args),
        default_warmup_laps=args.warmup_laps,
        smoke=args.smoke,
    )

    _log(f'output={out_dir}')
    _log(f'config: {config_path}')
    t0 = time.time()
    results = []
    for i, run in enumerate(runs, 1):
        _log(f'=== {i}/{len(runs)} ===')
        results.append(execute_run(run, root))

    table = summarize_results(results)
    summary_path = out_dir / 'summary.csv'
    write_summary_csv(table, summary_path)

    if not args.no_plots:
        plot_summary(table, out_dir / 'figures', tag)

    _log(f'done in {time.time() - t0:.0f}s')
    _log(f'summary: {summary_path}')
    for row in table:
        _log(f'  {row}')


if __name__ == '__main__':
    main()
