#!/usr/bin/env python3
"""Batch race benchmarks: run configs from ``scripts/configs/``, write to ``results/benchmark_<timestamp>/``.

Examples:
    python3 scripts/benchmark.py --smoke
    python3 scripts/benchmark.py --config scripts/configs/r4_latency_sweep.yaml
    python3 scripts/benchmark.py --config r4_latency_sweep.yaml --dry-run

Pure Pursuit / Pure Pursuit LiDAR runs may include inline ``navigation:`` (full
ROS parameter YAML) or ``nav_config:`` (repo-relative path to a YAML file); see
``scripts/configs/r3_pp_racing_optional.yaml`` and
``scripts/configs/f1_pure_pursuit_lidar.yaml``.
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
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import FrozenSet, Optional

import shlex

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
    'pure_pursuit_lidar': 'race_pure_pursuit_lidar_launch.py',
}

KNOWN_STACKS: FrozenSet[str] = frozenset(STACK_LAUNCH_FILES)
VALID_LINES: FrozenSet[str] = frozenset({'centerline', 'racing'})
VALID_TRACKS: FrozenSet[str] = frozenset({
    'circuit', 'oval', 'f1_circuit', 'f1_circuit_fenced',
})

LAP_TIMEOUT_S = 600.0
LAUNCH_SHUTDOWN_TIMEOUT_S = 20.0
_HARD_RESET_PGREP_PATTERNS = (
    'ros2 launch',
    'tracker.py',
    'localplanner.py',
    'globalplanner.py',
    'localisation.py',
    'lap_timer.py',
    'latency_injector.py',
    'control_manager.py',
    'bof',
    'odom_noise',
)
_HARD_RESET_KILLALL = (
    'gzserver',
    'gzclient',
    'gazebo',
    'robot_state_publisher',
    'rviz2',
    'bof',
)

SMOKE_RUN = {
    'stack': 'stanley',
    'profile': 'default',
    'track': 'circuit',
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
    track: str
    line: str
    latency_ms: int
    odom_noise_std: float
    lap_count: int
    warmup_laps: int
    nav_config: Optional[str] = None
    camera: bool = True

    def as_dict(self) -> dict:
        d = {
            'stack': self.stack,
            'profile': self.profile,
            'track': self.track,
            'line': self.line,
            'latency_ms': self.latency_ms,
            'odom_noise_std': self.odom_noise_std,
            'lap_count': self.lap_count,
            'warmup_laps': self.warmup_laps,
            'camera': self.camera,
        }
        if self.nav_config is not None:
            d['nav_config'] = self.nav_config
        return d


@dataclass(frozen=True)
class BenchmarkRunSpec:
    """One entry from a benchmark YAML: resolved run fields plus optional inline ``navigation``."""

    run: BenchmarkRun
    navigation: Optional[dict] = None


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

    track = str(raw.get('track', 'circuit'))
    if track not in VALID_TRACKS:
        sys.exit(
            f'unknown track {track!r}; use circuit, oval, f1_circuit, or f1_circuit_fenced')

    line = str(raw.get('line', 'centerline'))
    if line not in VALID_LINES:
        sys.exit(f'unknown line {line!r}; use centerline or racing')

    nav_raw = raw.get('nav_config')
    nav_config: Optional[str] = None
    if nav_raw is not None:
        if not isinstance(nav_raw, str) or not str(nav_raw).strip():
            sys.exit('nav_config must be a non-empty string when set')
        p = Path(str(nav_raw).strip())
        if not p.is_absolute():
            p = (_REPO_ROOT / p).resolve()
        else:
            p = p.resolve()
        if not p.is_file():
            sys.exit(f'nav_config file not found: {p}')
        nav_config = str(p)

    return BenchmarkRun(
        stack=stack,
        profile=str(raw.get('profile', 'default')),
        track=track,
        line=line,
        latency_ms=int(raw.get('latency_ms', 0)),
        odom_noise_std=float(raw.get('odom_noise_std', 0.0)),
        lap_count=lap_count,
        warmup_laps=warmup_laps,
        nav_config=nav_config,
        camera=bool(raw.get('camera', True)),
    )


def load_run_specs(path: Path, default_warmup_laps: int = 1) -> list[BenchmarkRunSpec]:
    specs: list[BenchmarkRunSpec] = []
    for entry in load_config_file(path):
        if not isinstance(entry, dict):
            sys.exit(f'each run must be a mapping, got {type(entry).__name__}')
        raw = dict(entry)
        navigation = raw.pop('navigation', None)
        if navigation is not None and not isinstance(navigation, dict):
            sys.exit('navigation must be a mapping (YAML dict) when set')
        run = normalize_run(raw, default_warmup_laps)
        if navigation is not None and run.nav_config is not None:
            sys.exit('use only one of navigation (inline) or nav_config (file path), not both')
        specs.append(BenchmarkRunSpec(run=run, navigation=navigation))
    return specs


def materialize_navigation_overrides(
    specs: list[BenchmarkRunSpec],
    work_dir: Path,
) -> list[BenchmarkRun]:
    """Write inline ``navigation`` dicts to ``work_dir/nav_overrides`` and set ``nav_config``."""
    try:
        import yaml
    except ImportError:
        sys.exit('PyYAML required: pip install pyyaml')

    runs: list[BenchmarkRun] = []
    nav_root = work_dir / 'nav_overrides'
    for i, spec in enumerate(specs, start=1):
        if spec.navigation is None:
            runs.append(spec.run)
            continue
        nav_root.mkdir(parents=True, exist_ok=True)
        path = nav_root / f'run_{i:03d}.yaml'
        with path.open('w', encoding='utf-8') as f:
            yaml.safe_dump(spec.navigation, f, default_flow_style=False, sort_keys=False)
        runs.append(replace(spec.run, nav_config=str(path.resolve())))
    return runs


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

def _run_quiet(cmd: list) -> None:
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def hard_reset_ros_sim() -> None:
    """Kill orphaned ROS/Gazebo processes and restart the ROS 2 daemon.

    Same sequence as the manual reset documented in README.md (``pkill`` /
    ``killall`` / ``ros2 daemon stop|start``).
    """
    _log('hard reset between runs (ROS/Gazebo cleanup)')
    for pattern in _HARD_RESET_PGREP_PATTERNS:
        _run_quiet(['pkill', '-9', '-f', pattern])
    for name in _HARD_RESET_KILLALL:
        _run_quiet(['killall', '-9', name])

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    subprocess.run(
        [
            'bash', '-lc',
            f'source /opt/ros/{ros_distro}/setup.bash && '
            'ros2 daemon stop; sleep 3; '
            'ros2 daemon stop; sleep 2; '
            'ros2 daemon start',
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


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
    cmd = (
        f'source /opt/ros/{ros_distro}/setup.bash && '
        'source install/setup.bash && '
        f'ros2 launch launches {launch_file} '
        f'profile:={run.profile} '
        f'track:={run.track} '
        f'line:={run.line} '
        f'latency_ms:={run.latency_ms} '
        f'odom_noise_std:={run.odom_noise_std} '
        f'camera:={str(run.camera).lower()}'
    )
    if run.stack in ('pure_pursuit', 'pure_pursuit_lidar') and run.nav_config:
        nav = run.nav_config
        nav_arg = shlex.quote(nav) if any(c in nav for c in ' \t\n') else nav
        cmd += f' nav_config:={nav_arg}'
    return cmd


def execute_run(run: BenchmarkRun, results_root: Path) -> dict:
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
    hard_reset_ros_sim()

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


def _lap_metric_summary(rows: list) -> dict:
    """Aggregate post-warmup lap rows.

    ``duration_s`` is summarized as ``median_lap_s`` / ``std_lap_s`` / … to
    avoid duplicating the same lap-time columns under the raw field name.
    """
    durations = [
        _float_field(x, 'duration_s')
        for x in rows
        if x.get('duration_s')
    ]
    empty = {
        'median_lap_s': None,
        'std_lap_s': None,
        'min_lap_s': None,
        'max_lap_s': None,
        'median_avg_speed_mps': None,
        'max_speed_mps': None,
        'median_distance_m': None,
        'median_lateral_error_rms': None,
        'max_lateral_error_max': None,
        'offtrack_events_total': None,
    }
    if not durations:
        return empty
    return {
        'median_lap_s': statistics.median(durations),
        'std_lap_s': statistics.stdev(durations) if len(durations) > 1 else 0.0,
        'min_lap_s': min(durations),
        'max_lap_s': max(durations),
        'median_avg_speed_mps': statistics.median(
            _float_field(x, 'avg_speed_mps') for x in rows),
        'max_speed_mps': max(_float_field(x, 'max_speed_mps') for x in rows),
        'median_distance_m': statistics.median(
            _float_field(x, 'distance_m') for x in rows),
        'median_lateral_error_rms': statistics.median(
            _float_field(x, 'lateral_error_rms') for x in rows),
        'max_lateral_error_max': max(
            _float_field(x, 'lateral_error_max') for x in rows),
        'offtrack_events_total': sum(
            int(_float_field(x, 'offtrack_events')) for x in rows),
    }


def summarize_results(results: list) -> list:
    table = []
    for entry in results:
        run = entry['run']
        rows = entry['rows'][run.warmup_laps:]

        summary = {
            'stack': run.stack,
            'profile': run.profile,
            'track': run.track,
            'line': run.line,
            'latency_ms': run.latency_ms,
            'odom_noise_std': run.odom_noise_std,
            'nav_config': run.nav_config,
            'session_id': entry['session_id'],
            'run_dir': entry['run_dir'],
            'lap_count': run.lap_count,
            'warmup_laps': run.warmup_laps,
            'laps_recorded': len(entry['rows']),
            'laps_measured': len(rows),
            'timed_out': entry['timed_out'],
        }
        summary.update(_lap_metric_summary(rows))
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


def resolve_run_specs(args: argparse.Namespace) -> list[BenchmarkRunSpec]:
    if args.smoke:
        return [
            BenchmarkRunSpec(
                run=normalize_run(SMOKE_RUN, args.warmup_laps),
                navigation=None,
            ),
        ]

    if args.config is None:
        build_parser().error('one of --config or --smoke is required')

    return load_run_specs(resolve_config_path(args.config), default_warmup_laps=args.warmup_laps)


def main(argv: Optional[list] = None) -> None:
    args = build_parser().parse_args(argv)

    specs = resolve_run_specs(args)
    _log(f'{len(specs)} run(s):')
    for i, spec in enumerate(specs, 1):
        line = f'  {i}: {spec.run.as_dict()}'
        if spec.navigation is not None:
            line += f' + navigation: {list(spec.navigation.keys())}'
        _log(line)
    if args.dry_run:
        return

    root = results_dir()
    root.mkdir(parents=True, exist_ok=True)

    tag = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')
    out_dir = args.output_dir or (root / f'benchmark_{tag}')
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = materialize_navigation_overrides(specs, out_dir)

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
    hard_reset_ros_sim()
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
