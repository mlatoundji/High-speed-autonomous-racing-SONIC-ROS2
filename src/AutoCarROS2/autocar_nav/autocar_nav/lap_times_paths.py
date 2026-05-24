"""Resolve lap-time CSV paths under results/.

Expected layout (relative to repo root):

    docs/
        REPORT_BASELINE.md
        REPORT_MPC.md              # optional, per-controller reports
        REPORT_PURE_PURSUIT.md
    results/
        lap_times_baseline.csv       # frozen reference lap (not written by lap_timer)
        lap_times_stanley.csv        # stack=stanley  (lap_timer)
        lap_times_mpc.csv            # stack=mpc
        lap_times_pure_pursuit.csv

Live runs: lap_timer appends to results/lap_times_<stack>.csv for
stack in KNOWN_STACKS. Filename pattern: lap_times_{stack}.csv
"""

import os
from pathlib import Path

RESULTS_DIRNAME = 'results'
BASELINE_NAME = 'lap_times_baseline.csv'
REPORT_BASELINE_NAME = 'REPORT_BASELINE.md'

KNOWN_STACKS = frozenset({'stanley', 'mpc', 'pure_pursuit'})

LAP_TIMES_CSV_FIELDS = (
    'session_id',
    'lap_number',
    'timestamp_iso',
    'duration_s',
    'avg_speed_mps',
    'max_speed_mps',
    'distance_m',
)


def stack_lap_times_name(stack):
    """Per-stack log filename: lap_times_<stack>.csv"""
    return f'lap_times_{stack}.csv'


def results_dir(root: Path) -> Path:
    return root / RESULTS_DIRNAME


def baseline_csv_in_repo(root: Path) -> Path:
    return results_dir(root) / BASELINE_NAME


def stack_csv_in_repo(root: Path, stack: str) -> Path:
    return results_dir(root) / stack_lap_times_name(stack)


def _repo_markers(root: Path):
    return (root / 'docs' / REPORT_BASELINE_NAME, baseline_csv_in_repo(root))


def find_repo_root():
    """Return repo root when docs/ + results/ match the expected layout."""
    env_root = os.environ.get('AUTOCAR_REPO_ROOT')
    if env_root:
        root = Path(env_root).expanduser().resolve()
        if all(m.is_file() for m in _repo_markers(root)):
            return root

    results_env = os.environ.get('AUTOCAR_RESULTS_DIR')
    if results_env:
        results = Path(results_env).expanduser().resolve()
        if results.is_file():
            results = results.parent
        root = results.parent
        if all(m.is_file() for m in _repo_markers(root)):
            return root

    for start in (Path(__file__).resolve(), Path.cwd()):
        for parent in (start, *start.parents):
            if all(m.is_file() for m in _repo_markers(parent)):
                return parent

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('autocar_nav')).resolve()
        for parent in (share, *share.parents):
            if all(m.is_file() for m in _repo_markers(parent)):
                return parent
    except Exception:
        pass

    return None


def resolve_results_dir():
    """Project results/ directory, or ~/.ros/results as fallback."""
    root = find_repo_root()
    if root is not None:
        return root / RESULTS_DIRNAME

    results_env = os.environ.get('AUTOCAR_RESULTS_DIR')
    if results_env:
        p = Path(results_env).expanduser().resolve()
        return p if p.is_dir() else p.parent

    fallback = Path(os.path.expanduser('~/.ros')) / RESULTS_DIRNAME
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def resolve_lap_times_dir():
    """results/ (created on demand)."""
    lap_dir = resolve_results_dir()
    lap_dir.mkdir(parents=True, exist_ok=True)
    return lap_dir


def lap_log_paths(stack='unknown', lap_times_csv=None):
    """Return CSV path list under results/ and whether the repo was found."""
    in_project_repo = find_repo_root() is not None
    lap_dir = resolve_lap_times_dir()

    if lap_times_csv:
        return [Path(lap_times_csv).expanduser()], in_project_repo

    if stack in KNOWN_STACKS:
        return [lap_dir / stack_lap_times_name(stack)], in_project_repo

    return [], in_project_repo
