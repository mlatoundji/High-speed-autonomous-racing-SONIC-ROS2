"""Resolve lap-time CSV paths under docs/lap_times/.

Expected layout (relative to repo root):

    docs/
        REPORT_BASELINE.md
        REPORT_MPC.md              # optional, per-controller reports
        lap_times/
            lap_times_baseline.csv   # frozen reference lap (not written by lap_timer)
            lap_times_stanley.csv    # stack=stanley  (lap_timer)
            lap_times_mpc.csv        # stack=mpc
            lap_times_pure_pursuit.csv

Live runs: lap_timer appends to docs/lap_times/lap_times_<stack>.csv for
stack in KNOWN_STACKS. Filename pattern: lap_times_{stack}.csv
"""

import os
from pathlib import Path

LAP_TIMES_DIRNAME = 'lap_times'
BASELINE_NAME = 'lap_times_baseline.csv'
REPORT_BASELINE_NAME = 'REPORT_BASELINE.md'

KNOWN_STACKS = frozenset({'stanley', 'mpc', 'pure_pursuit'})

LAP_TIMES_LEGACY_FIELDS = (
    'session_id',
    'lap_number',
    'timestamp_iso',
    'duration_s',
    'avg_speed_mps',
    'max_speed_mps',
    'distance_m',
)

LAP_TIMES_EXTRA_FIELDS = (
    'controller',
    'profile',
    'latency_ms',
    'odom_noise_std',
    'lateral_error_rms',
    'lateral_error_max',
    'steering_rate_max',
    'offtrack_events',
)

LAP_TIMES_CSV_FIELDS = LAP_TIMES_LEGACY_FIELDS + LAP_TIMES_EXTRA_FIELDS


def stack_lap_times_name(stack):
    """Per-stack log filename: lap_times_<stack>.csv"""
    return f'lap_times_{stack}.csv'


def lap_times_dir(docs_dir: Path) -> Path:
    return docs_dir / LAP_TIMES_DIRNAME


def baseline_csv_in_repo(docs_dir: Path) -> Path:
    return lap_times_dir(docs_dir) / BASELINE_NAME


def stack_csv_in_repo(docs_dir: Path, stack: str) -> Path:
    return lap_times_dir(docs_dir) / stack_lap_times_name(stack)


def _docs_markers(docs: Path):
    return (docs / REPORT_BASELINE_NAME, baseline_csv_in_repo(docs))


def find_repo_root():
    """Return repo root when docs/ matches the expected layout."""
    env_root = os.environ.get('AUTOCAR_REPO_ROOT')
    if env_root:
        root = Path(env_root).expanduser().resolve()
        if all(m.is_file() for m in _docs_markers(root / 'docs')):
            return root

    docs_env = os.environ.get('AUTOCAR_DOCS_DIR')
    if docs_env:
        docs = Path(docs_env).expanduser().resolve()
        if docs.name == LAP_TIMES_DIRNAME:
            docs = docs.parent
        if all(m.is_file() for m in _docs_markers(docs)):
            return docs.parent if docs.name == 'docs' else docs

    for start in (Path(__file__).resolve(), Path.cwd()):
        for parent in (start, *start.parents):
            docs = parent / 'docs'
            if all(m.is_file() for m in _docs_markers(docs)):
                return parent

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('autocar_nav')).resolve()
        for parent in (share, *share.parents):
            docs = parent / 'docs'
            if all(m.is_file() for m in _docs_markers(docs)):
                return parent
    except Exception:
        pass

    return None


def resolve_docs_dir():
    """Project docs/ directory, or ~/.ros as fallback."""
    root = find_repo_root()
    if root is not None:
        return root / 'docs'

    fallback = Path(os.path.expanduser('~/.ros'))
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def resolve_lap_times_dir():
    """docs/lap_times/ (created on demand)."""
    lap_dir = lap_times_dir(resolve_docs_dir())
    lap_dir.mkdir(parents=True, exist_ok=True)
    return lap_dir


def lap_log_paths(stack='unknown', lap_times_csv=None):
    """Return CSV path list under docs/lap_times/ and whether docs/ was found."""
    in_project_docs = find_repo_root() is not None
    lap_dir = resolve_lap_times_dir()

    if lap_times_csv:
        return [Path(lap_times_csv).expanduser()], in_project_docs

    if stack in KNOWN_STACKS:
        return [lap_dir / stack_lap_times_name(stack)], in_project_docs

    return [], in_project_docs
