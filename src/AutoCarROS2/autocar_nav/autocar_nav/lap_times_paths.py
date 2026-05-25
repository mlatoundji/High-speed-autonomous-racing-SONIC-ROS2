"""Resolve lap-time output paths under results/.

Layout (per run):

    results/
        <stack>_<run_id>/
            params.yml       # run metadata + navigation ROS parameters
            lap_times.csv    # one row per completed lap

run_id format: ``YYYY-MM-DDTHH-MM-SS`` (local time, no timezone).
"""

import os
from datetime import datetime
from pathlib import Path

RESULTS_DIRNAME = 'results'
REPORT_BASELINE_NAME = 'REPORT_BASELINE.md'

RUN_LAP_TIMES_NAME = 'lap_times.csv'
RUN_PARAMS_NAME = 'params.yml'

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


def make_run_id(when=None):
    """Return a filesystem-safe run identifier."""
    dt = when or datetime.now()
    return dt.strftime('%Y-%m-%dT%H-%M-%S')


def run_directory_name(stack, run_id, line=None):
    """Directory name: ``<stack>_<line>_<run_id>`` when line is not centerline."""
    if line and line not in ('centerline', 'center'):
        return f'{stack}_{line}_{run_id}'
    return f'{stack}_{run_id}'


def results_dir(root: Path) -> Path:
    return root / RESULTS_DIRNAME


def run_dir_path(stack, run_id=None, results_root=None, line=None):
    """Absolute path to ``results/<stack>_<run_id>/`` (or with line suffix)."""
    base = Path(results_root) if results_root else resolve_lap_times_dir()
    rid = run_id or make_run_id()
    return base / run_directory_name(stack, rid, line=line), rid


def run_lap_times_csv(run_dir: Path) -> Path:
    return Path(run_dir) / RUN_LAP_TIMES_NAME


def run_params_yml(run_dir: Path) -> Path:
    return Path(run_dir) / RUN_PARAMS_NAME


def _repo_root_ok(root: Path) -> bool:
    report = root / 'docs' / REPORT_BASELINE_NAME
    return report.is_file() and results_dir(root).is_dir()


def find_repo_root():
    """Return repo root when docs/REPORT_BASELINE.md and results/ exist."""
    env_root = os.environ.get('AUTOCAR_REPO_ROOT')
    if env_root:
        root = Path(env_root).expanduser().resolve()
        if _repo_root_ok(root):
            return root

    results_env = os.environ.get('AUTOCAR_RESULTS_DIR')
    if results_env:
        results = Path(results_env).expanduser().resolve()
        if results.is_file():
            results = results.parent
        root = results.parent
        if _repo_root_ok(root):
            return root

    for start in (Path(__file__).resolve(), Path.cwd()):
        for parent in (start, *start.parents):
            if _repo_root_ok(parent):
                return parent

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('autocar_nav')).resolve()
        for parent in (share, *share.parents):
            if _repo_root_ok(parent):
                return parent
    except Exception:
        pass

    return None


def resolve_results_dir():
    """Project ``results/`` directory, or ``~/.ros/results`` as fallback."""
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
    """``results/`` (created on demand)."""
    lap_dir = resolve_results_dir()
    lap_dir.mkdir(parents=True, exist_ok=True)
    return lap_dir


def _yaml_available():
    try:
        import yaml  # noqa: F401
        return True
    except ImportError:
        return False


def _load_yaml(path):
    import yaml
    with Path(path).open(encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _dump_yaml(path, data):
    import yaml
    with Path(path).open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


def write_run_params(run_dir, stack, run_id, navconfig_path=None, use_sim_time=None, line=None):
    """Write ``params.yml`` for a new run directory."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    dest = run_params_yml(run_dir)

    payload = {
        'run': {
            'stack': stack,
            'run_id': run_id,
            'started_at': datetime.now().isoformat(timespec='seconds'),
        },
    }
    if use_sim_time is not None:
        payload['run']['use_sim_time'] = bool(use_sim_time) if isinstance(
            use_sim_time, bool) else str(use_sim_time)
    if line is not None:
        payload['run']['line'] = str(line)

    nav_path = None
    if navconfig_path:
        nav_path = Path(navconfig_path).expanduser().resolve()
        payload['run']['navigation_params_source'] = str(nav_path)

    if nav_path is not None and nav_path.is_file():
        if _yaml_available():
            payload['navigation'] = _load_yaml(nav_path)
            _dump_yaml(dest, payload)
            return
        dest.write_text(nav_path.read_text(encoding='utf-8'), encoding='utf-8')
        header = (
            f'# run: stack={stack} run_id={run_id}\n'
            f'# navigation_params_source: {nav_path}\n'
        )
        if use_sim_time is not None:
            header += f'# use_sim_time: {use_sim_time}\n'
        dest.write_text(header + dest.read_text(encoding='utf-8'), encoding='utf-8')
        return

    if _yaml_available():
        _dump_yaml(dest, payload)
    else:
        dest.write_text(
            f'run:\n  stack: {stack}\n  run_id: {run_id}\n',
            encoding='utf-8',
        )


def init_lap_times_csv(csv_path):
    """Create ``lap_times.csv`` with a header row if missing or empty."""
    import csv
    csv_path = Path(csv_path)
    if csv_path.exists() and csv_path.stat().st_size > 0:
        return
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open('w', newline='', encoding='utf-8') as f:
        csv.writer(f).writerow(LAP_TIMES_CSV_FIELDS)


def prepare_run_directory(stack, navconfig_path=None, use_sim_time=None, run_id=None, line=None):
    """Create ``results/<stack>_<run_id>/`` with ``params.yml`` and ``lap_times.csv``.

    Returns:
        (run_dir, run_id) as ``Path`` and ``str``.
    """
    if stack not in KNOWN_STACKS:
        raise ValueError(
            f'Unknown stack {stack!r}; expected one of: {sorted(KNOWN_STACKS)}')

    run_dir, rid = run_dir_path(stack, run_id=run_id, line=line)
    write_run_params(run_dir, stack, rid, navconfig_path, use_sim_time=use_sim_time, line=line)
    init_lap_times_csv(run_lap_times_csv(run_dir))
    return run_dir, rid


def lap_log_paths(stack='unknown', run_dir=None, lap_times_csv=None):
    """Return lap CSV path list and whether the project repo was detected."""
    in_project_repo = find_repo_root() is not None

    if lap_times_csv:
        return [Path(lap_times_csv).expanduser()], in_project_repo

    if run_dir:
        return [run_lap_times_csv(Path(run_dir).expanduser())], in_project_repo

    return [], in_project_repo
