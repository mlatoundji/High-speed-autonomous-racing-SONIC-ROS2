"""Resolve repo results/ and prepare per-run output for lap_timer."""

import os

_RESULTS_DIR = 'results'


def repo_results_dir():
    results = os.environ.get('AUTOCAR_RESULTS_DIR')
    if results:
        return os.path.abspath(results)
    root = os.environ.get('AUTOCAR_REPO_ROOT')
    if root:
        return os.path.join(os.path.abspath(root), _RESULTS_DIR)
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..', _RESULTS_DIR))


def _use_sim_time_for_params(use_sim_time):
    if isinstance(use_sim_time, bool):
        return use_sim_time
    if isinstance(use_sim_time, str):
        return use_sim_time.lower() in ('true', '1', 'yes')
    return None


def lap_timer_parameters(stack, use_sim_time, navconfig_path=None, run_id=None, line=None):
    """Create ``results/<stack>_<run_id>/`` and return lap_timer ROS parameters."""
    from autocar_nav.lap_times_paths import prepare_run_directory

    run_dir, resolved_run_id = prepare_run_directory(
        stack,
        navconfig_path=navconfig_path,
        use_sim_time=_use_sim_time_for_params(use_sim_time),
        run_id=run_id,
        line=line,
    )

    lap_times_csv = run_dir / 'lap_times.csv'
    return {
        'use_sim_time': use_sim_time,
        'stack': stack,
        'run_id': resolved_run_id,
        'run_dir': str(run_dir),
        'lap_times_csv': str(lap_times_csv),
    }
