"""Resolve repo results/ for launch-time lap_timer parameters."""

import os

# Keep in sync with autocar_nav.lap_times_paths
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


def lap_timer_parameters(stack, use_sim_time):
    results = repo_results_dir()
    return {
        'use_sim_time': use_sim_time,
        'stack': stack,
        'lap_times_csv': os.path.join(results, f'lap_times_{stack}.csv'),
    }
