"""Resolve repo docs/ for launch-time lap_timer parameters."""

import os

# Keep in sync with autocar_nav.lap_times_paths
_LAP_TIMES_DIR = 'lap_times'


def repo_docs_dir():
    docs = os.environ.get('AUTOCAR_DOCS_DIR')
    if docs:
        p = os.path.abspath(docs)
        return p if os.path.basename(p) != _LAP_TIMES_DIR else os.path.dirname(p)
    root = os.environ.get('AUTOCAR_REPO_ROOT')
    if root:
        return os.path.join(os.path.abspath(root), 'docs')
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..', 'docs'))


def lap_timer_parameters(stack, use_sim_time):
    docs = repo_docs_dir()
    return {
        'use_sim_time': use_sim_time,
        'stack': stack,
        'lap_times_csv': os.path.join(docs, _LAP_TIMES_DIR, f'lap_times_{stack}.csv'),
    }
