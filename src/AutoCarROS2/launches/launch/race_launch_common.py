"""Shared helpers for race stack launch files (centerline / racing line)."""

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from repo_results import lap_timer_parameters

WAYPOINTS_FILES = {
    'centerline': 'waypoints.csv',
    'racing': 'waypoints_racing.csv',
}


def resolve_waypoints_file(line: str) -> str:
    if line not in WAYPOINTS_FILES:
        raise RuntimeError(
            f'Unknown line {line!r}; use one of: {sorted(WAYPOINTS_FILES)}')
    return WAYPOINTS_FILES[line]


def navigation_nodes(context, navpkg, stack, navconfig):
    """Return nav stack nodes with ``line`` applied to global_planner and lap_timer."""
    line = LaunchConfiguration('line').perform(context)
    waypoints_file = resolve_waypoints_file(line)
    use_sim_time = LaunchConfiguration('use_sim_time')
    mappkg = 'autocar_map'

    planner_params = [
        navconfig,
        {'use_sim_time': use_sim_time},
        {'waypoints_file': waypoints_file},
    ]

    return [
        Node(
            package=navpkg, name='localisation', executable='localisation.py',
            parameters=[navconfig, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package=navpkg, name='global_planner', executable='globalplanner.py',
            parameters=planner_params,
        ),
        Node(
            package=navpkg, name='local_planner', executable='localplanner.py',
            parameters=[navconfig, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package=mappkg, name='bof', executable='bof',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package=navpkg, name='path_tracker', executable='tracker.py',
            parameters=[navconfig, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autocar_nav',
            name='lap_timer',
            executable='lap_timer.py',
            parameters=[
                lap_timer_parameters(stack, use_sim_time, navconfig, line=line),
            ],
        ),
    ]
