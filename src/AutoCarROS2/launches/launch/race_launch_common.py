"""Shared helpers for race stack launch files (centerline / racing line)."""

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration

from repo_results import lap_timer_parameters

INJECTOR_PKG = 'autocar_nav'

WAYPOINTS_FILES = {
    'centerline': 'waypoints.csv',
    'racing': 'waypoints_racing.csv',
}


def experiment_launch_arguments():
    """Launch args for experiment metadata and perception perturbations."""
    return [
        DeclareLaunchArgument(
            'profile',
            default_value='default',
            description='Tuning profile label (recorded in lap_times.csv).',
        ),
        DeclareLaunchArgument(
            'latency_ms',
            default_value='0',
            description='Artificial perception latency in ms (0 = pass-through).',
        ),
        DeclareLaunchArgument(
            'odom_noise_std',
            default_value='0.0',
            description='Gaussian noise std on state2D pose (0 = pass-through).',
        ),
    ]


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
    profile = LaunchConfiguration('profile').perform(context)
    latency_ms = int(LaunchConfiguration('latency_ms').perform(context))
    odom_noise_std = float(LaunchConfiguration('odom_noise_std').perform(context))
    mappkg = 'autocar_map'

    planner_params = [
        navconfig,
        {'use_sim_time': use_sim_time},
        {'waypoints_file': waypoints_file},
    ]

    injector_params = {'use_sim_time': use_sim_time}

    return [
        Node(
            package=navpkg, name='localisation', executable='localisation.py',
            parameters=[navconfig, {'use_sim_time': use_sim_time}],
            remappings=[('/autocar/state2D', '/autocar/state2D_raw')],
        ),
        Node(
            package=INJECTOR_PKG, name='latency_injector',
            executable='latency_injector.py',
            parameters=[injector_params, {
                'latency_ms': ParameterValue(latency_ms, value_type=int),
            }],
        ),
        Node(
            package=INJECTOR_PKG, name='odom_noise_injector',
            executable='odom_noise_injector.py',
            parameters=[injector_params, {
                'odom_noise_std': ParameterValue(odom_noise_std, value_type=float),
            }],
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
                lap_timer_parameters(
                    stack,
                    use_sim_time,
                    navconfig,
                    line=line,
                    profile=profile,
                    latency_ms=latency_ms,
                    odom_noise_std=odom_noise_std,
                ),
            ],
        ),
    ]
