"""Shared helpers for race stack launch files (centerline / racing line)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
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


def race_launch_arguments(default_world):
    """Common launch args for race simulations."""
    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo client if true.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz if true.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world file to launch.',
        ),
        DeclareLaunchArgument(
            'line',
            default_value='centerline',
            description='Waypoint track: centerline or racing.',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='auto',
            description='Initial control mode: manual, semi, or auto.',
        ),
        DeclareLaunchArgument(
            'camera_mode',
            default_value='follow',
            description='Preferred camera mode: free, top, or follow.',
        ),
        *experiment_launch_arguments(),
    ]


def simulation_nodes(default_world):
    """Return Gazebo, robot_state_publisher and RViz actions."""
    descpkg = 'autocar_description'
    rviz = os.path.join(
        get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    urdf = os.path.join(
        get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return [
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),

        ExecuteProcess(
            cmd=[
                'gzserver',
                '--verbose',
                LaunchConfiguration('world', default=default_world),
                '-s',
                'libgazebo_ros_init.so',
                '-s',
                'libgazebo_ros_factory.so',
            ],
        ),
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui'))),

        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            parameters=[{'use_sim_time': use_sim_time}],
            output={'both': 'log'},
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def race_launch_description(navpkg, stack, navconfig, world_name='race_circuit.world'):
    """Build a full race launch description for one navigation stack."""
    gzpkg = 'autocar_gazebo'
    default_world = os.path.join(
        get_package_share_directory(gzpkg), 'worlds', world_name)

    def _nav_setup(context, *args, **kwargs):
        return navigation_nodes(context, navpkg, stack, navconfig)

    return LaunchDescription([
        *race_launch_arguments(default_world),
        *simulation_nodes(default_world),
        OpaqueFunction(function=_nav_setup),
    ])


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
    initial_mode = LaunchConfiguration('control_mode').perform(context)
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
            package='autocar_nav', name='control_manager',
            executable='control_manager.py',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_mode': initial_mode,
            }],
        ),
        Node(
            package=navpkg, name='path_tracker', executable='tracker.py',
            parameters=[navconfig, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autocar_nav', name='viz_status',
            executable='viz_status.py',
            parameters=[{'use_sim_time': use_sim_time}],
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
