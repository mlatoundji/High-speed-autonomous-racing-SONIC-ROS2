"""Shared helpers for race stack launch files (centerline / racing line)."""

import json
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration

from repo_results import lap_timer_parameters

INJECTOR_PKG = 'autocar_nav'

RACE_TRACKS = {
    'circuit': {
        'world': 'race_circuit.world',
        'waypoints': {
            'centerline': 'waypoints.csv',
            'racing': 'waypoints_racing.csv',
        },
        'lap_timer': {
            'finish_mode': 'pos_y',
            'finish_line_x': 103.67,
            'finish_y_center': 0.0,
            'finish_y_half_width': 8.0,
        },
    },
    'oval': {
        'world': 'race_oval.world',
        'waypoints': {
            'centerline': 'waypoints_oval.csv',
            'racing': 'waypoints_oval_racing.csv',
        },
        'lap_timer': {
            'finish_mode': 'pos_y',
            'finish_line_x': 125.81,
            'finish_y_center': 0.0,
            'finish_y_half_width': 8.0,
        },
    },
    'f1_circuit': {
        'world': 'race_f1_circuit.world',
        'waypoints': {
            'centerline': 'waypoints_f1.csv',
            'racing': 'waypoints_f1_racing.csv',
        },
    },
    'f1_circuit_fenced': {
        'world': 'race_f1_circuit_fenced.world',
        'waypoints': {
            'centerline': 'waypoints_f1.csv',
            'racing': 'waypoints_f1_racing.csv',
        },
    },
}

# Back-compat alias for code that only maps line -> filename on the circuit track.
WAYPOINTS_FILES = RACE_TRACKS['circuit']['waypoints']


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


def resolve_track(track: str) -> dict:
    if track not in RACE_TRACKS:
        raise RuntimeError(
            f'Unknown track {track!r}; use one of: {sorted(RACE_TRACKS)}')
    return RACE_TRACKS[track]


def resolve_world_path(track: str) -> str:
    gzpkg = 'autocar_gazebo'
    cfg = resolve_track(track)
    return os.path.join(
        get_package_share_directory(gzpkg), 'worlds', cfg['world'])


def resolve_waypoints_file(track: str, line: str) -> str:
    cfg = resolve_track(track)
    if line not in cfg['waypoints']:
        raise RuntimeError(
            f'Unknown line {line!r} for track {track!r}; '
            f'use one of: {sorted(cfg["waypoints"])}')
    return cfg['waypoints'][line]


def resolve_lap_timer_params(track: str) -> dict:
    cfg = resolve_track(track)
    params = dict(cfg.get('lap_timer', {}))
    if track in ('f1_circuit', 'f1_circuit_fenced'):
        meta_path = os.path.join(
            get_package_share_directory('autocar_racing_line'),
            'data',
            'f1_circuit_meta.json',
        )
        try:
            with open(meta_path) as f:
                finish = json.load(f).get('finish_line', {})
            if finish:
                params = {
                    'finish_mode': finish.get('mode', 'pos_x'),
                    'finish_line_x': finish.get('line_x', 0.0),
                    'finish_y_center': finish.get('y_center', 0.0),
                    'finish_y_half_width': finish.get('y_half_width', 16.0),
                }
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            pass
    return params


def world_path_from_context(context) -> str:
    """Gazebo world file: explicit ``world`` launch arg, else from ``track``."""
    world_override = LaunchConfiguration('world').perform(context).strip()
    if world_override:
        return world_override
    track = LaunchConfiguration('track').perform(context)
    return resolve_world_path(track)


def _world_without_camera(world_path: str) -> str:
    """Return a temp world SDF with the third-person camera sensor stripped."""
    with open(world_path) as f:
        sdf = f.read()
    sdf = re.sub(
        r'\n\s*<joint name="third_person_camera_joint".*?</joint>',
        '', sdf, flags=re.DOTALL)
    sdf = re.sub(
        r'\n\s*<link name="third_person_camera_link">.*?</link>',
        '', sdf, flags=re.DOTALL)
    tmp = tempfile.NamedTemporaryFile('w', suffix='_nocam.world', delete=False)
    tmp.write(sdf)
    tmp.close()
    return tmp.name


def simulation_world_path(context) -> str:
    """Resolve world path, optionally stripping the third-person camera sensor."""
    world = world_path_from_context(context)
    camera = LaunchConfiguration('camera').perform(context).strip().lower()
    if camera in ('false', '0', 'no'):
        return _world_without_camera(world)
    return world


def race_launch_arguments(
        default_track='circuit',
        default_line='centerline',
        default_rviz_config=''):
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
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz config file (empty = autocar_description/rviz/view.rviz).',
        ),
        DeclareLaunchArgument(
            'track',
            default_value=default_track,
            description='Race layout: circuit, oval, f1_circuit, or f1_circuit_fenced.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Gazebo world file path (empty = world for ``track``).',
        ),
        DeclareLaunchArgument(
            'line',
            default_value=default_line,
            description='Waypoint line: centerline or racing (lap 2+ for LiDAR stack).',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='auto',
            description='Initial control mode: manual, semi, or auto.',
        ),
        DeclareLaunchArgument(
            'use_control_manager',
            default_value='false',
            description=(
                'If true, start control_manager (auto_cmd_vel -> cmd_vel, rate limits, '
                'manual/semi). If false, remap path_tracker auto_cmd_vel to cmd_vel directly.'
            ),
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='true',
            description='Render the third-person camera sensor. '
                        'Set false for cheaper sim (higher RTF) while still '
                        'showing the car in Gazebo.',
        ),
        DeclareLaunchArgument(
            'camera_mode',
            default_value='follow',
            description='Preferred camera mode: free, top, or follow.',
        ),
        *experiment_launch_arguments(),
    ]


def simulation_nodes(context):
    """Return Gazebo, robot_state_publisher and RViz actions."""
    descpkg = 'autocar_description'
    rviz_override = LaunchConfiguration('rviz_config').perform(context).strip()
    if rviz_override:
        rviz = rviz_override
    else:
        rviz = os.path.join(
            get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    urdf = os.path.join(
        get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')
    world = simulation_world_path(context)

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
                world,
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


def race_launch_description(navpkg, stack, navconfig, default_track='circuit'):
    """Build a full race launch description for one navigation stack."""

    def _race_setup(context, *args, **kwargs):
        return simulation_nodes(context) + navigation_nodes(
            context, navpkg, stack, navconfig)

    return LaunchDescription([
        *race_launch_arguments(default_track),
        OpaqueFunction(function=_race_setup),
    ])


def navigation_nodes(context, navpkg, stack, navconfig):
    """Return nav stack nodes with ``track`` / ``line`` applied to global_planner."""
    track = LaunchConfiguration('track').perform(context)
    line = LaunchConfiguration('line').perform(context)
    waypoints_file = resolve_waypoints_file(track, line)
    use_sim_time = LaunchConfiguration('use_sim_time')
    profile = LaunchConfiguration('profile').perform(context)
    latency_ms = int(LaunchConfiguration('latency_ms').perform(context))
    odom_noise_std = float(LaunchConfiguration('odom_noise_std').perform(context))
    initial_mode = LaunchConfiguration('control_mode').perform(context)
    use_control_manager = LaunchConfiguration('use_control_manager').perform(
        context).strip().lower() in ('true', '1', 'yes')
    mappkg = 'autocar_map'

    planner_params = [
        navconfig,
        {'use_sim_time': use_sim_time},
        {'waypoints_file': waypoints_file},
    ]

    injector_params = {'use_sim_time': use_sim_time}

    tracker_remappings = (
        [] if use_control_manager
        else [('/autocar/auto_cmd_vel', '/autocar/cmd_vel')]
    )

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
            condition=IfCondition(LaunchConfiguration('use_control_manager')),
        ),
        Node(
            package=navpkg, name='path_tracker', executable='tracker.py',
            parameters=[navconfig, {'use_sim_time': use_sim_time}],
            remappings=tracker_remappings,
        ),
        Node(
            package='autocar_nav', name='viz_status',
            executable='viz_status.py',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_control_manager')),
        ),
        Node(
            package='autocar_nav',
            name='lap_timer',
            executable='lap_timer.py',
            parameters=[
                {
                    **lap_timer_parameters(
                        stack,
                        use_sim_time,
                        navconfig,
                        line=line,
                        profile=profile,
                        latency_ms=latency_ms,
                        odom_noise_std=odom_noise_std,
                    ),
                    **resolve_lap_timer_params(track),
                },
            ],
        ),
    ]


def navigation_nodes_lidar(context, navpkg, stack, navconfig):
    """LiDAR hybrid stack: lap-1 SLAM exploration, lap-2+ min-curvature line from SLAM map."""
    track = LaunchConfiguration('track').perform(context)
    line = LaunchConfiguration('line').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    profile = LaunchConfiguration('profile').perform(context)
    latency_ms = int(LaunchConfiguration('latency_ms').perform(context))
    odom_noise_std = float(LaunchConfiguration('odom_noise_std').perform(context))
    initial_mode = LaunchConfiguration('control_mode').perform(context)
    use_control_manager = LaunchConfiguration('use_control_manager').perform(
        context).strip().lower() in ('true', '1', 'yes')
    slam_config = os.path.join(
        get_package_share_directory(navpkg), 'config', 'slam_toolbox.yaml')

    base_params = [navconfig, {'use_sim_time': use_sim_time}]
    injector_params = {'use_sim_time': use_sim_time}
    run_meta = lap_timer_parameters(
        stack,
        use_sim_time,
        navconfig,
        line=line,
        profile=profile,
        latency_ms=latency_ms,
        odom_noise_std=odom_noise_std,
    )
    planner_params = [
        navconfig,
        {'use_sim_time': use_sim_time},
        {
            'run_id': run_meta['run_id'],
            'run_dir': run_meta['run_dir'],
        },
    ]

    tracker_remappings = (
        [] if use_control_manager
        else [('/autocar/auto_cmd_vel', '/autocar/cmd_vel')]
    )

    return [
        Node(
            package=navpkg, name='localisation', executable='localisation.py',
            parameters=base_params,
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
            package=navpkg, name='global_planner_lidar',
            executable='global_planner_lidar.py',
            parameters=planner_params,
        ),
        Node(
            package=navpkg, name='local_planner', executable='localplanner.py',
            parameters=base_params,
        ),
        Node(
            package='slam_toolbox',
            name='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=[slam_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autocar_nav', name='control_manager',
            executable='control_manager.py',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_mode': initial_mode,
            }],
            condition=IfCondition(LaunchConfiguration('use_control_manager')),
        ),
        Node(
            package=navpkg, name='path_tracker', executable='tracker.py',
            parameters=base_params,
            remappings=tracker_remappings,
        ),
        Node(
            package='autocar_nav', name='viz_status',
            executable='viz_status.py',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_control_manager')),
        ),
        Node(
            package='autocar_nav',
            name='lap_timer',
            executable='lap_timer.py',
            parameters=[
                {
                    **run_meta,
                    **resolve_lap_timer_params(track),
                },
            ],
        ),
    ]
