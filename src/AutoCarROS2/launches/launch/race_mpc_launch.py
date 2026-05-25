import os
import subprocess
import sys

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from race_launch_common import navigation_nodes  # noqa: E402

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nav_setup(context, *args, **kwargs):
    navpkg = 'autocar_nav_mpc'
    navconfig = os.path.join(
        get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')
    return navigation_nodes(context, navpkg, 'mpc', navconfig)


def generate_launch_description():

    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'

    world = os.path.join(
        get_package_share_directory(gzpkg), 'worlds', 'race_circuit.world')
    urdf = os.path.join(
        get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')
    rviz = os.path.join(
        get_package_share_directory(descpkg), 'rviz', 'view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    subprocess.run(['killall', 'gzserver'], check=False)
    subprocess.run(['killall', 'gzclient'], check=False)

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
        ),
        ExecuteProcess(cmd=['gzclient']),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.',
        ),
        DeclareLaunchArgument(
            'line',
            default_value='centerline',
            description='Waypoint track: centerline or racing.',
        ),

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
        ),

        OpaqueFunction(function=_nav_setup),
    ])


def main():
    generate_launch_description()


if __name__ == '__main__':
    main()
