import os
import subprocess
import sys

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from repo_results import lap_timer_parameters  # noqa: E402

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():

    navpkg = 'autocar_nav'
    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'
    mappkg = 'autocar_map'

    world = os.path.join(get_package_share_directory(gzpkg), 'worlds', 'race_city.world')
    urdf = os.path.join(get_package_share_directory(descpkg), 'urdf', 'autocar.xacro')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')

    navconfig = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    subprocess.run(['killall', 'gzserver'])
    subprocess.run(['killall', 'gzclient'])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        ExecuteProcess(
            cmd=['gzserver', '--verbose', world, 'libgazebo_ros_factory.so'],
        ),

        ExecuteProcess(
            cmd=['gzclient'],
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        ),

        Node(package=navpkg, name='localisation',   executable='localisation.py',  parameters=[navconfig]),
        Node(package=navpkg, name='global_planner', executable='globalplanner.py', parameters=[navconfig]),
        Node(package=navpkg, name='local_planner',  executable='localplanner.py',  parameters=[navconfig]),
        Node(package=mappkg, name='bof',            executable='bof'),
        Node(package=navpkg, name='path_tracker',   executable='tracker.py',       parameters=[navconfig]),
        Node(
            package='autocar_nav',
            name='lap_timer',
            executable='lap_timer.py',
            parameters=[lap_timer_parameters(
                'stanley',
                LaunchConfiguration('use_sim_time', default='True'),
                navconfig,
            )],
        ),
    ])


def main():
    generate_launch_description()


if __name__ == '__main__':
    main()
