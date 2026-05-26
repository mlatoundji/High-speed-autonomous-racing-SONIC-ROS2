"""Launch the AutoCar desktop control panel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='auto',
        description='Backend selection: auto, ros or http.',
    )
    api_url_arg = DeclareLaunchArgument(
        'api_url',
        default_value='http://localhost:8001',
        description='Base URL for HTTP fallback.',
    )

    panel = Node(
        package='autocar_gui',
        executable='control_panel.py',
        name='autocar_control_panel',
        output='screen',
        arguments=[
            '--backend', LaunchConfiguration('backend'),
            '--api-url', LaunchConfiguration('api_url'),
        ],
    )

    return LaunchDescription([
        backend_arg,
        api_url_arg,
        panel,
    ])
