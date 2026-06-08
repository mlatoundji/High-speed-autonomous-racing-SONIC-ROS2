import os
import subprocess
import sys

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from race_launch_common import (  # noqa: E402
    navigation_nodes_lidar,
    race_launch_arguments,
    simulation_nodes,
)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _race_setup(context, *args, **kwargs):
    navpkg = 'autocar_nav_pure_pursuit_lidar'
    override = LaunchConfiguration('nav_config').perform(context).strip()
    if override:
        navconfig = override
    else:
        navconfig = os.path.join(
            get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')
    return simulation_nodes(context) + navigation_nodes_lidar(
        context, navpkg, 'pure_pursuit_lidar', navconfig)


def generate_launch_description():
    subprocess.run(['killall', 'gzserver'], check=False)
    subprocess.run(['killall', 'gzclient'], check=False)

    return LaunchDescription([
        *race_launch_arguments(
            default_track='f1_circuit_fenced', default_line='racing'),
        DeclareLaunchArgument(
            'nav_config',
            default_value='',
            description=(
                'Absolute path to navigation_params YAML '
                '(empty = autocar_nav_pure_pursuit_lidar/config/navigation_params.yaml).'
            ),
        ),
        OpaqueFunction(function=_race_setup),
    ])


def main():
    generate_launch_description()


if __name__ == '__main__':
    main()
