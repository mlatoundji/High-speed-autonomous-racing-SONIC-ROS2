import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from race_launch_common import race_launch_description  # noqa: E402

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    navpkg = 'autocar_nav_mpc'
    navconfig = os.path.join(
        get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')
    return race_launch_description(navpkg, 'mpc', navconfig)


def main():
    generate_launch_description()


if __name__ == '__main__':
    main()
