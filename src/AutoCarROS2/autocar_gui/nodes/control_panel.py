#!/usr/bin/env python3
"""Entry point for the AutoCar desktop control panel."""

import argparse
import sys

import rclpy
from PySide6.QtWidgets import QApplication

from autocar_gui.backend import create_backend
from autocar_gui.main_window import MainWindow


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='AutoCar desktop control panel (PySide6 + rclpy).')
    parser.add_argument(
        '--backend',
        choices=('auto', 'ros', 'http'),
        default='auto',
        help='Communication backend (default: auto-detect).',
    )
    parser.add_argument(
        '--api-url',
        default='http://localhost:8001',
        help='Base URL for HTTP fallback (default: http://localhost:8001).',
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])

    app = QApplication(sys.argv)
    backend = create_backend(args.backend, args.api_url)
    window = MainWindow(backend)
    window.show()

    exit_code = app.exec()
    if rclpy.ok():
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
