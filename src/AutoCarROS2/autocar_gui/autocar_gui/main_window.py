"""Main window for the AutoCar control panel."""

from __future__ import annotations

import rclpy
from PySide6.QtCore import QTimer
from PySide6.QtGui import QCloseEvent, QImage
from PySide6.QtWidgets import QLabel, QMainWindow, QVBoxLayout, QWidget

from autocar_gui.backend import ControlBackend
from autocar_gui.widgets.camera_view import CameraView
from autocar_gui.widgets.control_bar import ControlBar
from autocar_gui.widgets.telemetry_panel import TelemetryPanel


class MainWindow(QMainWindow):
    def __init__(self, backend: ControlBackend):
        super().__init__()
        self.backend = backend
        self.setWindowTitle('AutoCar Control Panel')
        self.resize(1000, 720)

        self.camera_view = CameraView()
        self.telemetry = TelemetryPanel()
        self.control_bar = ControlBar()
        self.backend_label = QLabel('Backend: —')
        self.backend_label.setStyleSheet('color: #888;')

        if not backend.camera_available:
            self.camera_view.set_placeholder(
                'Caméra indisponible en mode HTTP.\n'
                'Lancez le panel avec ROS pour afficher le flux vidéo.')

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(self.camera_view, stretch=3)
        layout.addWidget(self.telemetry)
        layout.addWidget(self.control_bar)
        layout.addWidget(self.backend_label)
        self.setCentralWidget(central)

        backend.on_status_update = self._on_status_update
        backend.on_image_update = self._on_image_update
        backend.on_backend_info = self._on_backend_info

        self.control_bar.mode_selected.connect(backend.set_mode)
        self.control_bar.stop_requested.connect(backend.stop_vehicle)
        self.control_bar.resume_requested.connect(backend.resume_auto)
        self.control_bar.manual_changed.connect(backend.publish_manual)

        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._on_tick)
        self._spin_timer.start(20)

        self._manual_timer = QTimer(self)
        self._manual_timer.timeout.connect(self._publish_manual_keepalive)
        self._manual_timer.start(100)

        backend.start()
        self._on_backend_info(f'Backend: {backend.name}')

    def _on_tick(self) -> None:
        self.backend.tick()

    def _publish_manual_keepalive(self) -> None:
        mode = getattr(self.control_bar, '_current_mode', 'auto')
        if mode not in ('manual', 'semi'):
            return
        linear_x, angular_z = self.control_bar.current_manual()
        if linear_x == 0.0 and angular_z == 0.0:
            return
        self.backend.publish_manual(linear_x, angular_z)

    def _on_status_update(self, status: dict) -> None:
        self.telemetry.update_status(status)
        mode = status.get('mode')
        if isinstance(mode, str):
            self.control_bar.set_mode(mode)

    def _on_image_update(self, image: QImage) -> None:
        self.camera_view.set_image(image)

    def _on_backend_info(self, message: str) -> None:
        self.backend_label.setText(message)

    def closeEvent(self, event: QCloseEvent) -> None:
        self._spin_timer.stop()
        self._manual_timer.stop()
        self.backend.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)
