"""Mode buttons and manual drive sliders."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class ControlBar(QWidget):
    mode_selected = Signal(str)
    stop_requested = Signal()
    resume_requested = Signal()
    manual_changed = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_mode = 'auto'
        self._manual_active = False

        mode_box = QGroupBox('Mode')
        mode_layout = QHBoxLayout(mode_box)
        self._manual_btn = QPushButton('Manual')
        self._semi_btn = QPushButton('Semi')
        self._auto_btn = QPushButton('Auto')
        self._stop_btn = QPushButton('Stop')
        self._resume_btn = QPushButton('Resume')
        for button in (
            self._manual_btn,
            self._semi_btn,
            self._auto_btn,
            self._stop_btn,
            self._resume_btn,
        ):
            mode_layout.addWidget(button)

        self._manual_btn.clicked.connect(lambda: self._select_mode('manual'))
        self._semi_btn.clicked.connect(lambda: self._select_mode('semi'))
        self._auto_btn.clicked.connect(lambda: self._select_mode('auto'))
        self._stop_btn.clicked.connect(self.stop_requested.emit)
        self._resume_btn.clicked.connect(self.resume_requested.emit)

        drive_box = QGroupBox('Commande manuelle')
        drive_layout = QGridLayout(drive_box)
        self._throttle = QSlider(Qt.Orientation.Horizontal)
        self._throttle.setRange(0, 600)
        self._throttle.setValue(0)
        self._steering = QSlider(Qt.Orientation.Horizontal)
        self._steering.setRange(-850, 850)
        self._steering.setValue(0)
        self._throttle_label = QLabel('Accélération: 0.00 m/s')
        self._steering_label = QLabel('Direction: 0.00 rad/s')

        drive_layout.addWidget(QLabel('Accélération'), 0, 0)
        drive_layout.addWidget(self._throttle, 0, 1)
        drive_layout.addWidget(self._throttle_label, 0, 2)
        drive_layout.addWidget(QLabel('Direction'), 1, 0)
        drive_layout.addWidget(self._steering, 1, 1)
        drive_layout.addWidget(self._steering_label, 1, 2)

        self._throttle.valueChanged.connect(self._on_slider_changed)
        self._steering.valueChanged.connect(self._on_slider_changed)

        layout = QVBoxLayout(self)
        layout.addWidget(mode_box)
        layout.addWidget(drive_box)
        self._update_slider_enabled()

    def _select_mode(self, mode: str) -> None:
        self._current_mode = mode
        self.mode_selected.emit(mode)
        self._update_slider_enabled()

    def set_mode(self, mode: str) -> None:
        self._current_mode = mode
        self._update_slider_enabled()

    def _on_slider_changed(self) -> None:
        linear_x = self._throttle.value() / 100.0
        angular_z = self._steering.value() / 1000.0
        self._throttle_label.setText(f'Accélération: {linear_x:.2f} m/s')
        self._steering_label.setText(f'Direction: {angular_z:.2f} rad/s')
        if self._current_mode in ('manual', 'semi'):
            self._manual_active = True
            self.manual_changed.emit(linear_x, angular_z)

    def consume_manual_active(self) -> bool:
        active = self._manual_active
        self._manual_active = False
        return active

    def current_manual(self) -> tuple[float, float]:
        return (
            self._throttle.value() / 100.0,
            self._steering.value() / 1000.0,
        )

    def _update_slider_enabled(self) -> None:
        enabled = self._current_mode in ('manual', 'semi')
        self._throttle.setEnabled(enabled)
        self._steering.setEnabled(enabled)
