"""Telemetry labels for vehicle status."""

from PySide6.QtWidgets import QGridLayout, QLabel, QWidget


def _mode_color(status: dict) -> str:
    if status.get('collision'):
        return '#ff4444'
    if status.get('stopped'):
        return '#ff9933'
    mode = status.get('mode', 'unknown')
    if mode == 'manual':
        return '#3399ff'
    if mode == 'semi':
        return '#e6cc00'
    if mode == 'auto':
        return '#33cc66'
    return '#dddddd'


class TelemetryPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = QLabel('Mode: —')
        self._speed = QLabel('Vitesse: — m/s')
        self._lateral = QLabel('Erreur lat.: — m')
        self._collision = QLabel('Collision: —')
        self._state = QLabel('État: —')
        self._command = QLabel('Commande: —')

        layout = QGridLayout(self)
        layout.addWidget(self._mode, 0, 0)
        layout.addWidget(self._speed, 0, 1)
        layout.addWidget(self._lateral, 0, 2)
        layout.addWidget(self._collision, 1, 0)
        layout.addWidget(self._state, 1, 1)
        layout.addWidget(self._command, 1, 2)

    def update_status(self, status: dict) -> None:
        mode = status.get('mode', 'unknown')
        speed = float(status.get('speed', 0.0))
        target = float(status.get('target_speed', 0.0))
        lateral = float(status.get('lateral_error', 0.0))
        collision = bool(status.get('collision', False))
        stopped = bool(status.get('stopped', False))
        cmd_speed = float(status.get('cmd_speed', 0.0))
        cmd_steer = float(status.get('cmd_steer', 0.0))
        reason = status.get('collision_reason', '')

        color = _mode_color(status)
        style = f'color: {color}; font-weight: 600;'

        self._mode.setText(f'Mode: {mode}')
        self._mode.setStyleSheet(style)
        self._speed.setText(
            f'Vitesse: {speed:.2f} m/s (cible {target:.2f})')
        self._speed.setStyleSheet(style)
        self._lateral.setText(f'Erreur lat.: {lateral:+.2f} m')
        self._lateral.setStyleSheet(style)

        collision_text = 'OUI' if collision else 'NON'
        if collision and reason:
            collision_text += f' ({reason})'
        self._collision.setText(f'Collision: {collision_text}')
        self._collision.setStyleSheet(style)

        if stopped:
            state_text = 'STOP'
        elif collision:
            state_text = 'COLLISION'
        else:
            state_text = 'RUN'
        self._state.setText(f'État: {state_text}')
        self._state.setStyleSheet(style)

        self._command.setText(
            f'Commande: v={cmd_speed:.2f} m/s, ω={cmd_steer:.2f} rad/s')
        self._command.setStyleSheet(style)
