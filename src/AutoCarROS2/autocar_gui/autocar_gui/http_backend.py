"""HTTP fallback backend using the container control API."""

from __future__ import annotations

import requests

from autocar_gui.backend import ControlBackend


class HttpBackend(ControlBackend):
    name = 'HTTP'
    camera_available = False

    def __init__(self, api_url: str = 'http://localhost:8001'):
        super().__init__()
        self.api_url = api_url.rstrip('/')
        self._status = {}
        self._session = requests.Session()
        self._poll_counter = 0

    def start(self) -> None:
        try:
            response = self._session.get(
                f'{self.api_url}/api/health', timeout=2.0)
            response.raise_for_status()
            self._emit_info(
                f'Backend HTTP actif ({self.api_url}) — caméra indisponible.')
        except requests.RequestException as exc:
            self._emit_info(
                f'Backend HTTP ({self.api_url}) — API injoignable: {exc}')

    def shutdown(self) -> None:
        self._session.close()

    def tick(self) -> None:
        self._poll_counter += 1
        if self._poll_counter % 5 == 0:
            self._poll_status()

    def set_mode(self, mode: str) -> None:
        self._post('/api/control/mode', {'mode': mode})

    def stop_vehicle(self) -> None:
        self._post('/api/control/stop', {})

    def resume_auto(self) -> None:
        self._post('/api/control/resume', {})

    def publish_manual(self, linear_x: float, angular_z: float) -> None:
        self._post('/api/command/manual', {
            'linear_x': float(linear_x),
            'angular_z': float(angular_z),
            'duration_sec': 0.0,
            'rate_hz': 10.0,
        })

    def _poll_status(self) -> None:
        try:
            response = self._session.get(
                f'{self.api_url}/api/control/status', timeout=3.0)
            response.raise_for_status()
            payload = response.json()
            status = payload.get('status')
            if isinstance(status, dict):
                self._status = status
                self._emit_status(dict(self._status))
        except requests.RequestException:
            return

    def _post(self, path: str, payload: dict) -> None:
        try:
            response = self._session.post(
                f'{self.api_url}{path}', json=payload, timeout=5.0)
            response.raise_for_status()
        except requests.RequestException as exc:
            self._emit_info(f'Erreur API {path}: {exc}')
