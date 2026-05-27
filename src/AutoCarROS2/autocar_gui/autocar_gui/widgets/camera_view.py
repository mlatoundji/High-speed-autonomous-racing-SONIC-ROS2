"""Camera display widget."""

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget


class CameraView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._label = QLabel('En attente du flux caméra…')
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setMinimumHeight(360)
        self._label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._label.setStyleSheet(
            'background-color: #111; color: #bbb; border: 1px solid #333;')

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._label)

    def set_placeholder(self, text: str) -> None:
        self._label.setPixmap(QPixmap())
        self._label.setText(text)

    def set_image(self, image: QImage) -> None:
        if image is None or image.isNull():
            return
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self._label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._label.setPixmap(scaled)
        self._label.setText('')

    def resizeEvent(self, event):
        super().resizeEvent(event)
        pixmap = self._label.pixmap()
        if pixmap is not None and not pixmap.isNull():
            self._label.setPixmap(
                pixmap.scaled(
                    self._label.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                ))
