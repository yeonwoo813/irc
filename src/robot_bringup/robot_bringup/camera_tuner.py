"""Qt camera preview and controls, kept outside the inference processes."""

import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from robot_bringup.camera_settings import AUTO_KEYS, MANUAL_AUTO, NUMERIC_KEYS


LABELS = {
    'enable_auto_exposure': '자동 노출',
    'enable_auto_white_balance': '자동 화이트밸런스',
    'exposure': '노출', 'gain': '게인', 'white_balance': '화이트밸런스 (K)',
    'brightness': '밝기', 'contrast': '대비', 'saturation': '채도', 'sharpness': '선명도',
}

STYLE = '''
QWidget { background: #101820; color: #e4ebf2; font-size: 13px; }
QLabel#title { font-size: 23px; font-weight: 600; }
QLabel#muted { color: #a5b5c4; font-size: 11px; }
QLabel#liveValue { color: #71ded0; font-size: 16px; font-weight: 600; }
QLabel#status { color: #b9c9d8; padding: 9px; background: #1a2632; border-radius: 6px; }
QGroupBox { background: #182430; border: 1px solid #304151;
            border-radius: 8px; margin-top: 10px; padding: 12px 10px 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; color: #f1f5f9; }
QGroupBox QLabel, QGroupBox QCheckBox { background: transparent; }
QCheckBox { spacing: 8px; padding-bottom: 3px; }
QCheckBox::indicator { width: 17px; height: 17px; border: 1px solid #617587; border-radius: 4px; }
QCheckBox::indicator:checked { background: #46bbae; border-color: #71ded0; }
QSpinBox { background: #0e1821; border: 1px solid #405465; border-radius: 5px;
           padding: 3px 2px 3px 5px; selection-background-color: #278e84; }
QSpinBox:focus { border-color: #71ded0; }
QSlider { background: transparent; }
QSlider::groove:horizontal { height: 4px; background: #3b4e60; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #46bbae; border-radius: 2px; }
QSlider::handle:horizontal { background: #b3ebe5; width: 13px; margin: -5px 0; border-radius: 6px; }
QPushButton#saveButton { background: #25877e; color: white; border: none; border-radius: 7px; font-weight: 600; }
QPushButton#saveButton:hover { background: #2a9a8f; }
QPushButton#saveButton:disabled { background: #263845; color: #889aa8; }
QWidget:disabled { color: #617381; }
QScrollBar:vertical { width: 8px; background: #101820; }
QScrollBar::handle:vertical { background: #405465; min-height: 30px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QSplitter::handle { background: #304151; }
'''


def image_rgb(message):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    count = channels.get(message.encoding.lower())
    if count is None:
        raise ValueError(f'지원하지 않는 영상 형식: {message.encoding}')
    if (message.width <= 0 or message.height <= 0 or
            message.step < message.width * count or
            len(message.data) < message.height * message.step):
        raise ValueError('영상 크기/데이터 길이 오류')
    frame = np.ndarray((message.height, message.width, count), dtype=np.uint8,
                       buffer=bytes(message.data), strides=(message.step, count, 1))
    if count == 1:
        return np.repeat(frame, 3, axis=2)
    frame = frame[:, :, :3]
    if message.encoding.lower().startswith('bgr'):
        frame = frame[:, :, ::-1]
    return np.ascontiguousarray(frame)


class ImageCanvas(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.image = None
        self.setMinimumSize(400, 300)

    def set_rgb(self, rgb):
        height, width = rgb.shape[:2]
        self.image = QtGui.QImage(rgb.data, width, height, rgb.strides[0],
                                 QtGui.QImage.Format_RGB888).copy()
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#141820'))
        if self.image is None:
            painter.setPen(QtCore.Qt.white)
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, '카메라 영상 대기 중…')
            return
        size = self.image.size().scaled(self.size(), QtCore.Qt.KeepAspectRatio)
        rect = QtCore.QRect((self.width() - size.width()) // 2,
                            (self.height() - size.height()) // 2,
                            size.width(), size.height())
        painter.drawImage(rect, self.image)


class CameraTunerWindow(QtWidgets.QWidget):
    def __init__(self, node, image_topic):
        super().__init__()
        self.node = node
        self._message = self._last_message = None
        self._frame_at = self._render_at = 0.0
        self._pending = {}
        self._limits = {}
        camera_title = 'RealSense RGB' if node.camera == 'realsense' else 'Webcam'
        self.setWindowTitle(camera_title + ' · 카메라 설정')
        self.resize(1320, 900)
        self.setMinimumSize(920, 600)
        self.setStyleSheet(STYLE)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(20, 16, 20, 16)
        outer.setSpacing(14)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(camera_title)
        title.setObjectName('title')
        header.addWidget(title)
        header.addStretch()
        self.frame_status = QtWidgets.QLabel('영상 연결 대기')
        self.frame_status.setObjectName('muted')
        header.addWidget(self.frame_status)
        outer.addLayout(header)

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        outer.addWidget(self.splitter, 1)
        preview = QtWidgets.QWidget()
        preview_layout = QtWidgets.QVBoxLayout(preview)
        preview_layout.setContentsMargins(0, 0, 10, 0)
        self.canvas = ImageCanvas()
        preview_layout.addWidget(self.canvas, 1)
        caption = QtWidgets.QLabel('영상에서 결과를 확인하면서 오른쪽 설정을 조절하세요.')
        caption.setObjectName('muted')
        caption.setWordWrap(True)
        preview_layout.addWidget(caption)
        self.splitter.addWidget(preview)

        side = QtWidgets.QWidget()
        side.setMinimumWidth(390)
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setContentsMargins(6, 0, 0, 0)
        intro = QtWidgets.QLabel('현재값은 자동으로 갱신됩니다.\n슬라이더나 숫자를 바꾸면 수동으로 전환됩니다.')
        intro.setWordWrap(True)
        intro.setObjectName('muted')
        side_layout.addWidget(intro)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        side_layout.addWidget(scroll, 1)
        content = QtWidgets.QWidget()
        panel = QtWidgets.QVBoxLayout(content)
        panel.setContentsMargins(0, 4, 6, 4)
        panel.setSpacing(12)
        scroll.setWidget(content)
        self.checks, self.sliders, self.spins = {}, {}, {}
        self.live_labels, self.mode_labels = {}, {}
        self.manual_labels = {}
        groups = (
            ('빛과 밝기', ('exposure', 'gain'), AUTO_KEYS[0]),
            ('화이트밸런스', ('white_balance',), AUTO_KEYS[1]),
            ('영상 보정', ('brightness', 'contrast', 'saturation', 'sharpness'), None),
        )
        for title_text, keys, automatic in groups:
            group = QtWidgets.QGroupBox(title_text)
            layout = QtWidgets.QVBoxLayout(group)
            layout.setSpacing(4)
            if automatic:
                check = QtWidgets.QCheckBox(LABELS[automatic])
                check.toggled.connect(lambda checked, key=automatic: self._edited(key, checked))
                layout.addWidget(check)
                self.checks[automatic] = check
            else:
                note = QtWidgets.QLabel('수동 조절 · 현재 적용값 표시')
                note.setObjectName('muted')
                layout.addWidget(note)
            for key in keys:
                row = QtWidgets.QHBoxLayout()
                row.addWidget(QtWidgets.QLabel(LABELS[key]))
                mode = QtWidgets.QLabel('대기')
                mode.setObjectName('muted')
                row.addWidget(mode)
                row.addStretch()
                live = QtWidgets.QLabel('—')
                live.setObjectName('liveValue')
                live.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                row.addWidget(live)
                layout.addLayout(row)
                self.live_labels[key], self.mode_labels[key] = live, mode
                edit = QtWidgets.QHBoxLayout()
                manual = QtWidgets.QLabel('수동 설정')
                manual.setObjectName('muted')
                self.manual_labels[key] = manual
                edit.addWidget(manual)
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.setMinimumWidth(90)
                spin = QtWidgets.QSpinBox()
                spin.setMinimumWidth(92)
                spin.setKeyboardTracking(False)
                spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.PlusMinus)
                for widget in (slider, spin):
                    widget.valueChanged.connect(lambda value, key=key: self._edited(key, value))
                edit.addWidget(slider, 1)
                edit.addWidget(spin)
                layout.addLayout(edit)
                self.sliders[key], self.spins[key] = slider, spin
            panel.addWidget(group)
        panel.addStretch()
        self.save = QtWidgets.QPushButton('설정 저장 · 다음 실행에도 적용')
        self.save.setObjectName('saveButton')
        self.save.setMinimumHeight(44)
        self.save.clicked.connect(node.request_save)
        side_layout.addWidget(self.save)
        path = QtWidgets.QLabel('저장 위치 보기')
        path.setObjectName('muted')
        path.setToolTip(str(node.settings_path))
        side_layout.addWidget(path)
        self.splitter.addWidget(side)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 0)
        self.splitter.setSizes([800, 440])
        self.status = QtWidgets.QLabel(node.status)
        self.status.setWordWrap(True)
        self.status.setTextFormat(QtCore.Qt.PlainText)
        self.status.setObjectName('status')
        outer.addWidget(self.status)
        node.start_telemetry()
        self.debounce = QtCore.QTimer(self)
        self.debounce.setInterval(250)
        self.debounce.setSingleShot(True)
        self.subscription = node.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)
        self._sync_controls()

    def _on_image(self, message):
        self._message = message
        self._frame_at = time.monotonic()

    def _edited(self, key, value):
        if key not in self.node.controls:
            return
        control = self.node.controls[key]
        if control.bounds:
            value = control.snap(value)
            for widget in (self.sliders[key], self.spins[key]):
                with QtCore.QSignalBlocker(widget):
                    widget.setValue(value)
        self._pending[key] = value
        auto = MANUAL_AUTO.get(key)
        if auto in self.node.controls:
            self._pending[auto] = False
            with QtCore.QSignalBlocker(self.checks[auto]):
                self.checks[auto].setChecked(False)
        self.save.setEnabled(False)
        self.debounce.start()

    def _sync_controls(self):
        ready = self.node.current is not None
        editing = bool(self._pending) or self.node.busy
        for key in NUMERIC_KEYS:
            value, source = self.node.live_reading(key) if ready else (None, '')
            auto = bool(ready and self.node.current.get(MANUAL_AUTO.get(key), False))
            unit = (' µs' if self.node.camera == 'realsense' else ' ×100 µs') if key == 'exposure' else (' K' if key == 'white_balance' else '')
            self.live_labels[key].setText(f'{value:,}{unit}' if value is not None else '—')
            mode = '자동' if auto else '수동'
            state = {'frame': '프레임', 'device': '장치 조회'}.get(source, '현재값 미수신')
            if key not in self.node.controls and value is None:
                state = '지원 안 함' if ready else '연결 대기'
            self.mode_labels[key].setText(f'{mode} · {state}')
            self.live_labels[key].setToolTip(
                '최근 영상 프레임의 촬영값' if source == 'frame' else
                '카메라 장치가 보고한 현재값' if source == 'device' else
                '최신값을 받지 못했습니다. 수동 설정값을 현재값으로 대신 표시하지 않습니다.')
            self.manual_labels[key].setToolTip(
                '자동 해제 시 최신 현재값을 우선 사용합니다. 읽지 못한 항목은 이 수동값을 적용합니다.')
            self.manual_labels[key].setText('조절 시 수동' if auto and value is not None else '수동 설정')
        for key in AUTO_KEYS + NUMERIC_KEYS:
            control = self.node.controls.get(key)
            widgets = ((self.checks[key],) if key in AUTO_KEYS
                       else (self.sliders[key], self.spins[key]))
            for widget in widgets:
                widget.setEnabled(ready and control is not None)
                widget.setToolTip(self.node.unsupported.get(key, '') if control is None
                                  else control.name)
            if control is None:
                continue
            if control.bounds and self._limits.get(key) != control.bounds:
                self._limits[key] = control.bounds
                low, high, step = control.bounds
                for widget in widgets:
                    with QtCore.QSignalBlocker(widget):
                        widget.setRange(low, high)
                        widget.setSingleStep(step)
            if ready and not editing and not any(widget.hasFocus() for widget in widgets):
                for widget in widgets:
                    with QtCore.QSignalBlocker(widget):
                        if key in AUTO_KEYS:
                            widget.setChecked(self.node.current[key])
                        else:
                            value, _ = self.node.live_reading(key)
                            auto = self.node.current.get(MANUAL_AUTO.get(key), False)
                            widget.setValue(control.snap(value) if auto and value is not None
                                            else self.node.current[key])
        self.save.setEnabled(ready and not editing)

    def _tick(self):
        if not rclpy.ok():
            self.close()
            return
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.node.poll()
        dragging = any(slider.isSliderDown() for slider in self.sliders.values())
        if self._pending and not self.debounce.isActive() and not dragging and not self.node.busy:
            values, self._pending = self._pending, {}
            self.node.request_changes(values)
        self._sync_controls()
        self.status.setText(self.node.status)
        now = time.monotonic()
        if self._message is not None and self._message is not self._last_message and now - self._render_at >= 0.1:
            self._last_message, self._render_at = self._message, now
            try:
                self.canvas.set_rgb(image_rgb(self._message))
                self.frame_status.setText(f'영상 수신 중 · {self._message.width} × {self._message.height}')
            except ValueError as exc:
                self.frame_status.setText(f'영상 표시 실패: {exc}')
        elif self._frame_at and now - self._frame_at > 2.0:
            self.frame_status.setText('영상 수신 중단 · 마지막 화면 표시 중')

    def closeEvent(self, event):
        self.timer.stop()
        self.debounce.stop()
        self.node.destroy_subscription(self.subscription)
        event.accept()
