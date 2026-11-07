"""Independent aligned-depth viewer with verified ROS exposure controls."""

import argparse
from collections import deque
from pathlib import Path
import signal
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from robot_bringup.depth_settings import (
    DEFAULTS, PARAMETER_NAMES, load_depth_display_settings,
    save_depth_settings, validate_settings,
)


def depth_metres(message, scale):
    """Decode depth with row padding/endianness, preserving invalid pixels."""
    encoding = message.encoding.upper()
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError(f'Unsupported depth encoding: {message.encoding}')
    dtype = np.dtype(('>' if message.is_bigendian else '<') +
                     ('u2' if encoding == '16UC1' else 'f4'))
    if (message.width <= 0 or message.height <= 0 or
            message.step < message.width * dtype.itemsize or
            len(message.data) < message.height * message.step):
        raise ValueError('Invalid depth frame dimensions/data length')
    frame = np.ndarray(
        (message.height, message.width), dtype=dtype,
        buffer=bytes(message.data), strides=(message.step, dtype.itemsize),
    ).astype(np.float32)
    return frame * scale if encoding == '16UC1' else frame


def depth_statistics(depth, roi, minimum, maximum):
    if roi is not None:
        x1, y1, x2, y2 = roi
        depth = depth[y1:y2, x1:x2]
    valid = np.isfinite(depth) & (depth > 0)
    in_range = valid & (depth >= minimum) & (depth <= maximum)
    total = depth.size
    values = depth[in_range]
    return {
        'valid_percent': 100.0 * np.count_nonzero(valid) / max(total, 1),
        'range_percent': 100.0 * values.size / max(total, 1),
        'median_m': float(np.median(values)) if values.size else None,
    }


def depth_rgb(depth, minimum, maximum):
    valid = np.isfinite(depth) & (depth > 0)
    normalized = np.zeros(depth.shape, dtype=np.float32)
    normalized[valid] = np.clip(
        (depth[valid] - minimum) / (maximum - minimum), 0, 1,
    )
    # Fixed range: near=red, far=blue. No interpolation or hole filling.
    anchors = np.array([[255, 65, 45], [255, 220, 45], [40, 210, 100],
                        [35, 190, 255], [55, 65, 220]], dtype=np.float32)
    rgb = np.stack([np.interp(normalized, np.linspace(0, 1, 5), anchors[:, c])
                    for c in range(3)], axis=-1).astype(np.uint8)
    rgb[~valid] = 0
    return rgb


class DepthTunerNode(Node):
    """Serialize writes: auto off -> exposure -> readback -> optional save."""

    def __init__(self, camera_node, settings_path, depth_topic):
        super().__init__('realsense_depth_tuner')
        self.settings_path = settings_path
        self.camera_node = camera_node.rstrip('/')
        self.get_client = self.create_client(
            GetParameters, self.camera_node + '/get_parameters')
        self.set_client = self.create_client(
            SetParameters, self.camera_node + '/set_parameters')
        self.describe_client = self.create_client(
            DescribeParameters, self.camera_node + '/describe_parameters')
        self.create_subscription(Image, depth_topic, self._on_depth,
                                 qos_profile_sensor_data)
        self.depth_message = None
        self.frame_time = 0.0
        self.current = None
        self.exposure_limits = None
        self.pending_exposure = None
        self.save_requested = False
        self.status = '카메라 연결 대기 중…'
        self._future = None
        self._operation = None
        self._target = None
        self._deadline = 0.0
        self._last_read = 0.0
        self._retry_at = 0.0

    @property
    def busy(self):
        return (self._future is not None or self.pending_exposure is not None
                or self.save_requested)

    def _on_depth(self, message):
        self.depth_message = message
        self.frame_time = time.monotonic()

    def request_exposure(self, value):
        if self.exposure_limits is None:
            return
        low, high, step = self.exposure_limits
        value = low + round((int(value) - low) / step) * step
        self.pending_exposure = max(low, min(high, value))
        self.status = f'노출 {self.pending_exposure} µs 적용 대기…'

    def _call(self, operation, client, request):
        self._future = client.call_async(request)
        self._operation = operation
        self._deadline = time.monotonic() + 4.0

    def _get(self, operation='get'):
        self._call(operation, self.get_client,
                   GetParameters.Request(names=list(PARAMETER_NAMES)))

    def _set(self, operation, name, value):
        self._call(operation, self.set_client, SetParameters.Request(parameters=[
            Parameter('depth_module.' + name, value=value).to_parameter_msg(),
        ]))

    def _fail(self, message):
        self.status = '설정 확인 실패: ' + str(message)
        self.current = None
        self.pending_exposure = None
        self.save_requested = False
        self._target = None
        self._retry_at = time.monotonic() + 1.0
        self.get_logger().warning(self.status)

    def poll(self):
        now = time.monotonic()
        if self._future is not None:
            if not self._future.done() and now < self._deadline:
                return
            future, operation = self._future, self._operation
            self._future = None
            try:
                if not future.done():
                    future.cancel()
                    raise TimeoutError('카메라 응답 시간 초과')
                response = future.result()
                if response is None:
                    raise RuntimeError('카메라 응답 없음')
                self._complete(operation, response)
            except Exception as exc:
                self._fail(exc)
            return
        if now < self._retry_at:
            return
        if not all(c.service_is_ready() for c in (
                self.get_client, self.set_client, self.describe_client)):
            self.current = None
            self.exposure_limits = None
            self.pending_exposure = None
            self.save_requested = False
            self.status = f'카메라 연결 대기: {self.camera_node}'
            return
        if self.exposure_limits is None:
            self._call('describe', self.describe_client,
                       DescribeParameters.Request(names=['depth_module.exposure']))
        elif self.pending_exposure is not None:
            self._target, self.pending_exposure = self.pending_exposure, None
            self._set('auto_off', 'enable_auto_exposure', False)
        elif self.save_requested:
            self.save_requested = False
            self._get('save')
        elif self.current is None or now - self._last_read > 1.0:
            self._get()

    def _complete(self, operation, response):
        if operation == 'describe':
            if not response.descriptors:
                # The node's services appear before USB/sensor initialization.
                self.status = '카메라가 노출 설정을 준비하는 중…'
                self._retry_at = time.monotonic() + 1.0
                return
            descriptor = response.descriptors[0]
            if descriptor.type != Parameter.Type.INTEGER.value or descriptor.read_only:
                raise ValueError('수동 노출을 지원하지 않는 카메라')
            if not descriptor.integer_range:
                raise ValueError('카메라가 노출 조절 범위를 제공하지 않음')
            bounds = descriptor.integer_range[0]
            low, high = max(1, bounds.from_value), min(2147483647, bounds.to_value)
            if low > high:
                raise ValueError('잘못된 노출 범위')
            self.exposure_limits = (low, high, max(1, bounds.step))
            self.status = '카메라 설정 읽는 중…'
        elif operation in ('auto_off', 'exposure'):
            if len(response.results) != 1 or not response.results[0].successful:
                reason = response.results[0].reason if response.results else '응답 없음'
                raise RuntimeError(reason)
            if operation == 'auto_off':
                self._set('exposure', 'exposure', self._target)
            else:
                self._get('verify')
        else:
            if len(response.values) != len(DEFAULTS):
                raise ValueError('카메라 파라미터 응답 누락')
            values = validate_settings(dict(zip(DEFAULTS, (
                parameter_value_to_python(value) for value in response.values))))
            if operation == 'verify':
                if values['enable_auto_exposure'] or values['exposure'] != self._target:
                    raise RuntimeError('요청한 수동 노출값과 카메라 응답이 다름')
                self.status = f'적용 확인: {self._target} µs · 다음 실행에도 쓰려면 저장'
                self._target = None
            if operation == 'save':
                save_depth_settings(self.settings_path, values)
                self.status = f'저장 완료: {self.settings_path}'
            elif self.current is None and operation == 'get':
                self.status = '연결됨 · 노출 조절 시 자동 노출이 꺼집니다'
            self.current = values
            self._last_read = time.monotonic()


class DepthCanvas(QtWidgets.QWidget):
    roi_changed = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.image = None
        self.roi = None
        self._origin = None

    def image_rect(self):
        if self.image is None:
            return QtCore.QRect()
        size = self.image.size().scaled(self.size(), QtCore.Qt.KeepAspectRatio)
        return QtCore.QRect((self.width() - size.width()) // 2,
                            (self.height() - size.height()) // 2,
                            size.width(), size.height())

    def set_rgb(self, rgb):
        height, width = rgb.shape[:2]
        if self.image is not None and (self.image.width(), self.image.height()) != (width, height):
            self.clear_roi()
        self.image = QtGui.QImage(rgb.data, width, height, rgb.strides[0],
                                 QtGui.QImage.Format_RGB888).copy()
        self.update()

    def clear_roi(self):
        self.roi = None
        self._origin = None
        self.roi_changed.emit()
        self.update()

    def _pixel(self, point):
        rect = self.image_rect()
        if rect.isEmpty():
            return None
        return (int(np.clip((point.x() - rect.x()) * self.image.width() / rect.width(),
                            0, self.image.width() - 1)),
                int(np.clip((point.y() - rect.y()) * self.image.height() / rect.height(),
                            0, self.image.height() - 1)))

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.RightButton:
            self.clear_roi()
        elif event.button() == QtCore.Qt.LeftButton and self.image_rect().contains(event.pos()):
            self._origin = self._pixel(event.pos())

    def mouseMoveEvent(self, event):
        if self._origin is None:
            return
        end = self._pixel(event.pos())
        x, y = self._origin
        self.roi = (min(x, end[0]), min(y, end[1]), max(x, end[0]) + 1, max(y, end[1]) + 1)
        self.roi_changed.emit()
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self._origin is not None:
            self.mouseMoveEvent(event)
            self._origin = None

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#141820'))
        if self.image is None:
            painter.setPen(QtCore.Qt.white)
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, 'Depth 영상 대기 중…')
            return
        rect = self.image_rect()
        painter.drawImage(rect, self.image)
        if self.roi is not None:
            x1, y1, x2, y2 = self.roi
            sx, sy = rect.width() / self.image.width(), rect.height() / self.image.height()
            painter.setPen(QtGui.QPen(QtGui.QColor('#ffffff'), 2))
            painter.drawRect(QtCore.QRectF(rect.x() + x1 * sx, rect.y() + y1 * sy,
                                          (x2 - x1) * sx, (y2 - y1) * sy))


class DepthTunerWindow(QtWidgets.QWidget):
    def __init__(self, node, display):
        super().__init__()
        self.node = node
        self.display = display
        self.depth = None
        self._last_frame = None
        self._last_render = 0.0
        self._limits = None
        self._history = deque(maxlen=30)
        self.setWindowTitle('RealSense Depth · 노출 조절')
        self.resize(850, 780)
        layout = QtWidgets.QVBoxLayout(self)
        self.canvas = DepthCanvas()
        self.canvas.roi_changed.connect(self._reset_history)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(QtWidgets.QLabel(
            f'정렬된 depth · 가까움(빨강) → 멂(파랑): '
            f'{display["depth_min_m"]:g}–{display["depth_max_m"]:g} m · hole(0/NaN): 검정\n'
            '백보드 영역을 마우스로 드래그해서 선택 · 우클릭: 전체 화면'))
        self.stats = QtWidgets.QLabel('영상 대기 중')
        layout.addWidget(self.stats)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('Exposure (µs)'))
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.spin = QtWidgets.QSpinBox()
        self.spin.setSuffix(' µs')
        self.spin.setKeyboardTracking(False)
        self.slider.setRange(1, 200000)
        self.spin.setRange(1, 200000)
        self.slider.setValue(5000)
        self.spin.setValue(5000)
        row.addWidget(self.slider, 1)
        row.addWidget(self.spin)
        layout.addLayout(row)
        self.debounce = QtCore.QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(250)
        self.debounce.timeout.connect(self._apply)
        self.slider.valueChanged.connect(self._edited)
        self.spin.valueChanged.connect(self._edited)
        self.current_label = QtWidgets.QLabel('카메라 설정 읽는 중…')
        layout.addWidget(self.current_label)
        self.save = QtWidgets.QPushButton('현재 설정을 settings.ini에 저장')
        self.save.clicked.connect(self._save)
        layout.addWidget(self.save)
        self.status = QtWidgets.QLabel(node.status)
        self.status.setWordWrap(True)
        self.status.setTextFormat(QtCore.Qt.PlainText)
        layout.addWidget(self.status)
        path_label = QtWidgets.QLabel(f'설정 파일: {node.settings_path}')
        path_label.setWordWrap(True)
        path_label.setTextFormat(QtCore.Qt.PlainText)
        layout.addWidget(path_label)
        self.slider.setEnabled(False)
        self.spin.setEnabled(False)
        self.save.setEnabled(False)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)

    def _reset_history(self):
        self._history.clear()

    def _edited(self, value):
        for widget in (self.slider, self.spin):
            with QtCore.QSignalBlocker(widget):
                widget.setValue(value)
        self.save.setEnabled(False)
        self.debounce.start()

    def _apply(self):
        self._reset_history()
        self.node.request_exposure(self.spin.value())

    def _save(self):
        self.node.save_requested = True
        self.save.setEnabled(False)

    def _tick(self):
        if not rclpy.ok():
            self.close()
            return
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.node.poll()
        ready = self.node.current is not None and self.node.exposure_limits is not None
        self.slider.setEnabled(ready)
        self.spin.setEnabled(ready)
        if self.node.exposure_limits is not None and self._limits != self.node.exposure_limits:
            self._limits = self.node.exposure_limits
            low, high, step = self._limits
            for widget in (self.slider, self.spin):
                with QtCore.QSignalBlocker(widget):
                    widget.setRange(low, high)
                    widget.setSingleStep(step)
        editing = self.debounce.isActive() or self.slider.isSliderDown()
        if ready:
            values = self.node.current
            self.current_label.setText(
                f'카메라 응답: Exposure {values["exposure"]} µs · '
                f'Gain {values["gain"]} · Laser {values["laser_power"]:g} · '
                f'자동 노출 {"ON" if values["enable_auto_exposure"] else "OFF"}')
            if not editing and not self.node.busy and not self.spin.hasFocus():
                for widget in (self.slider, self.spin):
                    with QtCore.QSignalBlocker(widget):
                        widget.setValue(values['exposure'])
        else:
            self.current_label.setText('카메라 설정을 확인할 수 없습니다')
        self.save.setEnabled(ready and not editing and not self.node.busy)
        self.status.setText(self.node.status)
        now = time.monotonic()
        message = self.node.depth_message
        if message is not None and message is not self._last_frame and now - self._last_render >= 0.1:
            self._last_frame, self._last_render = message, now
            try:
                self.depth = depth_metres(message, self.display['depth_scale'])
                minimum, maximum = self.display['depth_min_m'], self.display['depth_max_m']
                self.canvas.set_rgb(depth_rgb(self.depth, minimum, maximum))
                stats = depth_statistics(self.depth, self.canvas.roi, minimum, maximum)
                median = stats['median_m']
                if median is not None:
                    self._history.append((now, median))
                while self._history and now - self._history[0][0] > 3.0:
                    self._history.popleft()
                distance = f'{median * 100:.1f} cm' if median is not None else '없음'
                jitter = (f'{np.std([v for _, v in self._history]) * 100:.2f} cm'
                          if median is not None and len(self._history) >= 2 else '—')
                self.stats.setText(
                    f'{"선택 영역" if self.canvas.roi else "전체 화면"}: '
                    f'유효 {stats["valid_percent"]:.1f}% · '
                    f'hole {100 - stats["valid_percent"]:.1f}% · '
                    f'설정 거리 범위 내 {stats["range_percent"]:.1f}%\n'
                    f'범위 내 깊이(Z) 중앙값 {distance} · 최근 3초 변동(표준편차) {jitter}')
            except Exception as exc:
                self.stats.setText(f'Depth 표시 실패: {exc}')
        elif self.node.frame_time and now - self.node.frame_time > 2.0:
            self.stats.setText('영상 수신 중단 · 마지막 화면 표시 중')
            self._history.clear()

    def closeEvent(self, event):
        self.timer.stop()
        self.debounce.stop()
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', required=True)
    parser.add_argument('--camera-node', default='/camera')
    parser.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw')
    args = parser.parse_args(argv)
    settings = str(Path(args.settings).expanduser().resolve(strict=True))
    display = load_depth_display_settings(settings)
    app = QtWidgets.QApplication([sys.argv[0]])
    rclpy.init(args=[])
    node = DepthTunerNode(args.camera_node, settings, args.depth_topic)
    window = DepthTunerWindow(node, display)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    try:
        window.show()
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
