"""Apply saved camera settings and optionally show the integrated preview."""

import argparse
from collections import deque
from pathlib import Path
import re
import signal
import sys
import time

import rclpy
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from robot_bringup.camera_settings import (
    AUTO_KEYS, MANUAL_AUTO, load_camera_settings, resolve_names,
    save_camera_settings, write_sequence,
)


class Control:
    def __init__(self, key, name, descriptor, camera):
        self.key, self.name = key, name
        self.kind = descriptor.type
        self.bounds = None
        self.auto_values = None
        if descriptor.read_only:
            raise ValueError('읽기 전용')
        if key in AUTO_KEYS:
            if self.kind == Parameter.Type.BOOL.value:
                self.auto_values = (False, True)
            elif camera == 'webcam' and key == AUTO_KEYS[0]:
                menu = {int(index): label.strip().lower() for index, label in
                        re.findall(r'(\d+)\s*-\s*([^,]+)', descriptor.additional_constraints)}
                # UVC manual=1; C920 auto is aperture-priority=3. Some
                # devices support full auto=0 instead; use the advertised menu.
                automatic = next((i for i in (3, 0) if i in menu), None)
                if 1 not in menu or automatic is None:
                    raise ValueError('자동/수동 노출 메뉴를 확인할 수 없음')
                self.auto_values = (1, automatic)
            else:
                raise ValueError('지원하지 않는 자동 모드 형식')
        elif self.kind == Parameter.Type.INTEGER.value and descriptor.integer_range:
            bounds = descriptor.integer_range[0]
            low = max(-2147483647, bounds.from_value)
            high = min(2147483647, bounds.to_value)
            step = max(1, bounds.step)
            if low > high:
                raise ValueError('잘못된 조절 범위')
            self.bounds = (low, high, step)
        else:
            raise ValueError('정수 조절 범위를 제공하지 않음')

    def encode(self, value):
        if self.auto_values is not None:
            if type(value) is not bool:
                raise ValueError(f'{self.key}: true/false 필요')
            return self.auto_values[int(value)]
        low, high, step = self.bounds
        if type(value) is not int or not low <= value <= high or (value - low) % step:
            raise ValueError(f'{self.key}: {low}–{high}, 간격 {step} 범위 밖의 값 {value}')
        return value

    def decode(self, value):
        if self.auto_values is not None:
            if type(value) is not type(self.auto_values[0]):
                raise ValueError(f'{self.key}: 잘못된 드라이버 응답')
            # Other UVC automatic modes still count as automatic.
            return value != self.auto_values[0]
        self.encode(value)
        return value

    def snap(self, value):
        low, high, step = self.bounds
        return low + max(0, min((high - low) // step, round((value - low) / step))) * step


class CameraControlNode(Node):
    def __init__(self, camera, camera_node, settings_path, ready_topic=''):
        super().__init__(camera + '_controls')
        self.camera = camera
        self.camera_node = camera_node.rstrip('/')
        self.settings_path = settings_path
        self.profile = load_camera_settings(settings_path, camera)
        self.parameter_clients = {name: self.create_client(service, self.camera_node + '/' + name)
                        for name, service in (
                            ('list_parameters', ListParameters),
                            ('describe_parameters', DescribeParameters),
                            ('get_parameters', GetParameters),
                            ('set_parameters', SetParameters))}
        self.controls = {}
        self.unsupported = {}
        self.current = None
        self.status = '카메라 설정 연결 대기 중…'
        self.startup_done = False
        self.startup_error = False
        self.preview_ready = not ready_topic
        self.ready_topic = ready_topic
        if ready_topic:
            self.create_subscription(
                Bool, ready_topic, self._on_ready,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._future = None
        self._operation = None
        self._queue = deque()
        self._expected = {}
        self._verify_pending = False
        self._save_requested = False
        self._retry_at = 0.0
        self._last_read = 0.0
        self._startup_deadline = time.monotonic() + 40.0
        self._names = {}
        self._last_discovery = None
        self.telemetry = None

    def start_telemetry(self):
        if self.telemetry is None:
            from robot_bringup.camera_telemetry import CameraTelemetry
            self.telemetry = CameraTelemetry(self)

    def live_reading(self, key):
        return self.telemetry.values.get(key) if self.telemetry else (None, '')

    def destroy_node(self):
        if self.telemetry is not None:
            self.telemetry.close()
            self.telemetry = None
        return super().destroy_node()

    def _on_ready(self, message):
        self.preview_ready = bool(message.data)

    @property
    def busy(self):
        return bool(self._future or self._queue or self._verify_pending or self._save_requested)

    def request_changes(self, values):
        if self.current is None or self.busy:
            return False
        try:
            values = dict(values)
            missing_live = []
            # Keep the latest measured values when leaving automatic mode.
            # Never copy a missing/stale reading into a manual control.
            for auto in AUTO_KEYS:
                switching = values.get(auto) is False or any(
                    MANUAL_AUTO.get(key) == auto for key in values)
                if switching and values.get(auto) is not True and auto in self.controls:
                    values[auto] = False
                    for key, parent in MANUAL_AUTO.items():
                        if parent == auto and key in self.current:
                            live, _ = self.live_reading(key)
                            if key not in values and self.current.get(auto) and self.telemetry:
                                if live is None:
                                    missing_live.append(key)
                                else:
                                    values[key] = self.controls[key].snap(live)
                            values.setdefault(key, self.current[key])
            self._prepare(values)
            self._manual_fallback = bool(missing_live)
            self.status = ('현재값이 없는 항목은 표시된 수동 설정값으로 적용 중…'
                           if missing_live else '카메라 설정 적용 중…')
            return True
        except ValueError as exc:
            self.status = str(exc)
            return False

    def request_save(self):
        if self.current is not None and not self.busy:
            self._save_requested = True

    def _prepare(self, values):
        sequence = write_sequence(values)
        # Validate the entire request before mutating any parameter.
        queue = []
        expected = {}
        for key, value in sequence:
            if key not in self.controls:
                # A camera without an automatic mode can still expose manual controls.
                if key in AUTO_KEYS and key not in values:
                    continue
                raise ValueError(f'{key}: 카메라가 이 설정을 지원하지 않습니다')
            control = self.controls[key]
            queue.append((control.name, control.encode(value)))
            expected[key] = value
        self._queue = deque(queue)
        self._expected = expected
        self._verify_pending = True

    def _call(self, operation, service, request):
        self._future = self.parameter_clients[service].call_async(request)
        self._operation = operation
        self._deadline = time.monotonic() + 4.0

    def _read(self, operation='read'):
        self._call(operation, 'get_parameters', GetParameters.Request(
            names=[control.name for control in self.controls.values()]))

    def _fail(self, error):
        self.status = f'카메라 설정 실패: {error}'
        self.get_logger().error(self.status)
        self._queue.clear()
        self._verify_pending = self._save_requested = False
        self._expected = {}
        self.current = None
        self._retry_at = time.monotonic() + 1.0
        if not self.startup_done:
            self.startup_error = True
            self.startup_done = True

    def poll(self):
        if self.telemetry is not None:
            self.telemetry.poll()
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
        if not self.startup_done and now > self._startup_deadline:
            self._fail('40초 동안 카메라 설정을 준비하지 못했습니다')
        if now < self._retry_at:
            return
        if not all(client.service_is_ready() for client in self.parameter_clients.values()):
            self.current = None
            self.status = f'카메라 연결 대기: {self.camera_node}'
            return
        if not self.controls:
            self._call('list', 'list_parameters', ListParameters.Request())
        elif self._queue:
            name, value = self._queue.popleft()
            self._call('write', 'set_parameters', SetParameters.Request(parameters=[
                Parameter(name, value=value).to_parameter_msg()]))
        elif self._verify_pending:
            self._read('verify')
        elif self._save_requested:
            self._save_requested = False
            self._read('save')
        elif self.current is None or now - self._last_read > 1.0:
            self._read()

    def _complete(self, operation, response):
        if operation == 'list':
            self._names = resolve_names(self.camera, response.result.names)
            # RealSense advertises services before finishing sensor parameter
            # declarations. Wait for a stable list before deciding support.
            if not self._names or self._names != self._last_discovery:
                self._last_discovery = dict(self._names)
                self._retry_at = time.monotonic() + 0.5
                return
            self._call('describe', 'describe_parameters', DescribeParameters.Request(
                names=list(self._names.values())))
        elif operation == 'describe':
            if len(response.descriptors) != len(self._names):
                raise ValueError('카메라 설정 설명 응답 누락')
            for (key, name), descriptor in zip(self._names.items(), response.descriptors):
                try:
                    self.controls[key] = Control(key, name, descriptor, self.camera)
                except ValueError as exc:
                    self.unsupported[key] = str(exc)
            if not self.controls:
                raise ValueError('조절할 수 있는 카메라 설정 없음')
            if not self.startup_done:
                profile = {key: value for key, value in self.profile.items()
                           if key in self.controls}
                missing = set(self.profile) - self.controls.keys()
                # Default automatic settings may be absent on a fixed-function
                # device, but explicit saved manual values must never vanish silently.
                if missing - set(AUTO_KEYS):
                    raise ValueError('저장된 설정을 지원하지 않음: ' + ', '.join(sorted(missing)))
                self._prepare(profile)
            else:
                self._read()
        elif operation == 'write':
            if len(response.results) != 1 or not response.results[0].successful:
                reason = response.results[0].reason if response.results else '응답 없음'
                raise RuntimeError(reason)
            # Humble v4l2_camera applies changes from parameter events. Give
            # auto-off time to reach the device before setting manual exposure.
            self._retry_at = time.monotonic() + 0.15
        else:
            if len(response.values) != len(self.controls):
                raise ValueError('카메라 파라미터 응답 누락')
            values = {key: control.decode(parameter_value_to_python(value))
                      for (key, control), value in zip(self.controls.items(), response.values)}
            if operation == 'verify':
                if any(values.get(key) != value for key, value in self._expected.items()):
                    raise RuntimeError('요청값과 드라이버 응답이 다릅니다')
                self._verify_pending = False
                self._expected = {}
                self.status = '설정 적용 확인 · 다음 실행에도 쓰려면 저장하세요'
                if getattr(self, '_manual_fallback', False):
                    self.status = '일부 현재값을 읽지 못해 수동 설정값을 적용했습니다 · 영상을 확인하세요'
                self._manual_fallback = False
                if not self.startup_done:
                    self.status = f'저장된 설정 적용 완료: {self.settings_path}'
                    self.get_logger().info(self.status)
                    self.startup_done = True
            elif operation == 'save':
                save_camera_settings(self.settings_path, self.camera, values)
                self.status = f'저장 완료: {self.settings_path}'
            self.current = values
            self._last_read = time.monotonic()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=('webcam', 'realsense'), required=True)
    parser.add_argument('--camera-node', required=True)
    parser.add_argument('--settings', required=True)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--image-topic', default='')
    parser.add_argument('--ready-topic', default='')
    args = parser.parse_args(argv)
    if args.preview and not args.image_topic:
        parser.error('--preview requires --image-topic')
    settings = str(Path(args.settings).expanduser().resolve(strict=True))
    rclpy.init(args=[])
    node = CameraControlNode(args.camera, args.camera_node, settings, args.ready_topic)
    try:
        # Apply startup settings immediately. Defer Qt/image subscriptions until
        # the existing YOLO readiness signal to avoid GPU startup contention.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            node.poll()
            if node.startup_done:
                if not args.preview:
                    return 1 if node.startup_error else 0
                if node.preview_ready:
                    break
        if not rclpy.ok():
            return 0
        from PyQt5 import QtCore, QtWidgets
        from robot_bringup.camera_tuner import CameraTunerWindow
        app = QtWidgets.QApplication([sys.argv[0]])
        window = CameraTunerWindow(node, args.image_topic)
        # Queue closing so a signal between construction and app.exec_()
        # cannot close an unseen window that window.show() then reopens.
        signal.signal(signal.SIGINT, lambda *_: QtCore.QTimer.singleShot(0, window.close))
        signal.signal(signal.SIGTERM, lambda *_: QtCore.QTimer.singleShot(0, window.close))
        window.show()
        return app.exec_()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
