"""Read camera values without changing controls or opening a video stream."""

import fcntl
import json
import os
import queue
import struct
import threading
import time

from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from realsense2_camera_msgs.msg import Metadata

from robot_bringup.camera_settings import NUMERIC_KEYS


MAX_AGE = 2.5
METADATA_KEYS = {
    'exposure': 'actual_exposure', 'gain': 'gain_level',
    'white_balance': 'white_balance', 'brightness': 'brightness',
    'contrast': 'contrast', 'saturation': 'saturation', 'sharpness': 'sharpness',
}
# Linux videodev2.h / v4l2-controls.h: VIDIOC_G_CTRL, never S_CTRL/STREAMON.
VIDIOC_G_CTRL = 0xC008561B
V4L2_IDS = {
    'exposure': 0x009A0902, 'gain': 0x00980913,
    'white_balance': 0x0098091A, 'brightness': 0x00980900,
    'contrast': 0x00980901, 'saturation': 0x00980902, 'sharpness': 0x0098091B,
}


def numeric_values(values):
    return {key: value for key, value in values.items()
            if key in NUMERIC_KEYS and type(value) is int
            and (key not in ('exposure', 'white_balance') or value > 0)
            and (key == 'brightness' or value >= 0)}


def parse_metadata(payload):
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return {}
    if not isinstance(data, dict):
        return {}
    # D400 manual_white_balance is a preset, and
    # auto_white_balance_temperature is an ON/OFF flag, not Kelvin.
    return numeric_values({key: data.get(field) for key, field in METADATA_KEYS.items()})


class LiveValues:
    def __init__(self):
        self.samples = {}

    def update(self, values, source, at=None):
        self.samples[source] = (time.monotonic() if at is None else at,
                                numeric_values(values))

    def get(self, key, now=None):
        now = time.monotonic() if now is None else now
        # Per-frame readings take precedence over the slower device query.
        for source in ('frame', 'device'):
            at, values = self.samples.get(source, (0.0, {}))
            if 0 <= now - at <= MAX_AGE and key in values:
                return values[key], source
        return None, ''


def read_v4l2(device):
    """Query the same video device as the driver, including UVC auto updates."""
    values = {}
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        for key, control_id in V4L2_IDS.items():
            data = bytearray(struct.pack('=Ii', control_id, 0))
            try:
                fcntl.ioctl(fd, VIDIOC_G_CTRL, data, True)
            except OSError:
                continue
            values[key] = struct.unpack('=Ii', data)[1]
    finally:
        os.close(fd)
    return numeric_values(values)


class CameraTelemetry:
    def __init__(self, node):
        self.node = node
        self.values = LiveValues()
        self._future = None
        self._next = 0.0
        self._stop = threading.Event()
        self._results = queue.Queue(maxsize=1)
        self._worker = None
        self._subscription = None
        suffix = '/get_rgb_controls' if node.camera == 'realsense' else '/get_parameters'
        self._client = node.create_client(GetParameters, node.camera_node + suffix)
        if node.camera == 'realsense':
            self._subscription = node.create_subscription(
                Metadata, node.camera_node + '/color/metadata', self._metadata,
                qos_profile_sensor_data)

    def _metadata(self, message):
        self.values.update(parse_metadata(message.json_data), 'frame')

    def _read_webcam(self, device):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                values = read_v4l2(device)
            except OSError:
                values = {}
            try:
                self._results.get_nowait()
            except queue.Empty:
                pass
            self._results.put_nowait((values, started))
            self._stop.wait(0.5)

    def poll(self):
        now = time.monotonic()
        try:
            values, at = self._results.get_nowait()
            self.values.update(values, 'device', at)
        except queue.Empty:
            pass
        if self._future is not None:
            if not self._future.done() and now < self._deadline:
                return
            future, self._future = self._future, None
            if not future.done():
                future.cancel()
                return
            try:
                response = future.result()
                if response is None:
                    return
                values = [parameter_value_to_python(value) for value in response.values]
                if self.node.camera == 'realsense':
                    if len(values) == len(NUMERIC_KEYS):
                        self.values.update(dict(zip(NUMERIC_KEYS, values)), 'device', self._requested_at)
                elif len(values) == 1 and isinstance(values[0], str) and values[0]:
                    self._worker = threading.Thread(target=self._read_webcam,
                                                    args=(values[0],), daemon=True)
                    self._worker.start()
            except Exception:
                # Telemetry failure must not interrupt preview or settings.
                self.values.update({}, 'device')
            return
        if self._worker is not None or self.node.busy or now < self._next:
            return
        self._next = now + 0.5
        if self._client.service_is_ready():
            names = list(NUMERIC_KEYS) if self.node.camera == 'realsense' else ['video_device']
            self._requested_at = now
            self._deadline = now + 1.5
            self._future = self._client.call_async(GetParameters.Request(names=names))

    def close(self):
        self._stop.set()
        if self._future is not None:
            self._future.cancel()
        if self._subscription is not None:
            self.node.destroy_subscription(self._subscription)
        self.node.destroy_client(self._client)
