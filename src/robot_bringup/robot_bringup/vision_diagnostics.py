"""Bounded event log for motion decisions, control changes and ROS warnings.

No periodic sampling, camera subscriptions, trace socket, system polling or
pre-incident history. Disk I/O stays in the independent writer thread.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import shutil
import threading
import time


def read_text(path, limit=16384):
    try:
        with open(path) as stream:
            return stream.read(limit).strip()
    except OSError:
        return None


def startup_configuration(settings):
    text = read_text(settings) if settings else None
    return {
        'settings_path': settings,
        'settings_sha256': hashlib.sha256(text.encode()).hexdigest() if text is not None else None,
        'settings_text': text,
        'quiet_logs': os.environ.get('IRC_QUIET_LOGS'),
    }


class BoundedWriter:
    """The only disk writer; queue overflow/disk failure cannot block a node."""

    def __init__(self, directory, max_bytes=8 * 1024 * 1024, files=3):
        self.directory = Path(directory)
        self.max_bytes = max_bytes
        self.files = files
        self.queue = queue.Queue(maxsize=128)
        self.dropped = 0
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, record):
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _rotate(self):
        oldest = self.directory / f'records.{self.files - 1}.jsonl'
        oldest.unlink(missing_ok=True)
        for index in range(self.files - 2, -1, -1):
            source = self.directory / f'records.{index}.jsonl'
            if source.exists():
                source.rename(self.directory / f'records.{index + 1}.jsonl')

    def _run(self):
        stream = None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory / 'records.0.jsonl'
            stream = path.open('a')
            size = path.stat().st_size
            last_flush = time.monotonic()
            while not self.stopping.is_set() or not self.queue.empty():
                try:
                    record = self.queue.get(timeout=0.2)
                except queue.Empty:
                    if time.monotonic() - last_flush >= 1:
                        stream.flush()
                        last_flush = time.monotonic()
                    continue
                line = json.dumps(record, ensure_ascii=False, separators=(',', ':')) + '\n'
                length = len(line.encode())
                if size + length > self.max_bytes:
                    stream.close()
                    self._rotate()
                    stream = path.open('w')
                    size = 0
                stream.write(line)
                size += length
                if time.monotonic() - last_flush >= 1:
                    stream.flush()
                    last_flush = time.monotonic()
                # Bound disk traffic even during a burst of warning events.
                self.stopping.wait(length / 65536)
        except (OSError, ValueError) as exc:
            self.error = str(exc)
        finally:
            if stream is not None and not stream.closed:
                try:
                    stream.close()
                except OSError:
                    pass

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=1.0)


def make_node(writer):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from decision.motion_log import MAX_RECORD_BYTES, TOPIC
    from msgs.msg import MotionCommand, MotionEnd
    from rcl_interfaces.msg import Log

    class Recorder(Node):
        def __init__(self):
            super().__init__('vision_diagnostics', enable_rosout=False)
            self.control_state = {}
            self.warn_second, self.warn_count = 0, 0
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions_kept = []
            for name, topic in {'ball_active': '/vision/ball_active', 'hoop_active': '/vision/hoop_active',
                                'armed': '/mission/armed', 'webcam_ready': '/vision/webcam_yolo_ready',
                                'rs_ready': '/vision/realsense_yolo_ready'}.items():
                self.subscriptions_kept.append(self.create_subscription(
                    Bool, topic, lambda msg, name=name: self.on_context(name, bool(msg.data)), latched))
            self.subscriptions_kept.append(self.create_subscription(
                MotionCommand, '/motion_command', lambda msg: self.on_context('motion_command', msg.command), 10))
            self.subscriptions_kept.append(self.create_subscription(
                MotionEnd, '/motion_end', lambda msg: self.on_context('motion_state',
                    {'ended': msg.motion_end, 'ready': msg.motion_ready}), 10))
            self.subscriptions_kept.append(self.create_subscription(Log, '/rosout', self.on_log,
                QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            self.subscriptions_kept.append(self.create_subscription(
                String, TOPIC, self.on_decision,
                QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)))

        def on_decision(self, msg):
            if len(msg.data.encode()) > MAX_RECORD_BYTES:
                return
            try:
                record = json.loads(msg.data)
            except (ValueError, TypeError):
                return
            if isinstance(record, dict) and record.get('type') == 'motion_decision' and record.get('v') == 1:
                self.emit(record)

        def on_context(self, name, value):
            if self.control_state.get(name) != value or name == 'motion_command':
                self.control_state[name] = value
                self.emit({'type': 'control', 'name': name, 'value': value})

        def on_log(self, msg):
            if msg.level < 30:
                return
            second = int(time.monotonic())
            if second != self.warn_second:
                self.warn_second, self.warn_count = second, 0
            self.warn_count += 1
            if self.warn_count <= 30:
                self.emit({'type': 'ros_warning', 'node': msg.name, 'level': msg.level,
                           'message_stamp_ns': msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec,
                           'message': msg.msg[:2048]})
            elif self.warn_count == 31:
                self.emit({'type': 'ros_warnings_rate_limited'})

        def emit(self, record):
            writer.put({'mono': time.monotonic(), 'wall_ns': time.time_ns(), **record})

    return Recorder()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--settings', default='')
    args = parser.parse_args()
    try:
        os.nice(10)
    except OSError:
        pass
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    # Only recorder-owned run-* directories; at most ten runs, 24 MiB each.
    previous = sorted((p for p in directory.parent.glob('run-*')
                       if p.is_dir() and p != directory), key=lambda p: p.stat().st_mtime, reverse=True)
    for old in previous[9:]:
        shutil.rmtree(old, ignore_errors=True)
    writer = BoundedWriter(directory)
    node = None
    import rclpy
    try:
        rclpy.init()
        node = make_node(writer)
        node.emit({'type': 'session', 'boot_id': read_text('/proc/sys/kernel/random/boot_id'),
                   'pid': os.getpid(), 'recording_mode': 'events_only'})
        node.emit({'type': 'configuration', **startup_configuration(args.settings)})
        print(f'[VisionDiagnostics] {directory}', flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT can be followed by launch forwarding another one.
        # Finish this recorder's bounded flush before accepting more interrupts.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.emit({'type': 'recorder_stop', 'disk_queue_dropped': writer.dropped, 'disk_error': writer.error})
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            writer.close()


if __name__ == '__main__':
    main()
