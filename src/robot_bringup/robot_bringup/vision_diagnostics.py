"""Independent, bounded flight recorder for camera stalls (no image data).

Detector checkpoints use nonblocking local datagrams. ROS subscriptions carry
only CameraInfo, health and control metadata, never Image. All disk writes and
journal reads happen outside camera/motion processes and outside ROS callbacks.
"""

import argparse
from collections import deque
import copy
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import shutil
import socket
import stat
import subprocess
import threading
import time

from vision_trace import MAGIC, PACKET, SOURCES, STAGES


def read_text(path, limit=16384):
    try:
        with open(path) as stream:
            return stream.read(limit).strip()
    except OSError:
        return None


def startup_configuration(settings):
    """Enough identity to compare first/second runs without guessing by date."""
    text = read_text(settings) if settings else None
    devices = {}
    for path in Path('/sys/bus/usb/devices').glob('*'):
        vendor = read_text(path / 'idVendor', 32)
        if vendor:
            devices[path.name] = {key: read_text(path / key, 128) for key in
                ('idVendor', 'idProduct', 'product', 'busnum', 'devnum', 'speed',
                 'power/control', 'power/autosuspend_delay_ms', 'power/runtime_status')}
    return {'settings_path': settings,
            'settings_sha256': hashlib.sha256(text.encode()).hexdigest() if text is not None else None,
            'settings_text': text, 'usb_devices': devices,
            'quiet_logs': os.environ.get('IRC_QUIET_LOGS'),
            'kernel': os.uname().release}


class BoundedWriter:
    """The only disk writer; queue overflow/disk failure cannot block a node."""

    def __init__(self, directory, max_bytes=8 * 1024 * 1024, files=3):
        self.directory = Path(directory)
        self.max_bytes = max_bytes
        self.files = files
        self.queue = queue.Queue(maxsize=512)
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
                # At most ~64 KiB/s even when dumping the pre-incident ring.
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


class TraceReceiver:
    def __init__(self, address):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        Path(address).parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        if Path(address).exists() and stat.S_ISSOCK(Path(address).stat().st_mode):
            Path(address).unlink()  # This launch's stale socket after recorder respawn.
        self.socket.bind(address)
        os.chmod(address, 0o600)
        self.socket.settimeout(0.2)
        self.address = address
        self.states = {}
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def accept(self, packet, received):
        if len(packet) != PACKET.size:
            return
        magic, pid, source, stage, mono, stamp1, stamp2, seq, dropped = PACKET.unpack(packet)
        if magic != MAGIC or source >= len(SOURCES) or stage >= len(STAGES):
            return
        name, phase = SOURCES[source], STAGES[stage]
        with self.lock:
            state = self.states.get(name)
            if state is None or state['pid'] != pid:
                state = {'pid': pid, 'phases': {}, 'max_phase_sec': {}, 'sequence': 0}
                self.states[name] = state
            if seq <= state['sequence']:
                return
            state['sequence'] = seq
            state['sender_dropped'] = dropped
            state['transport_sec'] = max(0.0, received - mono)
            had_complete = 'complete' in state['phases']
            old = state['phases'].get(phase, {})
            state['phases'][phase] = {
                'count': old.get('count', 0) + 1, 'mono': mono,
                'stamp_ns': [stamp1, stamp2],
            }
            if phase not in ('raw_color', 'raw_depth'):
                previous = state.get('stage')
                if had_complete and previous and state.get('stage_mono') is not None:
                    duration = max(0.0, mono - state['stage_mono'])
                    state['max_phase_sec'][previous] = max(
                        state['max_phase_sec'].get(previous, 0.0), duration)
                state.update(stage=phase, stage_mono=mono)

    def _run(self):
        while not self.stopping.is_set():
            try:
                packet = self.socket.recv(256)
                self.accept(packet, time.monotonic())
            except socket.timeout:
                continue
            except OSError:
                break

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(self.states)
            for state in self.states.values():
                state['max_phase_sec'] = {}
        return result

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=0.3)
        self.socket.close()
        Path(self.address).unlink(missing_ok=True)


class SystemSampler:
    def __init__(self):
        self.previous_cpu = None
        self.previous_mono = None
        self.previous_self_cpu = None
        self.thermal = list(Path('/sys/class/thermal').glob('thermal_zone*/temp'))
        self.gpu = [p for base in Path('/sys/class/devfreq').glob('*gpu*')
                    for p in (base / 'device/load', base / 'cur_freq') if p.exists()]
        self.cpu_freq = list(Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_cur_freq'))

    def sample(self, pids):
        now, own_cpu = time.monotonic(), time.process_time()
        result = {'loadavg': read_text('/proc/loadavg'), 'processes': {}}
        stat = read_text('/proc/stat')
        if stat:
            ticks = [int(x) for x in stat.splitlines()[0].split()[1:9]]
            total, idle = sum(ticks), ticks[3] + ticks[4]
            if self.previous_cpu:
                dt, di = total - self.previous_cpu[0], idle - self.previous_cpu[1]
                result['cpu_busy_pct'] = 100 * (dt - di) / dt if dt else 0
            self.previous_cpu = (total, idle)
        if self.previous_mono is not None:
            result['recorder_cpu_one_core_pct'] = 100 * (own_cpu - self.previous_self_cpu) / (now - self.previous_mono)
        self.previous_mono, self.previous_self_cpu = now, own_cpu
        mem = read_text('/proc/meminfo') or ''
        result['memory_kb'] = {line.split(':')[0]: int(line.split()[1])
                               for line in mem.splitlines()
                               if line.split(':')[0] in ('MemAvailable', 'MemFree', 'SwapFree', 'SwapTotal', 'Dirty', 'Writeback')}
        vm = read_text('/proc/vmstat') or ''
        result['vm'] = {line.split()[0]: int(line.split()[1]) for line in vm.splitlines()
                        if line.split()[0] in ('pgmajfault', 'pswpin', 'pswpout', 'oom_kill')}
        result['pressure'] = {name: read_text('/proc/pressure/' + name) for name in ('cpu', 'memory', 'io')}
        result['temperature_mC'] = {str(p.parent.name): read_text(p) for p in self.thermal}
        result['cpu_frequency_khz'] = {p.parent.name: read_text(p) for p in self.cpu_freq}
        result['gpu_load'] = {str(p): read_text(p) for p in self.gpu}
        for pid in set(pids) | {os.getpid()}:
            result['processes'][str(pid)] = {
                'stat': read_text(f'/proc/{pid}/stat', 2048),
                'wchan': read_text(f'/proc/{pid}/wchan', 256),
            }
        return result


class JournalFollower:
    def __init__(self, emit):
        self.emit = emit
        self.process = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self.process = subprocess.Popen([
                'journalctl', '-b', '-f', '-n', '30', '-o', 'json', '--no-pager',
                '_TRANSPORT=kernel', '+', '_COMM=Xorg', '+', '_COMM=gdm-x-session',
                '+', '_COMM=systemd',
            ], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            budget_second, count = 0, 0
            for line in self.process.stdout:
                if self.stopping.is_set():
                    break
                try:
                    entry = json.loads(line)
                except ValueError:
                    continue
                message = entry.get('MESSAGE', '')
                if not isinstance(message, str) or not re.search(
                    r'usb|uvc|xusb|nvmap|NVRM|oom|out of memory|thermal|throttl|libinput|device|Starting|Started|Finished|failed',
                    message, re.IGNORECASE,
                ):
                    continue
                second = int(time.monotonic())
                if second != budget_second:
                    budget_second, count = second, 0
                count += 1
                if count <= 30:
                    self.emit({'type': 'system_event', 'journal_mono_us': entry.get('__MONOTONIC_TIMESTAMP'),
                               'journal_wall_us': entry.get('__REALTIME_TIMESTAMP'),
                               'boot_id': entry.get('_BOOT_ID'), 'process': entry.get('_COMM'),
                               'message': message[:2048]})
                elif count == 31:
                    self.emit({'type': 'system_events_rate_limited'})
            if not self.stopping.is_set():
                self.emit({'type': 'journal_unavailable', 'returncode': self.process.wait()})
        except OSError as exc:
            self.emit({'type': 'journal_unavailable', 'error': str(exc)})

    def close(self):
        self.stopping.set()
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.thread.join(timeout=0.5)


def classify_stalls(traces, now, realsense_active, threshold=0.5):
    """Evidence labels, not an automatic root-cause diagnosis."""
    issues = []
    for source, state in traces.items():
        complete = state['phases'].get('complete')
        if source == 'realsense' and not realsense_active:
            continue
        # Initial engine load/warmup is recorded, not called a runtime stall.
        if complete is None:
            continue
        if now - complete['mono'] >= threshold:
            issues.append(f'{source}:completion_stale:{state.get("stage", "unknown")}')
        # Catch a stall that recovered between two recorder ticks as well.
        for phase, duration in state.get('max_phase_sec', {}).items():
            if phase in ('callback', 'predict', 'predicted', 'publish', 'preview') and duration >= threshold:
                issues.append(f'{source}:slow_phase:{phase}')
    return issues


def make_node(args, receiver, writer):
    import rclpy
    from rclpy.callback_groups import CallbackGroup
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import Bool, String
    from msgs.msg import MotionCommand, MotionEnd
    from rcl_interfaces.msg import Log

    class SampledGroup(CallbackGroup):
        """The timer takes depth-one metadata; no per-frame Python dispatch."""

        def can_execute(self, entity):
            return False

        def beginning_execution(self, entity):
            return False

        def ending_execution(self, entity):
            pass

    class Recorder(Node):
        def __init__(self):
            super().__init__('vision_diagnostics', enable_rosout=False)
            self.info = {}
            self.health = {}
            self.control_state = {'ball_active': True, 'hoop_active': False}
            self.realsense_enabled_at = 0.0
            self.history = deque(maxlen=150)  # 30 seconds at 5 Hz.
            self.last_issues = []
            self.last_dump = -float('inf')
            self.post_until = 0
            self.last_save = 0
            self.last_tick = time.monotonic()
            self.last_system = 0
            self.system = {}
            self.sampler = SystemSampler()
            self.warn_second, self.warn_count = 0, 0
            sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions_kept = []
            self.sampled_group = SampledGroup()
            self.sampled_subscriptions = []
            info_topics = {
                'webcam': '/camera/camera_info',
                'rs_color': '/camera/color/camera_info',
                'rs_depth': '/camera/depth/camera_info',
                'rs_aligned_depth': '/camera/aligned_depth_to_color/camera_info',
            }
            for name, topic in info_topics.items():
                self.sampled_subscriptions.append(self.create_subscription(
                    CameraInfo, topic, lambda msg, name=name: self.on_info(name, msg), sensor,
                    callback_group=self.sampled_group))
            for name in SOURCES[:2]:
                self.sampled_subscriptions.append(self.create_subscription(
                    String, f'/vision/{name}_frame_health',
                    lambda msg, name=name: self.on_health(name, msg), sensor,
                    callback_group=self.sampled_group))
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
            self.timer = self.create_timer(0.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_info(self, name, msg):
            old = self.info.get(name, {})
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.info[name] = {'mono': time.monotonic(), 'stamp_ns': stamp,
                               'sample_count': old.get('sample_count', 0) + 1,
                               'repeated_stamps': old.get('repeated_stamps', 0) + int(stamp == old.get('stamp_ns'))}

        def on_health(self, name, msg):
            try:
                self.health[name] = {'arrival_mono': time.monotonic(), 'report': json.loads(msg.data)}
            except (TypeError, ValueError):
                pass

        def on_context(self, name, value):
            if self.control_state.get(name) != value or name == 'motion_command':
                was_active = self.control_state['ball_active'] or self.control_state['hoop_active']
                self.control_state[name] = value
                if not was_active and (self.control_state['ball_active'] or self.control_state['hoop_active']):
                    self.realsense_enabled_at = time.monotonic()
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

        def tick(self):
            now = time.monotonic()
            # Same nonblocking take used by Humble's executor, restricted to
            # one latest message per subscription per tick. DDS keeps depth=1;
            # skipped metadata is intentional, trace counters remain per-frame.
            for subscription in self.sampled_subscriptions:
                with subscription.handle:
                    taken = subscription.handle.take_message(subscription.msg_type, subscription.raw)
                if taken is not None:
                    subscription.callback(taken[0])
            traces = receiver.snapshot()
            if now - self.last_system >= 1:
                self.system = self.sampler.sample([s['pid'] for s in traces.values()])
                self.last_system = now
            rs_expected = ((self.control_state['ball_active'] or self.control_state['hoop_active'])
                           and now - self.realsense_enabled_at >= 0.5)
            issues = classify_stalls(traces, now, rs_expected)
            for name, state in self.info.items():
                if now - state['mono'] >= 0.5:
                    issues.append(name + ':camera_info_stale')
            if now - self.last_tick >= 0.5:
                issues.append('recorder:scheduling_gap')
            record = {'type': 'sample', 'mono': now, 'wall_ns': time.time_ns(),
                      'tick_gap_sec': now - self.last_tick, 'traces': traces,
                      'camera_info': copy.deepcopy(self.info), 'health': copy.deepcopy(self.health),
                      'context': copy.deepcopy(self.control_state), 'system': self.system,
                      'issues': issues, 'disk_queue_dropped': writer.dropped, 'disk_error': writer.error}
            self.last_tick = now
            if issues != self.last_issues:
                self.emit({'type': 'incident' if issues else 'resumed', 'issues': issues})
                if issues and now - self.last_dump >= 30:
                    for previous in self.history:
                        writer.put({**previous, 'type': 'pre_incident'})
                    self.last_dump = now
                self.post_until = now + 15
                self.last_issues = issues
            self.history.append(record)
            if now < self.post_until or now - self.last_save >= 1:
                writer.put(record)
                self.last_save = now

    return Recorder()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', required=True)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--settings', default='')
    parser.add_argument('--no-journal', action='store_true')
    args = parser.parse_args()
    # Never compete for CPU priority with perception or motion.
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
    receiver = TraceReceiver(args.socket)
    journal = None
    node = None
    import rclpy
    try:
        rclpy.init()
        node = make_node(args, receiver, writer)
        node.emit({'type': 'session', 'boot_id': read_text('/proc/sys/kernel/random/boot_id'),
                   'uptime': read_text('/proc/uptime'), 'pid': os.getpid(),
                   'sampling_hz': 5, 'normal_disk_hz': 1, 'pre_incident_sec': 30,
                   'note': 'CameraInfo is driver publication evidence, not an independent image capture.'})
        node.emit({'type': 'configuration', **startup_configuration(args.settings)})
        if not args.no_journal:
            journal = JournalFollower(node.emit)
        print(f'[VisionDiagnostics] {directory}', flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if journal is not None:
            journal.close()
        if node is not None:
            node.emit({'type': 'recorder_stop', 'disk_queue_dropped': writer.dropped, 'disk_error': writer.error})
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        receiver.close()
        writer.close()
        try:
            Path(args.socket).parent.rmdir()
        except OSError:
            pass


if __name__ == '__main__':
    main()
