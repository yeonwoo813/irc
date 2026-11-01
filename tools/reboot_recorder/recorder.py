#!/usr/bin/env python3
"""Jetson reboot reports. Installed service is event-only; monitor is opt-in."""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import selectors
import shutil
import signal
import subprocess
import time
import uuid


DEFAULT_STATE = Path('/var/lib/jetson-reboot-recorder')
DEFAULT_EVENTS = Path('/var/log/jetson-reboots')
PMC = Path('/sys/devices/platform/bus@0/c360000.pmc')
SYSTEM_LOGS = ('syslog', 'syslog.1', 'kern.log', 'kern.log.1', 'auth.log', 'auth.log.1')
JOURNAL_FIELDS = ('__REALTIME_TIMESTAMP', '__MONOTONIC_TIMESTAMP', '_BOOT_ID',
                  'MESSAGE', 'PRIORITY', '_TRANSPORT', '_PID', '_COMM',
                  '_SYSTEMD_UNIT', 'SYSLOG_IDENTIFIER')


def now():
    return datetime.now(timezone.utc).isoformat()


def read(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, TypeError, UnicodeError):
        return None


def boot_id():
    return str(uuid.UUID(read('/proc/sys/kernel/random/boot_id')))


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix('.tmp')
    with temp.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
    sync_dir(path.parent)


def load_state(path):
    try:
        state = json.loads(path.read_text())
        state['boot_id'] = str(uuid.UUID(state['boot_id']))
        return state
    except FileNotFoundError:
        return None
    except (ValueError, KeyError, TypeError) as error:
        # Never silently overwrite the only evidence of a damaged state file.
        raise RuntimeError('Invalid recorder state; preserve it for inspection') from error


def classify(previous, current_id, reset_reason):
    if not previous or previous['boot_id'] == current_id:
        return None
    action = previous.get('shutdown_request', {}).get('action')
    if action in ('reboot', 'kexec'):
        return 'requested_reboot'
    if 'WDT' in (reset_reason or '').upper():
        return 'watchdog_reset_reported'
    if action in ('poweroff', 'halt'):
        return None
    return 'unclean_restart'


class RollingLog:
    def __init__(self, path, limit=2 * 1024 * 1024, backups=2):
        self.path, self.limit, self.backups = Path(path), limit, backups
        self.stream = self.path.open('ab', buffering=0)
        self.size = self.path.stat().st_size

    def append(self, value):
        line = (json.dumps(value, ensure_ascii=False) + '\n').encode()
        if len(line) > self.limit:
            line = (json.dumps({'truncated_record': line[:self.limit // 4].decode(
                errors='replace')}) + '\n').encode()
        if self.size and self.size + len(line) > self.limit:
            self.close()
            for index in range(self.backups, 0, -1):
                source = self.path if index == 1 else Path(str(self.path) + f'.{index - 1}')
                if source.exists():
                    os.replace(source, str(self.path) + f'.{index}')
            self.stream = self.path.open('ab', buffering=0)
            self.size = 0
            sync_dir(self.path.parent)
        self.stream.write(line)
        self.size += len(line)

    def sync(self):
        os.fsync(self.stream.fileno())

    def close(self):
        if not self.stream.closed:
            self.sync()
            self.stream.close()


def sensor_paths():
    paths = []
    for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
        paths.append((f'thermal_mC/{read(zone / "type") or zone.name}', zone / 'temp'))
    for device in Path('/sys/class/hwmon').glob('hwmon*'):
        name = read(device / 'name') or device.name
        for path in device.iterdir():
            if re.fullmatch(r'(?:in\d+|curr\d+|power\d+|fan\d+)_input|oc\d+_event_cnt', path.name):
                label = read(device / (path.name.split('_')[0] + '_label'))
                key = f'{device.name}/{name}/{label or path.name}/{path.name}'
                paths.append((key, path))
    return paths


def sample(paths):
    memory = {}
    for line in (read('/proc/meminfo') or '').splitlines():
        key, value = line.split(':', 1)
        if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty', 'Writeback'):
            memory[key] = value.strip()
    return {
        'wall_time_utc': now(), 'uptime_seconds': (read('/proc/uptime') or '').split(' ')[0],
        'memory': memory, 'loadavg': read('/proc/loadavg'),
        'pressure': {key: read('/proc/pressure/' + key) for key in ('cpu', 'memory', 'io')},
        'sensors': {key: value for key, path in paths if (value := read(path)) is not None},
    }


def sync_tree(path):
    for item in path.rglob('*'):
        if item.is_file():
            with item.open('rb') as stream:
                os.fsync(stream.fileno())
    for item in sorted((p for p in path.rglob('*') if p.is_dir()), reverse=True):
        sync_dir(item)
    sync_dir(path)


def copy_pstore(destination):
    result = {}
    budget = 8 * 1024 * 1024
    for name, source in [('pstore', Path('/sys/fs/pstore')),
                         ('pstore_archive', Path('/var/lib/systemd/pstore'))]:
        target = destination / name
        try:
            files = [p for p in source.rglob('*') if p.is_file() and not p.is_symlink()]
            copied = 0
            for path in files[:32]:
                if budget <= 0:
                    break
                output = target / path.relative_to(source)
                output.parent.mkdir(parents=True, exist_ok=True)
                with path.open('rb') as stream:
                    data = stream.read(min(budget, 2 * 1024 * 1024))
                output.write_bytes(data)
                budget -= len(data)
                copied += 1
            result[name] = {'files_found': len(files), 'files_copied': copied,
                            'bytes_per_file_limit': 2 * 1024 * 1024}
        except OSError as error:
            result[name] = {'error': str(error)}
    return result


def copy_system_logs(destination, source_dir=Path('/var/log')):
    """Read bounded tails only when an incident is detected, never poll the logs."""
    result = {}
    target = destination / 'system_logs'
    target.mkdir(exist_ok=True)
    for name in SYSTEM_LOGS:
        source = source_dir / name
        limit = 256 * 1024 if name.startswith('auth.log') else 1024 * 1024
        try:
            with source.open('rb') as stream:
                info = os.fstat(stream.fileno())
                start = max(0, info.st_size - limit)
                stream.seek(start)
                data = stream.read(limit)
            # Ignore a partial first line when taking a tail.
            if start and b'\n' in data:
                data = data.split(b'\n', 1)[1]
            (target / name).write_bytes(data)
            result[name] = {'source_bytes': info.st_size, 'copied_bytes': len(data),
                            'source_mtime_ns': info.st_mtime_ns, 'tail_only': start > 0}
        except OSError as error:
            result[name] = {'error': str(error)}
    return result


def evidence_summary(buffer):
    indicators = []
    pattern = re.compile(r'Kernel panic|\bOops:|BUG:|soft lockup|hard LOCKUP|'
                         r'watchdog.*(?:timeout|expired)|Out of memory:|oom-kill|'
                         r'Killed process |thermal.*(?:critical|shutdown)|'
                         r'under.?voltage|over.?current|NVRM.*(?:Failed|ERROR)|I/O error', re.I)
    for path in [buffer / 'journal.jsonl.2', buffer / 'journal.jsonl.1', buffer / 'journal.jsonl']:
        if not path.exists():
            continue
        for line in path.read_text(errors='replace').splitlines():
            try:
                item = json.loads(line)
            except ValueError:
                continue
            message = item.get('MESSAGE', '')
            if isinstance(message, str) and pattern.search(message):
                indicators.append(item)
    last_sample = None
    for path in [buffer / 'telemetry.jsonl.1', buffer / 'telemetry.jsonl']:
        if not path.exists():
            continue
        for line in path.read_text(errors='replace').splitlines():
            try:
                item = json.loads(line)
                if 'sensors' in item:
                    last_sample = item
            except ValueError:
                continue
    return {'last_sensor_sample': last_sample, 'matched_messages': indicators[-20:],
            'note': 'These are observed messages, not an automatic root-cause diagnosis.'}


def update_latest(events_root, event):
    latest = events_root / '.latest.tmp'
    latest.unlink(missing_ok=True)
    latest.symlink_to(event.name)
    os.replace(latest, events_root / 'latest')
    sync_dir(events_root)


def archive_event(state_root, events_root, previous, current_id, reason, level, kind):
    event = events_root / (previous['boot_id'] + '--' + current_id)
    if (event / 'summary.json').exists():
        update_latest(events_root, event)
        return event  # A recorder restart must not duplicate an incident.
    event.mkdir(parents=True, exist_ok=True)
    buffer = state_root / 'buffers' / previous['boot_id']
    if buffer.exists():
        for source in buffer.iterdir():
            if source.is_file() and not source.is_symlink():
                shutil.copyfile(source, event / source.name)
    summary = {
        'event': kind, 'detected_at_utc': now(), 'previous': previous,
        'current_boot_id': current_id, 'reset_reason': reason, 'reset_level': level,
        'pstore_capture': copy_pstore(event),
        'system_log_capture': copy_system_logs(event),
        'evidence': evidence_summary(event),
        'limits': [
            'Unclean restart includes manual power removal; it does not prove an automatic reboot.',
            'A shutdown marker records a request, not proof that the shutdown completed.',
            'SYS_RESET_N does not identify the underlying fault.',
            'Local logs may lose the last samples on sudden power loss or a hard lockup.',
            'Event-only mode does not sample sensors or continuously save the journal.',
            'Optional monitor mode samples every 2 seconds; brief voltage dips may be missed.',
            'System log tails may include this boot or older boots; use timestamps and boot boundaries.',
            'pstore_archive may contain older incidents. Sensor in*_input is mV, curr*_input mA.',
            'Wall clock may jump after boot; use boot IDs and monotonic/uptime fields for ordering.',
        ],
    }
    # Commit the summary last; a partial archive can be retried after interruption.
    sync_tree(event)
    atomic_json(event / 'summary.json', summary)
    sync_dir(events_root)
    update_latest(events_root, event)
    return event


def initialize(state_root, events_root, current_id, reason, level, max_events=10,
               capture_mode='continuous'):
    state_root.mkdir(parents=True, exist_ok=True)
    events_root.mkdir(parents=True, exist_ok=True)
    state_path = state_root / 'state.json'
    previous = load_state(state_path)
    if previous and previous['boot_id'] == current_id:
        if previous.get('capture_mode') != capture_mode:
            previous['capture_mode'] = capture_mode
            previous['mode_changed_at_utc'] = now()
            atomic_json(state_path, previous)
        buffer = state_root / 'buffers' / current_id
        buffer.mkdir(parents=True, exist_ok=True)
        return buffer, None
    kind = classify(previous, current_id, reason)
    event = archive_event(state_root, events_root, previous, current_id, reason, level, kind) if kind else None
    buffer = state_root / 'buffers' / current_id
    buffer.mkdir(parents=True, exist_ok=True)
    atomic_json(state_path, {'boot_id': current_id, 'monitoring_started_at_utc': now(),
                             'started_uptime': read('/proc/uptime'), 'shutdown_request': {},
                             'capture_mode': capture_mode})
    # Only remove previous buffers after their event and the new state are durable.
    for old in buffer.parent.iterdir():
        if old != buffer and old.is_dir() and not old.is_symlink():
            shutil.rmtree(old)
    events = sorted((p for p in events_root.iterdir() if p.is_dir() and not p.is_symlink()),
                    key=lambda p: p.stat().st_mtime_ns, reverse=True)
    for old in events[max_events:]:
        if old != event:
            shutil.rmtree(old)
    return buffer, event


def mark(state_root, action, current_id):
    path = state_root / 'state.json'
    state = load_state(path)
    if state and state['boot_id'] == current_id:
        state['shutdown_request'] = {'action': action, 'wall_time_utc': now(),
                                     'uptime': read('/proc/uptime')}
        atomic_json(path, state)


def check(args):
    current_id = boot_id()
    with (args.state_dir / 'state.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        _buffer, event = initialize(args.state_dir, args.events_dir, current_id,
                                    read(PMC / 'reset_reason'), read(PMC / 'reset_level'),
                                    capture_mode='event_only')
    print(json.dumps({'boot_id': current_id, 'capture_mode': 'event_only',
                      'new_event': str(event) if event else None}), flush=True)


def journal_command():
    # Capture kernel messages, PID 1 and explicit shutdown/authentication activity.
    return ['journalctl', '--boot=0', '--follow', '--lines=150', '--no-pager',
            '--output=json', '--output-fields=' + ','.join(JOURNAL_FIELDS),
            '_TRANSPORT=kernel', '+', '_PID=1', '+', 'SYSLOG_IDENTIFIER=sudo', '+',
            '_SYSTEMD_UNIT=systemd-logind.service', '+', '_SYSTEMD_UNIT=systemd-oomd.service',
            '+', '_SYSTEMD_UNIT=nvfancontrol.service', '+', '_SYSTEMD_UNIT=nvargus-daemon.service']


def monitor(args):
    current_id = boot_id()
    with (args.state_dir / 'state.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        buffer, event = initialize(args.state_dir, args.events_dir, current_id,
                                   read(PMC / 'reset_reason'), read(PMC / 'reset_level'))
    print(json.dumps({'monitoring_boot_id': current_id,
                      'new_event': str(event) if event else None}), flush=True)
    journal = RollingLog(buffer / 'journal.jsonl')
    telemetry = RollingLog(buffer / 'telemetry.jsonl', limit=1024 * 1024, backups=1)
    paths = sensor_paths()
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True  # Service stop alone must never imply clean poweroff.

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    selector = selectors.DefaultSelector()
    process = None
    pending = b''
    next_sample, next_journal = 0.0, 0.0
    started = time.monotonic()
    try:
        while not stopping and (not args.duration or time.monotonic() - started < args.duration):
            tick = time.monotonic()
            if process is None and tick >= next_journal:
                process = subprocess.Popen(journal_command(), stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL)
                selector.register(process.stdout, selectors.EVENT_READ)
            for key, _mask in selector.select(timeout=0.25):
                data = os.read(key.fileobj.fileno(), 65536)
                if not data:
                    selector.unregister(key.fileobj)
                    key.fileobj.close()
                    code = process.wait(timeout=2)
                    telemetry.append({'wall_time_utc': now(), 'journal_reader_exited': code})
                    process = None
                    next_journal = time.monotonic() + 5
                    pending = b''
                    continue
                pending += data
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    try:
                        item = json.loads(line)
                        if isinstance(item.get('MESSAGE'), str):
                            item['MESSAGE'] = item['MESSAGE'][:16384]
                        journal.append(item)
                    except (ValueError, AttributeError):
                        journal.append({'unparsed': line[:16384].decode(errors='replace')})
                if len(pending) > 1024 * 1024:
                    journal.append({'reader_warning': 'oversized unfinished journal record dropped'})
                    pending = b''
            if tick >= next_sample:
                telemetry.append(sample(paths))
                journal.sync()
                telemetry.sync()
                next_sample = tick + 2
    finally:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            process.stdout.close()
        selector.close()
        telemetry.append({'wall_time_utc': now(), 'recorder_stopped': True,
                          'note': 'Stopping the recorder does not mark a clean shutdown.'})
        journal.close()
        telemetry.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state-dir', type=Path, default=DEFAULT_STATE)
    parser.add_argument('--events-dir', type=Path, default=DEFAULT_EVENTS)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('check', help='Check the boot transition once, then exit (installed default)')
    run = sub.add_parser('monitor')
    run.add_argument('--duration', type=float, default=0, help='Stop after N seconds (smoke test only)')
    marker = sub.add_parser('mark')
    marker.add_argument('action', choices=['reboot', 'poweroff', 'halt', 'kexec'])
    args = parser.parse_args()
    os.umask(0o027)
    args.state_dir.mkdir(parents=True, exist_ok=True)
    # Serialize startup against shutdown markers; keep monitoring's lock separately.
    if args.command == 'mark':
        with (args.state_dir / 'state.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            mark(args.state_dir, args.action, boot_id())
    elif args.command == 'check':
        check(args)
    else:
        with (args.state_dir / 'monitor.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            monitor(args)


if __name__ == '__main__':
    main()
