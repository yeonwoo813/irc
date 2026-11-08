"""Record one termination signal's provenance without affecting other nodes.

Run this process in launch's process group. It never reads keyboard input,
forwards signals, or installs handlers in another process. A group-wide signal
reaches it independently; a signal addressed only to another PID does not.
Linux SI_KERNEL does not establish that a person pressed a physical key.
"""

import signal


WATCHED_SIGNALS = {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}
# Linux UAPI values; Python's signal module does not expose these constants.
LINUX_SI_USER = 0
LINUX_SI_QUEUE = -1
LINUX_SI_TKILL = -6
LINUX_SI_KERNEL = 128


def classify_signal(code):
    """Describe siginfo evidence without inferring a physical input source."""
    if code == LINUX_SI_KERNEL:
        return {
            'origin': 'kernel',
            'detail': 'Compatible with a terminal-generated signal; '
                      'does not identify physical keys or the input source.',
        }
    if code in (LINUX_SI_USER, LINUX_SI_QUEUE, LINUX_SI_TKILL):
        return {
            'origin': 'userspace',
            'detail': 'Signal reports a sending process; sender metadata '
                      'is a best-effort snapshot after delivery.',
        }
    return {
        'origin': 'unknown',
        'detail': 'Unclassified Linux siginfo code; preserve the raw values.',
    }


def process_snapshot(pid):
    """Read limited process identity; tolerate an already-exited sender."""
    import os
    from pathlib import Path

    result = {'pid': pid}
    if pid <= 0:
        return result
    errors = {}
    proc = Path('/proc') / str(pid)
    for name, read in (
        ('comm', lambda: (proc / 'comm').read_text(errors='replace')[:256]
         .rstrip('\n')),
        ('exe', lambda: os.readlink(proc / 'exe')[:4096]),
        ('pgid', lambda: os.getpgid(pid)),
        ('sid', lambda: os.getsid(pid)),
    ):
        try:
            result[name] = read()
        except OSError as error:
            result[name] = None
            errors[name] = error.errno
    if errors:
        result['lookup_errno'] = errors
    return result


def terminal_snapshot(launch_pid):
    """Inspect controlling-terminal metadata without reading terminal input."""
    import os
    from pathlib import Path

    result = {'tty_nr': None, 'path': None, 'foreground_pgid': None}
    try:
        fields = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
        result['tty_nr'] = int(fields[4])
        result['foreground_pgid'] = int(fields[5])
        if not result['tty_nr']:
            return result
        # launch generally has the real terminal on stdin; witness stdin may
        # be a pipe. Verify device identity rather than trusting an fd label.
        for pid in (launch_pid, os.getpid()):
            for fd in (0, 1, 2):
                try:
                    path = os.readlink(f'/proc/{pid}/fd/{fd}')
                    if (
                        path.startswith('/dev/')
                        and os.stat(path).st_rdev == result['tty_nr']
                    ):
                        result['path'] = path
                        return result
                except OSError:
                    continue
    except (OSError, ValueError, IndexError) as error:
        result['lookup_error'] = type(error).__name__
    return result


def write_record(fd, record):
    """Append and persist one bounded JSON record."""
    import json
    import os

    data = memoryview(
        (json.dumps(record, ensure_ascii=True, separators=(',', ':')) + '\n')
        .encode('ascii')
    )
    while data:
        count = os.write(fd, data)
        if not count:
            raise OSError('Signal witness write made no progress')
        data = data[count:]
    os.fsync(fd)


def main(argv=None):
    """Block signals before setup, record one exit event, and stop."""
    # This is deliberately the first operation: signals arriving during
    # argument parsing or file creation remain pending until sigwaitinfo.
    # The parent must arrange an inherited mask if coverage before Python
    # reaches this instruction is required.
    signal.pthread_sigmask(signal.SIG_BLOCK, WATCHED_SIGNALS)

    import os
    initial_parent_pid = os.getppid()

    import argparse
    from pathlib import Path
    import sys
    import time

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--launch-pid', type=int, default=initial_parent_pid)
    args = parser.parse_args(argv)
    if not sys.platform.startswith('linux'):
        parser.error('Linux siginfo semantics are required')
    if args.launch_pid <= 0:
        parser.error('--launch-pid must be a positive PID')

    fd = None
    try:
        directory = Path(args.directory).expanduser()
        directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        path = directory / f'signal-witness-{os.getpid()}-{time.time_ns()}.jsonl'
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.fchmod(fd, 0o600)
        startup = {
            'type': 'startup',
            'schema_version': 1,
            'wall_ns': time.time_ns(),
            'monotonic_ns': time.monotonic_ns(),
            'witness': process_snapshot(os.getpid()),
            'parent_pid': initial_parent_pid,
            'launch': process_snapshot(args.launch_pid),
            'terminal': terminal_snapshot(args.launch_pid),
            'watched_signals': sorted(int(item) for item in WATCHED_SIGNALS),
            'scope': 'Only signals delivered to this witness are observed. '
                     'A later launch cleanup signal may follow an earlier '
                     'signal addressed only to another process.',
        }
        write_record(fd, startup)
        print(f'[signal_witness] Ready: {path}', flush=True)
        while True:
            info = signal.sigtimedwait(WATCHED_SIGNALS, 1.0)
            if info is not None:
                break
            current_parent_pid = os.getppid()
            if current_parent_pid != initial_parent_pid:
                write_record(fd, {
                    'type': 'parent_exit',
                    'wall_ns': time.time_ns(),
                    'monotonic_ns': time.monotonic_ns(),
                    'previous_parent_pid': initial_parent_pid,
                    'current_parent_pid': current_parent_pid,
                    'witness_pid': os.getpid(),
                    'launch_pid': args.launch_pid,
                    'detail': 'Immediate parent changed; no terminating '
                              'signal was observed by this witness.',
                })
                return 0
        # Sample timestamps immediately after receiving, before proc lookups.
        event = {
            'type': 'signal',
            'wall_ns': time.time_ns(),
            'monotonic_ns': time.monotonic_ns(),
            'signal': info.si_signo,
            'signal_name': signal.Signals(info.si_signo).name,
            'si_code': info.si_code,
            'si_pid': info.si_pid,
            'si_uid': info.si_uid,
            'witness_pid': os.getpid(),
            'witness_pgid': os.getpgrp(),
            'launch_pid': args.launch_pid,
            'classification': classify_signal(info.si_code),
            'sender': (
                process_snapshot(info.si_pid)
                if info.si_code in (
                    LINUX_SI_USER, LINUX_SI_QUEUE, LINUX_SI_TKILL,
                ) and info.si_pid > 0
                else None
            ),
        }
        write_record(fd, event)
        return 0
    except OSError as error:
        print(f'[signal_witness] Recording failed: {error}', file=sys.stderr)
        return 1
    finally:
        if fd is not None:
            os.close(fd)


if __name__ == '__main__':
    raise SystemExit(main())
