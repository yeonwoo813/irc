"""Give the single /camera driver exclusive ownership for its whole lifetime."""

import argparse
import ctypes
import fcntl
import os
from pathlib import Path
import signal
import stat
import sys


READY_MARKER = '[realsense_guard] Ownership acquired'


def find_camera_owners(proc_root=Path('/proc'), sys_root=Path('/sys')):
    """Find old drivers and processes holding RealSense video/USB devices."""
    devices = set()
    for video in (sys_root / 'class/video4linux').glob('video*'):
        try:
            if 'realsense' in (video / 'name').read_text().lower():
                devices.add('/dev/' + video.name)
        except OSError:
            continue
    for usb in (sys_root / 'bus/usb/devices').glob('*'):
        try:
            if 'realsense' in (usb / 'product').read_text().lower():
                bus = int((usb / 'busnum').read_text())
                device = int((usb / 'devnum').read_text())
                devices.add(f'/dev/bus/usb/{bus:03d}/{device:03d}')
        except (OSError, ValueError):
            continue

    owners = []
    for process in proc_root.iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            executable = Path(os.readlink(process / 'exe')).name
            # Also catch an orphaned driver waiting for a disconnected device.
            occupied = executable == 'realsense2_camera_node'
            if not occupied and devices:
                for fd in (process / 'fd').iterdir():
                    try:
                        target = os.readlink(fd).removesuffix(' (deleted)')
                    except OSError:
                        continue
                    if target in devices:
                        occupied = True
                        break
            if occupied:
                owners.append(f'PID {process.name} ({executable})')
        except (OSError, ProcessLookupError):
            continue
    return owners


def acquire_lock(path):
    """Acquire immediately; keep the file in place to avoid inode races."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid():
            raise RuntimeError(f'Invalid camera lock file: {path}')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = os.read(fd, 80).decode(errors='replace').strip() or 'starting'
            raise RuntimeError(
                f'RealSense is already running (PID {owner}). '
                'Stop that launch before starting another one.'
            ) from None
        os.ftruncate(fd, 0)
        os.write(fd, str(os.getpid()).encode())
        return fd
    except BaseException:
        os.close(fd)
        raise


def arm_parent_death_signal(parent_pid):
    """Release camera ownership even if launch itself crashes or is killed."""
    libc = ctypes.CDLL(None, use_errno=True)
    # Normal shutdown uses launch's bounded SIGINT -> SIGTERM -> SIGKILL.
    # A vanished launch cannot supervise that sequence; do not leave an orphan.
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        errno = ctypes.get_errno()
        raise OSError(errno, os.strerror(errno))
    if parent_pid <= 1 or os.getppid() != parent_pid:
        raise RuntimeError('Parent launch exited before camera startup.')


def main(argv=None):
    """Acquire ownership and replace this process with the actual driver."""
    parent_pid = os.getppid()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--lock-file', default=f'/tmp/irc-realsense-{os.getuid()}.lock'
    )
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('driver command required after --')

    fd = None
    try:
        arm_parent_death_signal(parent_pid)
        fd = acquire_lock(args.lock_file)
        owners = find_camera_owners()
        if owners:
            raise RuntimeError(
                'RealSense is already in use: ' + ', '.join(owners)
                + '. Stop the owning program before restarting.'
            )
        # exec keeps the same PID and signal handling relationship with launch.
        # Only this driver inherits the descriptor; OS exit releases the lock.
        os.set_inheritable(fd, True)
        print(f'{READY_MARKER}; PID {os.getpid()}', flush=True)
        os.execvp(command[0], command)
    except (OSError, RuntimeError) as error:
        print(f'[realsense_guard] ERROR: {error}', file=sys.stderr, flush=True)
        return 1
    finally:
        if fd is not None:
            os.close(fd)


if __name__ == '__main__':
    sys.exit(main())
