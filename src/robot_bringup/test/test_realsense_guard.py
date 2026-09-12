"""Exercise real process locks, exec inheritance and orphan cleanup."""

import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import pytest

from robot_bringup import realsense_guard as guard


# These process tests do not depend on a connected camera. Device discovery is
# covered separately against a fake proc/sys tree, while flock/exec are real.
RUNNER = (
    'from robot_bringup import realsense_guard as g; '
    'g.find_camera_owners = lambda: []; '
    'raise SystemExit(g.main())'
)
DRIVER = 'import time; print("driver-started", flush=True); time.sleep(30)'


def command(lock_file):
    return [
        sys.executable, '-u', '-c', RUNNER, '--lock-file', str(lock_file),
        '--', sys.executable, '-u', '-c', DRIVER,
    ]


def wait_output(process, token, timeout=5):
    deadline = time.monotonic() + timeout
    output = b''
    while time.monotonic() < deadline:
        readable, _, _ = select.select([process.stdout], [], [], 0.1)
        if readable:
            block = os.read(process.stdout.fileno(), 4096)
            if not block:
                break
            output += block
            if token in output:
                return output
    raise AssertionError(f'Missing {token!r}: {output!r}')


def stop(process):
    if process.poll() is None:
        process.kill()
    process.communicate(timeout=5)


@pytest.mark.parametrize('exit_signal', [signal.SIGTERM, signal.SIGKILL])
def test_exec_keeps_lock_and_exit_releases_it(tmp_path, exit_signal):
    lock = tmp_path / 'camera.lock'
    first = subprocess.Popen(command(lock), stdout=subprocess.PIPE)
    try:
        wait_output(first, b'driver-started')
        duplicate = subprocess.run(
            command(lock), capture_output=True, timeout=5,
        )
        assert duplicate.returncode == 1
        assert f'PID {first.pid}'.encode() in duplicate.stderr
        assert b'driver-started' not in duplicate.stdout
        os.kill(first.pid, exit_signal)
        first.wait(timeout=5)
        # The PID text is stale, but an existing file must never block restart.
        assert lock.exists()
        restarted = subprocess.Popen(command(lock), stdout=subprocess.PIPE)
        try:
            wait_output(restarted, b'driver-started')
        finally:
            stop(restarted)
    finally:
        stop(first)


def test_simultaneous_start_has_one_winner(tmp_path):
    lock = tmp_path / 'camera.lock'
    processes = [
        subprocess.Popen(command(lock), stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
        for _ in range(2)
    ]
    try:
        deadline = time.monotonic() + 5
        while all(p.poll() is None for p in processes):
            assert time.monotonic() < deadline
            time.sleep(0.01)
        losers = [p for p in processes if p.poll() is not None]
        winners = [p for p in processes if p.poll() is None]
        assert len(losers) == len(winners) == 1
        assert losers[0].returncode == 1
        wait_output(winners[0], b'driver-started')
    finally:
        for process in processes:
            stop(process)


def test_parent_crash_kills_driver_and_releases_lock(tmp_path):
    lock = tmp_path / 'camera.lock'
    parent_code = (
        'import subprocess, sys, time; '
        'p = subprocess.Popen(sys.argv[1:]); '
        'print("child-pid=" + str(p.pid), flush=True); time.sleep(30)'
    )
    parent = subprocess.Popen(
        [sys.executable, '-u', '-c', parent_code, *command(lock)],
        stdout=subprocess.PIPE,
    )
    child_pid = None
    try:
        output = wait_output(parent, b'driver-started')
        child_pid = int(output.split(b'child-pid=')[1].splitlines()[0])
        parent.kill()
        parent.wait(timeout=5)
        deadline = time.monotonic() + 5
        while True:
            try:
                fd = guard.acquire_lock(lock)
                os.close(fd)
                break
            except RuntimeError:
                assert time.monotonic() < deadline
                time.sleep(0.01)
        status = Path(f'/proc/{child_pid}/stat')
        assert not status.exists() or status.read_text().split()[2] == 'Z'
    finally:
        stop(parent)
        if child_pid is not None:
            try:
                os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


@pytest.mark.parametrize('target', ['/dev/video4', '/dev/bus/usb/002/077'])
def test_external_camera_device_owner_is_reported(tmp_path, target):
    proc = tmp_path / 'proc'
    sys_root = tmp_path / 'sys'
    process = proc / '12345'
    (process / 'fd').mkdir(parents=True)
    (process / 'exe').symlink_to('/usr/bin/realsense-viewer')
    (process / 'fd/9').symlink_to(target)
    video = sys_root / 'class/video4linux/video4'
    video.mkdir(parents=True)
    (video / 'name').write_text('Intel RealSense Depth Camera')
    usb = sys_root / 'bus/usb/devices/2-1.4'
    usb.mkdir(parents=True)
    for name, value in [('product', 'RealSense D435'), ('busnum', '2'),
                        ('devnum', '77')]:
        (usb / name).write_text(value)
    assert guard.find_camera_owners(proc, sys_root) == [
        'PID 12345 (realsense-viewer)'
    ]


def test_old_driver_is_detected_even_when_usb_is_disconnected(tmp_path):
    process = tmp_path / 'proc/12345'
    process.mkdir(parents=True)
    (process / 'exe').symlink_to('/opt/ros/realsense2_camera_node')
    assert guard.find_camera_owners(tmp_path / 'proc', tmp_path / 'sys') == [
        'PID 12345 (realsense2_camera_node)'
    ]
