"""Check quiet launch arguments and terminal input without robot hardware."""

import fcntl
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import termios

import pytest
from rclpy.utilities import remove_ros_args

from robot_bringup import quiet_process as quiet
from robot_bringup.realsense_guard import READY_MARKER


@pytest.fixture
def child_env():
    env = os.environ.copy()
    source = Path(__file__).resolve().parents[1]
    decision = source.parent / 'decision'
    env['PYTHONPATH'] = os.pathsep.join([
        str(source), str(decision), env.get('PYTHONPATH', ''),
    ])
    return env


def command(mode, script, *args):
    return [
        sys.executable, '-m', 'robot_bringup.quiet_process', mode, '--',
        sys.executable, str(script), *args,
    ]


@pytest.mark.parametrize('enabled', ['0', '1'])
def test_native_output_and_arguments_are_reversible(tmp_path, child_env, enabled):
    record = tmp_path / 'args.json'
    script = tmp_path / 'native.py'
    script.write_text(
        'import json, os, sys\n'
        f'open({str(record)!r}, "w").write(json.dumps(sys.argv[1:]))\n'
        'print("stdout log", flush=True)\n'
        'os.write(2, b"native stderr log\\n")\n'
    )
    child_env[quiet.QUIET_ENV] = enabled
    args = ['--ros-args', '--log-level', 'info', '-r', '__node:=test']
    result = subprocess.run(
        command('--ros', script, *args), env=child_env,
        capture_output=True, timeout=10, check=True,
    )
    actual = json.loads(record.read_text())
    if enabled == '1':
        assert result.stdout == result.stderr == b''
        assert actual == args + quiet.ROS_QUIET_ARGS
    else:
        assert b'stdout log' in result.stdout
        assert b'native stderr log' in result.stderr
        assert actual == args
    # Parse the real Humble ROS options without creating a node/DDS participant.
    assert remove_ros_args(args=['test'] + actual) == ['test']


def test_python_custom_parser_and_ros_initialization(tmp_path, child_env):
    record = tmp_path / 'init.json'
    # Observe what rclpy.init receives without starting ROS or a robot node.
    (tmp_path / 'rclpy.py').write_text(
        'import json\n'
        'def init(*, args=None, **kwargs):\n'
        f'    open({str(record)!r}, "w").write(json.dumps([args, kwargs]))\n'
    )
    script = tmp_path / 'custom_parser.py'
    script.write_text(
        'import argparse, logging, os, rclpy\n'
        'parser = argparse.ArgumentParser()\n'
        'parser.add_argument("--custom", required=True)\n'
        'assert parser.parse_args().custom == "unchanged"\n'
        'rclpy.init(args=[], domain_id=23)\n'
        'logging.critical("python log")\n'
        'print("stdout log", flush=True)\n'
        'os.write(2, b"native log\\n")\n'
    )
    child_env[quiet.QUIET_ENV] = '1'
    child_env['PYTHONPATH'] = str(tmp_path) + os.pathsep + child_env['PYTHONPATH']
    result = subprocess.run(
        command('--python', script, '--custom', 'unchanged'),
        env=child_env, capture_output=True, timeout=10, check=True,
    )
    assert result.stdout == result.stderr == b''
    assert json.loads(record.read_text()) == [quiet.ROS_QUIET_ARGS, {'domain_id': 23}]


def test_camera_ownership_marker_survives_before_quiet_child(tmp_path, child_env):
    record = tmp_path / 'driver-started'
    script = tmp_path / 'driver.py'
    script.write_text(
        'from pathlib import Path\n'
        f'Path({str(record)!r}).touch()\n'
        'print("driver output must be hidden", flush=True)\n'
    )
    runner = (
        'from robot_bringup import realsense_guard as g; '
        'g.find_camera_owners = lambda: []; '
        'raise SystemExit(g.main())'
    )
    child_env[quiet.QUIET_ENV] = '1'
    result = subprocess.run([
        sys.executable, '-c', runner,
        '--lock-file', str(tmp_path / 'camera.lock'), '--',
        *command('--ros', script),
    ], env=child_env, capture_output=True, timeout=10, check=True)
    # Launch needs this internal event to start the vision dependencies.
    assert READY_MARKER.encode() in result.stdout
    assert b'driver output must be hidden' not in result.stdout
    assert result.stderr == b''
    assert record.exists()


def test_enter_prompt_and_key_read_survive_quiet_output(tmp_path, child_env):
    record = tmp_path / 'enter-received'
    script = tmp_path / 'keyboard.py'
    script.write_text(
        'import select\n'
        'from pathlib import Path\n'
        'from types import SimpleNamespace, MethodType\n'
        'from decision.mission_start_keyboard import MissionStartKeyboard as K\n'
        'k = SimpleNamespace(tty_fd=None, original_terminal_settings=None, '
        'armed_screen_active=False, start_requested=False, mission_armed=True)\n'
        'for name in ("_open_terminal", "_write_terminal", "_enter_armed_screen", '
        '"_draw_armed_prompt", "_leave_armed_screen", "_restore_terminal", '
        '"_poll_keyboard"):\n'
        '    setattr(k, name, MethodType(getattr(K, name), k))\n'
        'k.get_logger = lambda: SimpleNamespace(info=lambda *a: None, '
        'error=lambda *a: None)\n'
        f'k._request_mission_start = lambda: Path({str(record)!r}).touch()\n'
        'k._open_terminal()\n'
        'assert k.tty_fd is not None\n'
        'try:\n'
        '    print("hidden runtime log", flush=True)\n'
        '    k._enter_armed_screen()\n'
        '    assert select.select([k.tty_fd], [], [], 5)[0]\n'
        '    k._poll_keyboard()\n'
        'finally:\n'
        '    k._restore_terminal()\n'
    )
    child_env[quiet.QUIET_ENV] = '1'
    master, slave = pty.openpty()

    def attach_terminal():
        os.setsid()
        fcntl.ioctl(slave, termios.TIOCSCTTY, 0)

    child = subprocess.Popen(
        command('--python', script), env=child_env, stdin=slave,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=attach_terminal,
    )
    try:
        assert select.select([master], [], [], 8)[0], 'Enter prompt missing'
        prompt = os.read(master, 4096)
        if b'Press Enter to start' not in prompt:
            assert select.select([master], [], [], 2)[0]
            prompt += os.read(master, 4096)
        assert b'Press Enter to start' in prompt
        assert b'hidden runtime log' not in prompt
        os.write(master, b'\n')
        stdout, stderr = child.communicate(timeout=8)
        assert child.returncode == 0
        assert stdout == stderr == b''
        assert record.exists()
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=5)
        os.close(master)
        os.close(slave)
