"""Verify startup failures never launch dependent camera/vision processes."""

import importlib.util
from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetLaunchConfiguration, Shutdown
from launch.launch_context import LaunchContext
import pytest


@pytest.fixture
def stack(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    path = Path(__file__).resolve().parents[1] / 'launch/vision_stack.launch.py'
    spec = importlib.util.spec_from_file_location('vision_stack', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # launch owns asynchronous stdout pipes and process signal handlers.
    # pytest's descriptor capture can otherwise keep those pipes alive.
    with capsys.disabled():
        yield module


@pytest.mark.parametrize('device', ['auto', '/definitely-missing-camera'])
def test_missing_webcam_never_starts_camera(stack, tmp_path, monkeypatch, device):
    marker = tmp_path / 'unexpected-start'

    def fake_node(**kwargs):
        return ExecuteProcess(cmd=[
            sys.executable, '-c',
            f'from pathlib import Path; Path({str(marker)!r}).touch()',
        ])

    monkeypatch.setattr(stack, 'Node', fake_node)
    monkeypatch.setattr(stack.glob, 'glob', lambda pattern: [])
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('webcam_device', device),
        stack.generate_launch_description(),
    ]))
    assert service.run() != 0
    assert not marker.exists()


@pytest.mark.parametrize('ownership_acquired', [True, False])
def test_only_camera_owner_starts_dependents(stack, tmp_path, ownership_acquired):
    marker = tmp_path / 'dependent-started'
    # Deliberately split the guard message across separate output events.
    message = stack.READY_MARKER
    driver_code = (
        'import sys, time; '
        f'sys.stdout.write({message[:12]!r}); sys.stdout.flush(); '
        'time.sleep(0.1); '
        f'print({message[12:]!r}, flush=True); time.sleep(0.3)'
        if ownership_acquired else 'raise SystemExit(1)'
    )
    camera = ExecuteProcess(
        cmd=[sys.executable, '-u', '-c', driver_code],
        on_exit=[Shutdown(reason='test driver finished')],
    )
    dependent = ExecuteProcess(cmd=[
        sys.executable, '-c',
        f'from pathlib import Path; Path({str(marker)!r}).touch()',
    ])
    service = LaunchService()
    actions = stack._after_camera_ownership(service.context, camera, [dependent])
    service.include_launch_description(LaunchDescription(actions))
    service.run()
    assert marker.exists() is ownership_acquired


def test_realsense_only_does_not_require_webcam(stack):
    context = LaunchContext()
    context.launch_configurations['start_webcam'] = 'false'
    assert stack._make_webcam_node(context) == []
