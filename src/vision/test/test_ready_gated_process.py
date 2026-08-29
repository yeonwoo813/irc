from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "ready_gated_process.py"
)


def _load_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "ready_gated_process",
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_boolean_launch_values_are_strict():
    module = _load_module()

    assert module.parse_bool("true") is True
    assert module.parse_bool("ON") is True
    assert module.parse_bool("0") is False
    assert module.parse_bool("no") is False
    with pytest.raises(Exception):
        module.parse_bool("maybe")


def test_retry_backoff_is_capped():
    module = _load_module()

    assert module.next_backoff(5.0, 30.0) == 10.0
    assert module.next_backoff(20.0, 30.0) == 30.0


def test_child_command_after_separator_is_normalized():
    module = _load_module()

    args = module.parse_arguments(
        [
            "--gate-topic",
            "/vision/webcam_yolo_ready",
            "--",
            "/usr/bin/python3",
            "detector.py",
        ]
    )
    assert args.command == ["/usr/bin/python3", "detector.py"]
    assert args.gate_open_delay_seconds == 0.0


def test_gate_open_delay_accepts_positive_seconds():
    module = _load_module()

    args = module.parse_arguments(
        [
            "--gate-topic",
            "/vision/webcam_yolo_ready",
            "--gate-open-delay-seconds",
            "3.0",
            "--",
            "/bin/true",
        ]
    )
    assert args.gate_open_delay_seconds == 3.0
