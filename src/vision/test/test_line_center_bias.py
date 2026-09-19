"""Check distance decision bias without changing measured geometry."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from hurdle_vision_fusion import WebcamHurdlePublisherNode  # noqa: E402
from line_status_publisher import LineStatusPublisher  # noqa: E402
from yolo_detector import (  # noqa: E402
    LinePayloadSmoother,
    LineStatus,
    ObjectDetection,
    apply_line_status,
    load_config,
    make_line_payload,
    make_vision_payload,
    visualize_yolo,
)


def test_configured_bias_preserves_all_measured_geometry(tmp_path):
    settings = tmp_path / "settings.ini"
    settings.write_text("[camera]\nline_center_bias_px = 15.0\n")
    cfg = load_config(str(settings))
    points = [(360.0, 450.0), (360.0, 350.0)]
    detections = [ObjectDetection("ball", 1, 0.99, 390, 290, 410, 310)]
    original = make_vision_payload(
        detections, points, 640, 480, dict(cfg, line_center_bias_px=0.0)
    )
    corrected = make_vision_payload(detections, points, 640, 480, cfg)

    assert corrected == original
    assert corrected["line_distance"] == 15.0
    assert corrected["follow_angle"] > 0.0
    assert corrected["robot_center_x"] == 345.0
    apply_line_status(
        corrected, 640, 480, line_center_bias_px=cfg["line_center_bias_px"]
    )
    for key, value in original.items():
        assert corrected[key] == value, key


@pytest.mark.parametrize("curve", [False, True])
@pytest.mark.parametrize("raw_distance, expected", [
    (-90.0, LineStatus.Left_Half_Forward),
    (-89.99, LineStatus.Forward_4step),
    (-80.0, LineStatus.Forward_4step),
    (15.0, LineStatus.Forward_4step),
    (89.99, LineStatus.Forward_4step),
    (90.0, LineStatus.Right_Half_Forward),
    (100.0, LineStatus.Right_Half_Forward),
])
def test_workspace_config_restores_original_distance_boundaries(
    curve, raw_distance, expected
):
    cfg = load_config(str(SCRIPTS_DIR.parent / "config" / "settings.ini"))
    assert cfg["line_center_bias_px"] == 0.0
    x = 345.0 + raw_distance
    points = (
        [(x, 450), (x - 30, 350), (x - 40, 250), (x - 30, 150)]
        if curve else [(x, 450), (x, 350), (x, 250)]
    )
    payload = make_line_payload(points, 640, 480)
    result = apply_line_status(
        dict(payload), 640, 480, line_center_bias_px=cfg["line_center_bias_px"]
    )
    assert result["status"] == expected
    assert result["line_distance"] == pytest.approx(raw_distance)

    messages = []
    node = SimpleNamespace(
        create_publisher=lambda *_: SimpleNamespace(publish=messages.append)
    )
    publisher = LineStatusPublisher(
        node, line_center_bias_px=cfg["line_center_bias_px"]
    )
    published = apply_line_status(dict(payload), 640, 480, publisher)
    assert published["status"] == expected
    assert messages[-1].status == expected
    assert messages[-1].line_distance == pytest.approx(raw_distance)


@pytest.mark.parametrize("curve", [False, True])
@pytest.mark.parametrize("raw_distance, expected", [
    (-75.0, LineStatus.Left_Half_Forward),
    (-74.99, LineStatus.Forward_4step),
    (15.0, LineStatus.Forward_4step),
    (104.99, LineStatus.Forward_4step),
    (105.0, LineStatus.Right_Half_Forward),
])
def test_shifted_distance_boundaries(curve, raw_distance, expected):
    x = 345.0 + raw_distance
    points = (
        [(x, 450), (x - 30, 350), (x - 40, 250), (x - 30, 150)]
        if curve else [(x, 450), (x, 350), (x, 250)]
    )
    payload = make_line_payload(points, 640, 480)
    result = apply_line_status(payload, 640, 480, line_center_bias_px=15.0)

    assert result["line_distance"] == pytest.approx(raw_distance)
    assert result["status"] == expected


@pytest.mark.parametrize("x, expected", [
    (344.0, LineStatus.Left_Turn),
    (345.0, LineStatus.Forward_4step),
    (346.0, LineStatus.Right_Turn),
])
def test_single_point_follow_keeps_original_angle_reference(x, expected):
    payload = make_line_payload([(x, 400)], 640, 480)
    assert apply_line_status(
        payload, 640, 480, line_center_bias_px=15.0
    )["status"] == expected


def test_hurdle_fusion_uses_original_angle_after_smoothing():
    smoother = LinePayloadSmoother()
    for _ in range(3):
        payload = make_line_payload([(355.0, 400)], 640, 480)
        payload = smoother.smooth(payload, 640, 480)
    apply_line_status(payload, 640, 480, line_center_bias_px=15.0)
    assert payload["follow_angle"] > 0.0
    harness = SimpleNamespace(
        webcam_min_conf=0.0,
        _finite_float=WebcamHurdlePublisherNode._finite_float,
        _optional_finite_float=WebcamHurdlePublisherNode._optional_finite_float,
        _nonnegative_int=WebcamHurdlePublisherNode._nonnegative_int,
    )
    WebcamHurdlePublisherNode.cb_webcam_state(
        harness, SimpleNamespace(data=json.dumps(payload))
    )
    assert harness.latest_webcam["line_follow_angle_deg"] == (
        payload["follow_angle"]
    )


def test_ros_decision_applies_bias_but_publishes_raw_distance():
    messages = []
    node = SimpleNamespace(
        create_publisher=lambda *_: SimpleNamespace(publish=messages.append)
    )
    publisher = LineStatusPublisher(node, line_center_bias_px=15.0)
    payload = make_line_payload([(445, 450), (445, 350), (445, 250)], 640, 480)
    assert apply_line_status(dict(payload), 640, 480)["status"] == (
        LineStatus.Right_Half_Forward
    )
    result = apply_line_status(payload, 640, 480, publisher)
    assert result["status"] == LineStatus.Forward_4step
    assert result["line_distance"] == 100.0
    assert messages[-1].status == LineStatus.Forward_4step
    assert messages[-1].line_distance == 100.0


def test_missing_line_stays_lost_and_overlay_keeps_original_center():
    cfg = {
        "line_class": "line",
        "ball_class": "ball",
        "ball_conf": 0.2,
        "hurdle_class": "hurdle",
        "hurdle_conf": 0.2,
        "line_center_bias_px": 15.0,
    }
    payload = make_vision_payload([], [], 640, 480, cfg)
    payload = apply_line_status(payload, 640, 480, line_center_bias_px=15.0)
    assert payload["status"] == LineStatus.Line_Lost
    assert payload["line_distance"] == 0.0
    image = visualize_yolo(
        np.zeros((480, 640, 3), dtype=np.uint8), [], [], payload,
        (0, 480, 0, 640), cfg,
    )
    assert image[300, 345].max() > 0
    assert image[300, 360].max() == 0
