from copy import deepcopy
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import yolo_detector  # noqa: E402
from yolo_detector import (  # noqa: E402
    BallDetectionHold,
    ContinuousTrueGate,
    MotionDisplayState,
    ObjectDetection,
    analyze_frame_yolo,
    load_config,
    webcam_ball_evidence,
)


def _ball_payload(*, detected: bool, x1: float = 100.0) -> dict:
    if not detected:
        return {
            "ball_detected": False,
            "ball_x": -1.0,
            "ball_y": -1.0,
            "ball_conf": 0.0,
            "ball_bbox": [],
        }
    return {
        "ball_detected": True,
        "ball_x": x1 + 20.0,
        "ball_y": 120.0,
        "ball_conf": 0.95,
        "ball_bbox": [x1, 100.0, x1 + 40.0, 140.0],
    }


def test_webcam_ball_hold_keeps_true_then_expires_after_half_second():
    hold = BallDetectionHold(hold_seconds=0.5)

    detected = hold.apply(_ball_payload(detected=True), now=10.0)
    assert detected["ball_detected"] is True
    assert detected["ball_raw_detected"] is True
    assert detected["ball_hold_active"] is False

    held = hold.apply(_ball_payload(detected=False), now=10.2)
    assert held["ball_detected"] is True
    assert held["ball_raw_detected"] is False
    assert held["ball_hold_active"] is True
    assert held["ball_bbox"] == [100.0, 100.0, 140.0, 140.0]
    assert held["ball_hold_elapsed_sec"] == pytest.approx(0.2)
    assert held["ball_hold_remaining_sec"] == pytest.approx(0.3)

    expired = hold.apply(_ball_payload(detected=False), now=10.5)
    assert expired["ball_detected"] is False
    assert expired["ball_hold_active"] is False
    assert hold.last_valid_payload is None


def test_webcam_ball_reacquisition_immediately_stops_hold_and_uses_new_ball():
    hold = BallDetectionHold(hold_seconds=0.5)
    hold.apply(_ball_payload(detected=True, x1=100.0), now=20.0)
    held = hold.apply(_ball_payload(detected=False), now=20.2)
    assert held["ball_hold_active"] is True

    reacquired = hold.apply(_ball_payload(detected=True, x1=200.0), now=20.3)
    assert reacquired["ball_detected"] is True
    assert reacquired["ball_raw_detected"] is True
    assert reacquired["ball_hold_active"] is False
    assert reacquired["ball_bbox"] == [200.0, 100.0, 240.0, 140.0]

    held_again = hold.apply(_ball_payload(detected=False), now=20.6)
    assert held_again["ball_bbox"] == [200.0, 100.0, 240.0, 140.0]


def test_webcam_ball_hold_is_reset_when_detector_is_off():
    hold = BallDetectionHold(hold_seconds=0.5)
    hold.apply(_ball_payload(detected=True), now=30.0)

    inactive = hold.apply(
        _ball_payload(detected=False),
        active=False,
        now=30.1,
    )
    assert inactive["ball_detected"] is False
    assert inactive["ball_hold_active"] is False
    assert hold.last_valid_payload is None


def test_webcam_ball_hold_default_is_half_second():
    cfg = load_config("/__missing_webcam_yolo_test__.ini")
    assert cfg["ball_detection_hold_seconds"] == 0.5


@pytest.mark.parametrize("frame_count", [1, 2, 4, 5, 6, 9])
def test_webcam_ball_evidence_keeps_confidence_on_non_summary_frames(frame_count):
    hold = BallDetectionHold(hold_seconds=0.5)
    raw = _ball_payload(detected=True)
    raw["ball_conf"] = 0.73123456789
    payload = hold.apply(raw, now=10.0)
    before = deepcopy(payload)
    motion = MotionDisplayState()
    motion.on_command(26)
    motion.on_motion_state(motion_end=False, motion_ready=True)

    evidence = webcam_ball_evidence(payload, frame_count, motion)
    evidence = json.loads(json.dumps(evidence))

    assert evidence["frame"] == frame_count
    assert evidence["ball_conf"] == raw["ball_conf"]
    assert evidence["ball_raw"] is True
    assert evidence["ball_hold"] is False
    assert evidence["hold_elapsed_sec"] == 0.0
    assert evidence["ball_bbox"] == [100.0, 100.0, 140.0, 140.0]
    assert evidence["active_motion"] == 26
    assert evidence["motion_running"] is True
    assert payload == before


def test_webcam_ball_evidence_labels_held_confidence_and_stops_after_expiry():
    hold = BallDetectionHold(hold_seconds=0.5)
    hold.apply(_ball_payload(detected=True), now=10.0)
    motion = MotionDisplayState()
    held = hold.apply(_ball_payload(detected=False), now=10.2)

    evidence = webcam_ball_evidence(held, 2, motion)

    assert evidence["ball_conf"] == 0.95
    assert evidence["ball_raw"] is False
    assert evidence["ball_hold"] is True
    assert evidence["hold_elapsed_sec"] == pytest.approx(0.2)
    assert evidence["active_motion"] is None
    assert evidence["motion_running"] is False

    expired = hold.apply(_ball_payload(detected=False), now=10.5)
    assert webcam_ball_evidence(expired, 3, motion) is None


def test_webcam_ball_evidence_uses_reacquired_confidence():
    hold = BallDetectionHold(hold_seconds=0.5)
    hold.apply(_ball_payload(detected=True), now=10.0)
    hold.apply(_ball_payload(detected=False), now=10.2)
    raw = _ball_payload(detected=True, x1=200.0)
    raw["ball_conf"] = 0.42
    reacquired = hold.apply(raw, now=10.3)

    evidence = webcam_ball_evidence(reacquired, 3, MotionDisplayState())

    assert evidence["ball_conf"] == 0.42
    assert evidence["ball_raw"] is True
    assert evidence["ball_hold"] is False
    assert evidence["ball_bbox"] == [200.0, 100.0, 240.0, 140.0]


def test_webcam_ball_evidence_skips_missing_ball():
    assert webcam_ball_evidence(
        _ball_payload(detected=False), 1, MotionDisplayState()
    ) is None


def test_analyzer_outputs_held_true_and_draws_hold_overlay():
    cfg = load_config("/__missing_webcam_yolo_test__.ini")
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    ball = ObjectDetection(
        name=cfg["ball_class"],
        cls_id=0,
        conf=0.95,
        x1=100.0,
        y1=100.0,
        x2=140.0,
        y2=140.0,
    )
    hold = BallDetectionHold(hold_seconds=0.5)

    with (
        patch.object(yolo_detector, "yolo_detect", return_value=[ball]),
        patch.object(yolo_detector.time, "monotonic", return_value=40.0),
    ):
        detected, _image = analyze_frame_yolo(
            frame,
            model=object(),
            cfg=cfg,
            ball_detection_hold=hold,
        )
    assert detected["ball_raw_detected"] is True
    assert detected["ball_hold_active"] is False

    with (
        patch.object(yolo_detector, "yolo_detect", return_value=[]),
        patch.object(yolo_detector.time, "monotonic", return_value=40.2),
    ):
        held, image = analyze_frame_yolo(
            frame,
            model=object(),
            cfg=cfg,
            ball_detection_hold=hold,
        )

    assert held["ball_detected"] is True
    assert held["ball_raw_detected"] is False
    assert held["ball_hold_active"] is True
    assert held["ball_x"] == 120.0
    assert held["ball_angle_deg"] is not None
    assert held["raw_ball_in_hand"] is False
    assert np.any(np.all(image == np.array([255, 0, 255]), axis=2))


def test_webcam_ball_off_removes_current_detection_and_possession_state():
    cfg = load_config("/__missing_webcam_yolo_test__.ini")
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    ball = ObjectDetection(
        name=cfg["ball_class"],
        cls_id=0,
        conf=0.95,
        x1=300.0,
        y1=300.0,
        x2=340.0,
        y2=340.0,
    )
    gate = ContinuousTrueGate(hold_seconds=0.0)
    assert gate.update(True, now=1.0) is True

    with patch.object(yolo_detector, "yolo_detect", return_value=[ball]):
        payload, image = analyze_frame_yolo(
            frame,
            model=object(),
            cfg=cfg,
            raw_ball_in_hand_gate=gate,
            ball_detection_active=False,
        )

    assert payload["ball_detected"] is False
    assert payload["ball_detection_active"] is False
    assert payload["raw_ball_in_hand"] is False
    assert gate.started_at is None
    # 제어 결과는 OFF여도 모델의 원시 공 검출 박스는 보여야 게이트
    # 상태를 실제 YOLO 오검출/미검출과 구분할 수 있다.
    assert np.array_equal(image[300, 300], np.array([0, 180, 255]))


def test_webcam_ball_on_keeps_current_detection():
    cfg = load_config("/__missing_webcam_yolo_test__.ini")
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    ball = ObjectDetection(
        name=cfg["ball_class"],
        cls_id=0,
        conf=0.95,
        x1=300.0,
        y1=300.0,
        x2=340.0,
        y2=340.0,
    )

    with patch.object(yolo_detector, "yolo_detect", return_value=[ball]):
        payload, _image = analyze_frame_yolo(
            frame,
            model=object(),
            cfg=cfg,
            ball_detection_active=True,
        )

    assert payload["ball_detected"] is True
    assert payload["ball_detection_active"] is True
