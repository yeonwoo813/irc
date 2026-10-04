"""Diagnostic candidates must not change the robot's vision decisions."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import yolo_detector as yolo  # noqa: E402


@pytest.fixture
def cfg():
    return yolo.load_config(str(SCRIPTS_DIR.parent / "config" / "settings.ini"))


@pytest.fixture
def analyze(monkeypatch, cfg):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    monkeypatch.setattr(yolo, "LINE_SMOOTHER", yolo.LinePayloadSmoother())
    monkeypatch.setattr(yolo, "visualize_yolo", lambda frame, *args: frame)

    def run(detections, motion=None):
        monkeypatch.setattr(yolo, "yolo_detect", lambda *args: detections)
        return yolo.analyze_frame_yolo(
            frame, object(), cfg, motion_state=motion
        )[0]

    return run


def detection(conf, *, name="line", x=345.0, y=400.0):
    return yolo.ObjectDetection(name, 0, conf, x - 5, y - 5, x + 5, y + 5)


@pytest.mark.parametrize("existing_conf, expected", [(0.35, 0.30), (0.20, 0.20)])
def test_inference_collects_diagnostics_without_raising_existing_floor(
    cfg, existing_conf, expected
):
    cfg["yolo_conf"] = existing_conf
    calls = []
    box = SimpleNamespace(
        cls=[0], conf=[0.32], xyxy=np.array([[100, 200, 120, 220]])
    )

    def predict(**kwargs):
        calls.append(kwargs)
        return [SimpleNamespace(boxes=[box])]

    model = SimpleNamespace(names={0: "line"}, predict=predict)
    result = yolo.yolo_detect(model, np.zeros((10, 10, 3)), cfg)
    assert calls[0]["conf"] == expected
    assert result[0].conf == 0.32
    assert cfg["line_conf"] == 0.35


def test_below_threshold_candidates_are_logged_but_stay_lost(analyze):
    detections = [detection(0.299), detection(0.30), detection(0.349)]
    for _ in range(5):
        payload = analyze(detections)
    evidence = payload["line_evidence"]
    assert [d["conf"] for d in evidence["candidates"]] == [0.30, 0.349]
    assert all(not d["used_for_raw_line"] for d in evidence["candidates"])
    assert all(d["rejection_reason"] == "below_line_conf"
               for d in evidence["candidates"])
    assert evidence["raw_point_count"] == evidence["point_count"] == 0
    assert payload["status"] == yolo.LineStatus.Line_Lost


def test_confident_line_can_be_rejected_by_temporal_filter(analyze):
    motion = yolo.MotionDisplayState()
    motion.on_command(25)
    motion.on_motion_state(False, True)
    detections = [detection(0.80)]
    first = analyze(detections, motion)["line_evidence"]
    assert first["max_conf"] == 0.80
    assert first["raw_point_count"] == 1
    assert first["point_count"] == 0
    assert first["lost_reason"] == "insufficient_valid_frames"
    assert first["smooth_valid_count"] == 1
    assert first["candidates"][0]["used_for_raw_line"] is True
    assert first["active_motion"] == 25 and first["motion_running"] is True

    analyze(detections, motion)
    third = analyze(detections, motion)["line_evidence"]
    assert third["smooth_valid_count"] == 3
    assert third["point_count"] == 1
    assert third["lost"] is False and third["lost_reason"] is None


def test_rejected_roi_and_accepted_boundary_keep_actual_coordinates(analyze, cfg):
    cfg["roi_left_ratio"] = 0.25
    payload = analyze([detection(0.35), detection(0.90, x=100)])
    accepted, rejected = payload["line_evidence"]["candidates"]
    assert accepted["used_for_raw_line"] is True
    assert accepted["bbox"] == [340, 395, 350, 405]
    assert accepted["center"] == [345, 400]
    assert rejected["conf"] == 0.90
    assert rejected["rejection_reason"] == "outside_roi"
    assert payload["raw_point_count"] == 1


def test_no_candidate_uses_null_even_when_history_keeps_line_visible(analyze):
    empty = analyze([])["line_evidence"]
    assert empty["candidates"] == [] and empty["max_conf"] is None
    assert empty["lost"] is True
    assert '"max_conf": null' in json.dumps(empty, allow_nan=False)
    for _ in range(3):
        analyze([detection(0.8)])
    history = analyze([])["line_evidence"]
    assert history["candidate_count"] == 0 and history["max_conf"] is None
    assert history["raw_point_count"] == 0 and history["point_count"] == 1
    assert history["lost"] is False


def test_diagnostic_candidates_leave_all_existing_payload_and_display_unchanged(
    analyze, cfg, monkeypatch
):
    # Even if a class threshold is below the original global threshold,
    # lowering the diagnostic inference floor must not admit those objects.
    cfg.update(ball_conf=0.20, hurdle_conf=0.20)
    accepted = [detection(0.8), detection(0.7, y=300)]
    diagnostic_only = [detection(0.32, name=name)
                       for name in ("line", "ball", "hurdle")]
    shown = []

    def visualize(frame, detections, *args):
        shown.append(detections)
        return frame

    monkeypatch.setattr(yolo, "visualize_yolo", visualize)
    for _ in range(3):
        baseline = analyze(accepted)
    yolo.LINE_SMOOTHER.reset()
    for _ in range(3):
        diagnosed = analyze(accepted + diagnostic_only)
    baseline.pop("line_evidence")
    evidence = diagnosed.pop("line_evidence")
    assert baseline == diagnosed
    assert evidence["candidate_count"] == 3
    assert shown[-1] == accepted


def test_diagnostic_exclusion_also_reports_original_global_threshold(analyze, cfg):
    cfg["line_conf"] = 0.30
    evidence = analyze([detection(0.32)])["line_evidence"]
    assert evidence["candidates"][0]["rejection_reason"] == "below_yolo_conf"
    assert evidence["raw_point_count"] == 0


def test_quiet_mode_skips_diagnostics_and_preserves_control(analyze, monkeypatch):
    detections = [detection(0.8), detection(0.7, y=300), detection(0.32)]
    for _ in range(3):
        baseline = analyze(detections)
    baseline.pop("line_evidence")
    yolo.LINE_SMOOTHER.reset()
    monkeypatch.setattr(yolo, "QUIET_LOGS", True)

    def unexpected_diagnostic(*args):
        raise AssertionError("quiet mode built diagnostic data")

    monkeypatch.setattr(yolo, "webcam_line_evidence", unexpected_diagnostic)
    for _ in range(3):
        quiet_payload = analyze(detections)
    assert quiet_payload == baseline


def test_quiet_inference_uses_control_threshold(cfg, monkeypatch):
    monkeypatch.setattr(yolo, "QUIET_LOGS", True)
    calls = []

    def predict(**kwargs):
        calls.append(kwargs)
        return [SimpleNamespace(boxes=[])]

    model = SimpleNamespace(names={0: "line"}, predict=predict)
    assert yolo.yolo_detect(model, np.zeros((10, 10, 3)), cfg) == []
    assert calls[0]["conf"] == cfg["yolo_conf"]
