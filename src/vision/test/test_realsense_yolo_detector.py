import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np
from std_msgs.msg import Bool


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "realsense_yolo_detector.py"
)
SPEC = importlib.util.spec_from_file_location(
    "realsense_yolo_detector",
    SCRIPT,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_loads_dedicated_realsense_yolo_settings(tmp_path):
    ini = tmp_path / "settings.ini"
    ini.write_text(
        """
[realsense_yolo]
model = /tmp/realsense.engine
goal_class = goal
backboard_class = backboard
ball_class = ball
max_fps = 30
ball_detection_hold_seconds = 0.5
backboard_detection_hold_seconds = 0.5
publish_debug_image = true
""".strip(),
        encoding="utf-8",
    )

    config = MODULE.load_config(str(ini))

    assert config["model"] == "/tmp/realsense.engine"
    assert config["goal_class"] == "goal"
    assert config["backboard_class"] == "backboard"
    assert config["ball_class"] == "ball"
    assert config["max_fps"] == 30.0
    assert config["ball_detection_hold_seconds"] == 0.5
    assert config["backboard_detection_hold_seconds"] == 0.5
    assert config["publish_debug_image"] is True


def test_ball_dropout_holds_last_distance_angle_and_box_for_half_second():
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.cfg = {"ball_detection_hold_seconds": 0.5}
    detector.ball_active = True
    detector._reset_ball_detection_hold()

    first_detection = MODULE.Detection("ball", 2, 0.91, 40, 50, 80, 90)
    first_state = detector._empty_ball_state(True)
    first_state.update(
        {
            "realsense_ball_detected": True,
            "realsense_ball_distance_cm": 81.2,
            "realsense_ball_angle_error": -4.3,
            "raw_detected": True,
            "raw_ball_conf": first_detection.conf,
            "raw_ball_bbox": [40, 50, 80, 90],
        }
    )

    accepted, displayed = detector._apply_ball_detection_hold(
        first_state, first_detection, 10.0
    )
    assert accepted["held_previous_detection"] is False
    assert displayed == first_detection

    missing_state = detector._empty_ball_state(True)
    missing_state["process_ms"] = 12.3
    held, displayed = detector._apply_ball_detection_hold(
        missing_state, None, 10.2
    )

    assert held["realsense_ball_detected"] is True
    assert held["raw_detected"] is False
    assert held["held_previous_detection"] is True
    assert held["realsense_ball_distance_cm"] == 81.2
    assert held["realsense_ball_angle_error"] == -4.3
    assert np.isclose(held["ball_hold_elapsed_sec"], 0.2)
    assert np.isclose(held["ball_hold_remaining_sec"], 0.3)
    assert displayed == first_detection

    expired, displayed = detector._apply_ball_detection_hold(
        detector._empty_ball_state(True), None, 10.5
    )
    assert expired["realsense_ball_detected"] is False
    assert expired["held_previous_detection"] is False
    assert displayed is None


def test_ball_reacquisition_immediately_replaces_held_result():
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.cfg = {"ball_detection_hold_seconds": 0.5}
    detector.ball_active = True
    detector._reset_ball_detection_hold()

    old_detection = MODULE.Detection("ball", 2, 0.90, 10, 20, 30, 40)
    old_state = detector._empty_ball_state(True)
    old_state.update(
        {
            "realsense_ball_detected": True,
            "realsense_ball_distance_cm": 100.0,
            "realsense_ball_angle_error": 8.0,
        }
    )
    detector._apply_ball_detection_hold(old_state, old_detection, 20.0)
    detector._apply_ball_detection_hold(
        detector._empty_ball_state(True), None, 20.2
    )

    new_detection = MODULE.Detection("ball", 2, 0.95, 50, 60, 70, 80)
    new_state = detector._empty_ball_state(True)
    new_state.update(
        {
            "realsense_ball_detected": True,
            "realsense_ball_distance_cm": 72.0,
            "realsense_ball_angle_error": -2.0,
        }
    )
    reacquired, displayed = detector._apply_ball_detection_hold(
        new_state, new_detection, 20.3
    )

    assert reacquired["held_previous_detection"] is False
    assert reacquired["realsense_ball_distance_cm"] == 72.0
    assert reacquired["realsense_ball_angle_error"] == -2.0
    assert displayed == new_detection


def test_backboard_dropout_holds_last_geometry_then_reacquires_or_expires():
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.cfg = {"backboard_detection_hold_seconds": 0.5}
    detector.hoop_active = True
    detector._reset_backboard_detection_hold()

    old_detection = MODULE.Detection(
        "backboard", 1, 0.88, 100, 40, 220, 130
    )
    old_state = detector._empty_hoop_state(True)
    old_state.update(
        {
            "detected": True,
            "raw_detected": True,
            "backboard_detected": True,
            "center_x": 160.0,
            "center_y": 85.0,
            "realsense_goal_distance_cm": 100.0,
            "realsense_goal_angle": 8.0,
            "center_depth_cm": 168.0,
            "confidence": old_detection.conf,
            "backboard_bbox": [100, 40, 220, 130],
        }
    )
    accepted, displayed = detector._apply_backboard_detection_hold(
        old_state, old_detection, 30.0
    )
    assert accepted["held_previous_detection"] is False
    assert displayed == old_detection

    missing_state = detector._empty_hoop_state(True)
    missing_state["process_ms"] = 11.5
    held, displayed = detector._apply_backboard_detection_hold(
        missing_state, None, 30.2
    )
    assert held["detected"] is True
    assert held["backboard_detected"] is True
    assert held["raw_detected"] is False
    assert held["held_previous_detection"] is True
    assert held["realsense_goal_distance_cm"] == 100.0
    assert held["realsense_goal_angle"] == 8.0
    assert held["center_x"] == 160.0
    assert held["center_y"] == 85.0
    assert held["backboard_bbox"] == [100, 40, 220, 130]
    assert np.isclose(held["backboard_hold_elapsed_sec"], 0.2)
    assert np.isclose(held["backboard_hold_remaining_sec"], 0.3)
    assert displayed == old_detection

    new_detection = MODULE.Detection(
        "backboard", 1, 0.93, 110, 45, 230, 135
    )
    new_state = detector._empty_hoop_state(True)
    new_state.update(
        {
            "detected": True,
            "realsense_goal_distance_cm": 150.0,
            "realsense_goal_angle": 3.0,
        }
    )
    reacquired, displayed = detector._apply_backboard_detection_hold(
        new_state, new_detection, 30.3
    )
    assert reacquired["held_previous_detection"] is False
    assert reacquired["realsense_goal_distance_cm"] == 150.0
    assert reacquired["realsense_goal_angle"] == 3.0
    assert displayed == new_detection

    expired, displayed = detector._apply_backboard_detection_hold(
        detector._empty_hoop_state(True), None, 30.8
    )
    assert expired["detected"] is False
    assert expired["held_previous_detection"] is False
    assert displayed is None


def test_backboard_hold_is_limited_to_80_through_120_cm():
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.cfg = {"backboard_detection_hold_seconds": 0.5}
    detector.hoop_active = True
    detection = MODULE.Detection(
        "backboard", 1, 0.9, 100, 40, 220, 130
    )

    for distance_cm, should_hold in (
        (79.9, False),
        (80.0, True),
        (120.0, True),
        (120.1, False),
    ):
        detector._reset_backboard_detection_hold()
        state = detector._empty_hoop_state(True)
        state.update(
            {
                "detected": True,
                "realsense_goal_distance_cm": distance_cm,
                "realsense_goal_angle": 0.0,
            }
        )
        detector._apply_backboard_detection_hold(
            state, detection, 40.0
        )
        held, displayed = detector._apply_backboard_detection_hold(
            detector._empty_hoop_state(True), None, 40.2
        )

        assert held["detected"] is should_hold
        assert held["held_previous_detection"] is should_hold
        assert (displayed == detection) is should_hold


def test_ball_hold_is_labeled_on_realsense_debug_view(monkeypatch):
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.cfg = {
        "ball_class": "ball",
        "backboard_class": "backboard",
        "goal_class": "goal",
    }
    detector.centerline_x_px = 120.0
    detector.ball_active = True
    detector.hoop_active = False
    labels = []
    panels = []
    detector._draw_detection = (
        lambda _frame, _detection, label: labels.append(label)
    )
    detector._draw_target_guide = lambda _frame, _detection: None
    monkeypatch.setattr(
        MODULE,
        "draw_info_panel",
        lambda _frame, lines, **_kwargs: panels.append(lines),
    )

    ball = MODULE.Detection("ball", 2, 0.91, 40, 50, 80, 90)
    ball_state = detector._empty_ball_state(True)
    ball_state.update(
        {
            "realsense_ball_detected": True,
            "held_previous_detection": True,
            "ball_hold_remaining_sec": 0.3,
            "realsense_ball_distance_cm": 81.2,
            "realsense_ball_angle_error": -4.3,
        }
    )
    detector._draw_debug_view(
        np.zeros((240, 320, 3), dtype=np.uint8),
        "ball",
        ball,
        None,
        ball_state,
        detector._empty_hoop_state(False),
        12.4,
    )

    assert labels == ["BALL HOLD"]
    assert any("detect:HOLD" in line for line in panels[0])
    assert any("hold:0.30s remaining" in line for line in panels[0])


def test_backboard_hold_is_labeled_on_realsense_debug_view(monkeypatch):
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.centerline_x_px = 120.0
    detector.ball_active = False
    detector.hoop_active = True
    labels = []
    panels = []
    detector._draw_detection = (
        lambda _frame, _detection, label: labels.append(label)
    )
    detector._draw_target_guide = lambda _frame, _detection: None
    monkeypatch.setattr(
        MODULE,
        "draw_info_panel",
        lambda _frame, lines, **_kwargs: panels.append(lines),
    )

    backboard = MODULE.Detection(
        "backboard", 1, 0.88, 100, 40, 220, 130
    )
    hoop_state = detector._empty_hoop_state(True)
    hoop_state.update(
        {
            "detected": True,
            "held_previous_detection": True,
            "backboard_hold_remaining_sec": 0.3,
            "realsense_goal_distance_cm": 170.0,
            "realsense_goal_angle": 8.0,
        }
    )
    detector._draw_debug_view(
        np.zeros((240, 320, 3), dtype=np.uint8),
        "hoop",
        None,
        backboard,
        detector._empty_ball_state(False),
        hoop_state,
        12.4,
    )

    assert labels == ["BACKBOARD HOLD"]
    assert any("detect:HOLD" in line for line in panels[0])
    assert any("hold:0.30s remaining" in line for line in panels[0])


def test_goal_detections_are_discarded_before_state_or_visualization():
    class Value:
        def __init__(self, value):
            self.value = value

        def item(self):
            return self.value

    class Coordinates:
        def __init__(self, values):
            self.values = values

        def tolist(self):
            return self.values

    class Box:
        def __init__(self, cls_id, confidence, coordinates):
            self.cls = [Value(cls_id)]
            self.conf = [Value(confidence)]
            self.xyxy = [Coordinates(coordinates)]

    class Result:
        names = {0: "goal", 1: "backboard", 2: "ball"}
        boxes = [
            Box(0, 0.99, [10, 10, 30, 30]),
            Box(1, 0.88, [40, 40, 80, 80]),
            Box(2, 0.91, [90, 90, 120, 120]),
        ]

    class Model:
        def predict(self, **_kwargs):
            return [Result()]

    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.model = Model()
    detector.cfg = {
        "imgsz": 640,
        "conf": 0.20,
        "device": "0",
        "ball_class": "ball",
        "ball_conf": 0.25,
        "backboard_class": "backboard",
        "backboard_conf": 0.30,
        "goal_class": "goal",
    }

    detections = detector._run_yolo(np.zeros((120, 160, 3), dtype=np.uint8))

    assert [detection.name for detection in detections] == ["backboard", "ball"]


def test_ball_and_backboard_views_have_yellow_boxes_and_information_panels():
    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.cfg = {
        "ball_class": "ball",
        "backboard_class": "backboard",
        "goal_class": "goal",
    }
    detector.centerline_x_px = 120.0
    detector.ball_active = True
    detector.hoop_active = False

    ball = MODULE.Detection("ball", 2, 0.91, 40, 175, 90, 225)
    backboard = MODULE.Detection(
        "backboard",
        1,
        0.88,
        145,
        155,
        260,
        225,
    )
    ball_state = {
        "realsense_ball_detected": True,
        "realsense_ball_distance_cm": 81.2,
        "realsense_ball_angle_error": -4.3,
        "raw_x_m": -0.06,
        "raw_y_m": 0.14,
        "raw_z_m": 0.79,
    }
    hoop_state = {
        "detected": True,
        "center_x": 202.5,
        "center_y": 87.5,
        "realsense_goal_distance_cm": 170.0,
        "realsense_goal_angle": 8.0,
        "center_depth_cm": 168.0,
    }
    frame = np.zeros((240, 320, 3), dtype=np.uint8)

    ball_view = detector._draw_debug_view(
        frame,
        "ball",
        ball,
        backboard,
        ball_state,
        hoop_state,
        12.4,
    )
    hoop_view = detector._draw_debug_view(
        frame,
        "hoop",
        ball,
        backboard,
        ball_state,
        hoop_state,
        12.4,
    )
    combined_view = detector._draw_debug_view(
        frame,
        "combined",
        ball,
        backboard,
        ball_state,
        hoop_state,
        12.4,
    )

    yellow = np.array(MODULE.YELLOW, dtype=np.uint8)
    assert np.array_equal(ball_view[175, 40], yellow)
    assert np.array_equal(hoop_view[155, 145], yellow)
    assert np.array_equal(combined_view[175, 40], yellow)
    assert np.array_equal(combined_view[155, 145], yellow)
    # The old GOAL box used this top edge; only BACKBOARD may be drawn now.
    assert not np.array_equal(hoop_view[170, 200], yellow)
    assert np.count_nonzero(cv2.inRange(ball_view, yellow, yellow)) > 100
    assert np.count_nonzero(cv2.inRange(hoop_view, yellow, yellow)) > 100
    assert not np.array_equal(ball_view, hoop_view)


def test_display_mode_survives_off_then_switches_on_next_active_signal():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Logger:
        def info(self, _message):
            pass

    detector = MODULE.RealSenseYoloDetector.__new__(
        MODULE.RealSenseYoloDetector
    )
    detector.ball_active = True
    detector.hoop_active = False
    detector.display_mode = "ball"
    detector.ball_state_pub = Publisher()
    detector.ball_detected_pub = Publisher()
    detector.hoop_state_pub = Publisher()
    detector.hoop_detected_pub = Publisher()
    detector.get_logger = lambda: Logger()

    detector.cb_ball_active(Bool(data=False))
    assert detector.ball_active is False
    assert detector.display_mode == "ball"

    detector.cb_hoop_active(Bool(data=True))
    assert detector.hoop_active is True
    assert detector.display_mode == "hoop"
