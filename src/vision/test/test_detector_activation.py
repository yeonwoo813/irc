from collections import deque
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

import numpy as np
from std_msgs.msg import Bool


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from ball_vision_fusion import BallVisionFusionNode  # noqa: E402
from hoop_vision import HoopVisionNode  # noqa: E402


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _Publisher:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, msg):
        self.events.append((self.name, bool(msg.data)))


def test_inactive_ball_callbacks_skip_image_and_webcam_processing():
    harness = SimpleNamespace(ball_detection_active=False)

    BallVisionFusionNode.cb_realsense_images(harness, object(), object())
    BallVisionFusionNode.cb_webcam_state(harness, object())


def test_ball_activity_switch_keeps_warm_subscriptions_and_clears_state():
    logger = _Logger()
    color_sub = object()
    depth_sub = object()
    sync = object()
    detection_enabled_calls = []
    harness = SimpleNamespace(
        ball_detection_active=True,
        latest_realsense={"realsense_ball_detected": True},
        latest_realsense_time=1.0,
        last_realsense_detection={"realsense_ball_detected": True},
        realsense_lost_frames=0,
        realsense_hold_frames=3,
        latest_webcam={"webcam_ball_detected": True},
        latest_webcam_time=1.0,
        rs_color_sub=color_sub,
        rs_depth_sub=depth_sub,
        rs_sync=sync,
        get_logger=lambda: logger,
        ball_status_publisher=SimpleNamespace(
            set_detection_enabled=(
                lambda enabled: detection_enabled_calls.append(enabled)
            )
        ),
    )
    harness._clear_ball_detection_state = MethodType(
        BallVisionFusionNode._clear_ball_detection_state,
        harness,
    )

    BallVisionFusionNode.cb_ball_active(harness, Bool(data=False))
    assert harness.ball_detection_active is False
    assert harness.latest_realsense is None
    assert harness.latest_webcam is None
    assert (harness.rs_color_sub, harness.rs_depth_sub, harness.rs_sync) == (
        color_sub,
        depth_sub,
        sync,
    )

    BallVisionFusionNode.cb_ball_active(harness, Bool(data=True))
    assert harness.ball_detection_active is True
    assert detection_enabled_calls == [False, True]
    assert (harness.rs_color_sub, harness.rs_depth_sub, harness.rs_sync) == (
        color_sub,
        depth_sub,
        sync,
    )


def test_realsense_rejection_diagnostics_distinguish_hsv_depth_distance():
    harness = SimpleNamespace(depth_threshold_m=1.5)
    empty = np.zeros((2, 2), dtype=np.uint8)

    hsv_diagnostic = BallVisionFusionNode._classify_realsense_rejection(
        harness,
        ball_color_mask=empty,
        roi_depth_m=np.ones((2, 2), dtype=np.float32),
        depth_filtered_mask=empty,
        rejection_counts={},
    )
    assert hsv_diagnostic["category"] == "hsv"

    color_mask = np.full((2, 2), 255, dtype=np.uint8)
    depth_diagnostic = BallVisionFusionNode._classify_realsense_rejection(
        harness,
        ball_color_mask=color_mask,
        roi_depth_m=np.zeros((2, 2), dtype=np.float32),
        depth_filtered_mask=empty,
        rejection_counts={},
    )
    assert depth_diagnostic["category"] == "depth"

    distance_diagnostic = BallVisionFusionNode._classify_realsense_rejection(
        harness,
        ball_color_mask=color_mask,
        roi_depth_m=np.full((2, 2), 2.0, dtype=np.float32),
        depth_filtered_mask=empty,
        rejection_counts={},
    )
    assert distance_diagnostic["category"] == "distance"


def test_realsense_publish_diagnostic_reports_ball_entry_distance_gate():
    harness = SimpleNamespace(
        ball_detection_active=True,
        realsense_timeout_sec=0.5,
        latest_realsense={
            "realsense_diagnostic": {
                "category": "accepted",
                "detail": "detector_passed",
            }
        },
        ball_status_publisher=SimpleNamespace(
            ball_decision=SimpleNamespace(ball_entry_distance_cm=120.0)
        ),
    )

    diagnostic = BallVisionFusionNode._published_realsense_diagnostic(
        harness,
        realsense_valid=True,
        realsense_age=0.0,
        features={
            "ball_in_hand": False,
            "realsense_ball_distance_cm": 140.0,
        },
    )

    assert diagnostic["category"] == "distance"
    assert diagnostic["distance_cm"] == 140.0
    assert diagnostic["limit_cm"] == 120.0


def test_realsense_rejection_log_is_emitted_only_on_reason_change():
    logger = _Logger()
    harness = SimpleNamespace(
        last_realsense_diagnostic_label=None,
        get_logger=lambda: logger,
    )
    hsv = {
        "category": "hsv",
        "detail": "no_pixels_in_hsv_range",
        "hsv_pixels": 0,
    }
    depth = {
        "category": "depth",
        "detail": "no_finite_positive_depth",
        "hsv_pixels": 12,
    }

    assert BallVisionFusionNode._log_realsense_diagnostic_transition(
        harness,
        hsv,
    ) is True
    assert BallVisionFusionNode._log_realsense_diagnostic_transition(
        harness,
        hsv,
    ) is False
    assert BallVisionFusionNode._log_realsense_diagnostic_transition(
        harness,
        depth,
    ) is True

    assert len(logger.messages) == 2
    assert "REJECT_HSV" in logger.messages[0]
    assert "REJECT_DEPTH" in logger.messages[1]


def test_hoop_activity_switch_keeps_warm_subscriptions_and_clears_history():
    logger = _Logger()
    color_sub = object()
    depth_sub = object()
    sync = object()
    published = []
    harness = SimpleNamespace(
        active=True,
        history=deque([{"detected": True}], maxlen=5),
        last_detection={"detected": True},
        last_detection_time=1.0,
        color_sub=color_sub,
        depth_sub=depth_sub,
        sync=sync,
        get_logger=lambda: logger,
        _publish_state=lambda **kwargs: published.append(kwargs),
    )

    HoopVisionNode.active_callback(harness, Bool(data=False))
    assert harness.active is False
    assert list(harness.history) == []
    assert harness.last_detection is None
    assert len(published) == 1
    assert (harness.color_sub, harness.depth_sub, harness.sync) == (
        color_sub,
        depth_sub,
        sync,
    )

    HoopVisionNode.active_callback(harness, Bool(data=True))
    assert harness.active is True
    assert len(published) == 1
    assert (harness.color_sub, harness.depth_sub, harness.sync) == (
        color_sub,
        depth_sub,
        sync,
    )


def test_ball_in_hand_coordinates_modes_inside_vision_off_before_on():
    events = []
    logger = _Logger()
    harness = SimpleNamespace(
        manage_activity_from_ball_in_hand=True,
        managed_hoop_active=False,
        ball_detection_active=True,
        pub_ball_active=_Publisher("ball", events),
        pub_hoop_active=_Publisher("hoop", events),
        get_logger=lambda: logger,
    )

    def set_ball_processing(msg):
        harness.ball_detection_active = bool(msg.data)

    harness.cb_ball_active = set_ball_processing

    changed = BallVisionFusionNode._set_vision_mode_from_ball_in_hand(
        harness,
        True,
    )

    assert changed is True
    assert harness.ball_detection_active is False
    assert harness.managed_hoop_active is True
    assert events == [("ball", False), ("hoop", True)]

    events.clear()
    changed = BallVisionFusionNode._set_vision_mode_from_ball_in_hand(
        harness,
        True,
    )
    assert changed is False
    assert events == []

    changed = BallVisionFusionNode._set_vision_mode_from_ball_in_hand(
        harness,
        False,
    )
    assert changed is True
    assert harness.ball_detection_active is True
    assert events == [("hoop", False), ("ball", True)]
