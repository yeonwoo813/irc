import json
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

from std_msgs.msg import Bool, String


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from ball_vision_fusion import BallVisionFusionNode  # noqa: E402


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


class _Publisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(bool(message.data))


def test_inactive_ball_callbacks_skip_yolo_state_processing():
    harness = SimpleNamespace(ball_detection_active=False)

    BallVisionFusionNode.cb_realsense_yolo_state(harness, object())
    BallVisionFusionNode.cb_webcam_state(harness, object())


def test_ball_activity_switch_only_clears_latest_state():
    logger = _Logger()
    enabled_values = []
    harness = SimpleNamespace(
        ball_detection_active=True,
        latest_realsense={"realsense_ball_detected": True},
        latest_realsense_time=1.0,
        latest_webcam={"webcam_ball_detected": True},
        latest_webcam_time=1.0,
        get_logger=lambda: logger,
    )
    harness._clear_ball_detection_state = MethodType(
        BallVisionFusionNode._clear_ball_detection_state, harness
    )
    harness.ball_status_publisher = SimpleNamespace(
        set_detection_enabled=lambda enabled: enabled_values.append(enabled)
    )

    BallVisionFusionNode.cb_ball_active(harness, Bool(data=False))
    assert harness.ball_detection_active is False
    assert harness.latest_realsense is None
    assert harness.latest_webcam is None
    assert enabled_values == [False]

    BallVisionFusionNode.cb_ball_active(harness, Bool(data=True))
    assert harness.ball_detection_active is True
    assert enabled_values == [False, True]


def test_realsense_yolo_state_is_accepted_without_legacy_hsv_vote():
    harness = SimpleNamespace(
        ball_detection_active=True,
        latest_realsense=None,
        latest_realsense_time=0.0,
        get_logger=lambda: _Logger(),
        _empty_realsense_state=BallVisionFusionNode._empty_realsense_state,
    )
    payload = {
        "realsense_ball_detected": True,
        "realsense_ball_distance_cm": 91.5,
        "realsense_ball_angle_error": -4.0,
        "raw_ball_conf": 0.88,
        "source": "realsense_yolo",
    }

    BallVisionFusionNode.cb_realsense_yolo_state(
        harness, String(data=json.dumps(payload))
    )

    assert harness.latest_realsense["realsense_ball_detected"] is True
    assert harness.latest_realsense["realsense_ball_distance_cm"] == 91.5
    assert harness.latest_realsense["raw_ball_conf"] == 0.88
    assert harness.latest_realsense_time > 0.0


def test_fusion_does_not_override_detector_activity_from_ball_in_hand():
    # 검출기 ON/OFF의 단일 제어자는 MainDecision이다. Fusion이
    # ball_in_hand 변화로 /vision/ball_active를 다시 켜면 Shoot 후
    # 검출금지 구간을 깨므로 자동 전환 메서드 자체가 없어야 한다.
    assert not hasattr(
        BallVisionFusionNode,
        "_set_vision_mode_from_ball_in_hand",
    )
