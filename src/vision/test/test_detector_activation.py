from collections import deque
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

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


def test_inactive_ball_callbacks_skip_image_and_webcam_processing():
    harness = SimpleNamespace(ball_detection_active=False)

    BallVisionFusionNode.cb_realsense_images(harness, object(), object())
    BallVisionFusionNode.cb_webcam_state(harness, object())


def test_ball_activity_switch_clears_state_and_controls_subscriptions():
    logger = _Logger()
    counts = {"start": 0, "stop": 0, "reset_vote": 0}
    harness = SimpleNamespace(
        ball_detection_active=True,
        latest_realsense={"realsense_ball_detected": True},
        latest_realsense_time=1.0,
        last_realsense_detection={"realsense_ball_detected": True},
        realsense_lost_frames=0,
        realsense_hold_frames=3,
        latest_webcam={"webcam_ball_detected": True},
        latest_webcam_time=1.0,
        get_logger=lambda: logger,
    )
    harness._clear_ball_detection_state = MethodType(
        BallVisionFusionNode._clear_ball_detection_state,
        harness,
    )
    harness._start_ball_image_subscriptions = lambda: counts.__setitem__(
        "start", counts["start"] + 1
    )
    harness._stop_ball_image_subscriptions = lambda: counts.__setitem__(
        "stop", counts["stop"] + 1
    )
    harness.ball_status_publisher = SimpleNamespace(
        _reset_webcam_detection_cycle=lambda: counts.__setitem__(
            "reset_vote", counts["reset_vote"] + 1
        )
    )

    BallVisionFusionNode.cb_ball_active(harness, Bool(data=False))
    assert harness.ball_detection_active is False
    assert harness.latest_realsense is None
    assert harness.latest_webcam is None
    assert counts == {"start": 0, "stop": 1, "reset_vote": 0}

    BallVisionFusionNode.cb_ball_active(harness, Bool(data=True))
    assert harness.ball_detection_active is True
    assert counts == {"start": 1, "stop": 1, "reset_vote": 1}


def test_hoop_activity_switch_clears_history_and_controls_subscriptions():
    logger = _Logger()
    counts = {"start": 0, "stop": 0, "publish": 0}
    harness = SimpleNamespace(
        active=True,
        history=deque([{"detected": True}], maxlen=5),
        last_detection={"detected": True},
        last_detection_time=1.0,
        get_logger=lambda: logger,
    )
    harness._start_image_subscriptions = lambda: counts.__setitem__(
        "start", counts["start"] + 1
    )
    harness._stop_image_subscriptions = lambda: counts.__setitem__(
        "stop", counts["stop"] + 1
    )
    harness._publish_state = lambda **_kwargs: counts.__setitem__(
        "publish", counts["publish"] + 1
    )

    HoopVisionNode.active_callback(harness, Bool(data=False))
    assert harness.active is False
    assert list(harness.history) == []
    assert harness.last_detection is None
    assert counts == {"start": 0, "stop": 1, "publish": 1}

    HoopVisionNode.active_callback(harness, Bool(data=True))
    assert harness.active is True
    assert counts == {"start": 1, "stop": 1, "publish": 1}
