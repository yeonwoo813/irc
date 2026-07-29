from collections import deque
from types import MethodType, SimpleNamespace

from decision.main_decision import MainDecision


class _Logger:

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def _make_harness():
    harness = SimpleNamespace(
        test_mode=False,
        motion_ready=True,
        motion_end=False,
        line_data=False,
        ball_data=False,
        hurdle_data=False,
        line_buffer=deque([1, 2, 2, 2, 3], maxlen=5),
        ball_buffer=deque([99, 99, 12, 99, 12], maxlen=5),
        hurdle_buffer=deque([99, 26, 99, 99, 26], maxlen=5),
        latest_line_angle=11.5,
        latest_line_follow_point=False,
        latest_ball_angle=-7.25,
        latest_ball_in_hand=True,
        latest_hurdle_angle=3.5,
        latest_hurdle_ready=True,
        hurdle_count=0,
        hurdle_detected=False,
        hurdle_go_active=False,
        hurdle_go_started=False,
    )
    harness.logger = _Logger()
    harness.decision_count = 0
    harness.get_logger = lambda: harness.logger

    def record_decision():
        harness.decision_count += 1

    harness.Decision = record_decision
    harness._try_decision_from_cached_results = MethodType(
        MainDecision._try_decision_from_cached_results,
        harness,
    )
    return harness


def test_motion_end_immediately_decides_from_cached_results():
    harness = _make_harness()
    motion_end_msg = SimpleNamespace(motion_ready=True, motion_end=True)

    MainDecision.MotionEndCallback(harness, motion_end_msg)

    assert harness.decision_count == 1
    assert harness.line_status == 2
    assert harness.line_follow_point is False
    assert harness.ball_status == 99
    assert harness.hurdle_status == 99
    assert harness.angle == 11.5
    assert harness.ball_angle == -7.25
    assert harness.ball_in_hand is True
    assert harness.hurdle_angle == 3.5
    assert harness.hurdle_ready is True


def test_motion_end_waits_when_cached_results_are_insufficient():
    harness = _make_harness()
    harness.line_buffer = deque([1, 2], maxlen=5)
    motion_end_msg = SimpleNamespace(motion_ready=True, motion_end=True)

    MainDecision.MotionEndCallback(harness, motion_end_msg)

    assert harness.decision_count == 0
    assert harness.line_data is False
    assert any(
        "저장된 비전 데이터 부족" in message
        for message in harness.logger.messages
    )


def test_callbacks_store_latest_flags_before_motion_is_ready():
    harness = _make_harness()
    harness.motion_ready = False
    line_msg = SimpleNamespace(
        status=3,
        angle=8.0,
        follow_point=True,
    )
    ball_msg = SimpleNamespace(
        status=12,
        angle=4.0,
        ball_in_hand=False,
    )
    hurdle_msg = SimpleNamespace(
        status=26,
        angle=-2.0,
        hurdle_ready=False,
    )

    MainDecision.LineResultCallback(harness, line_msg)
    MainDecision.BallResultCallback(harness, ball_msg)
    MainDecision.HurdleResultCallback(harness, hurdle_msg)

    assert harness.latest_line_angle == 8.0
    assert harness.latest_line_follow_point is True
    assert harness.latest_ball_angle == 4.0
    assert harness.latest_ball_in_hand is False
    assert harness.latest_hurdle_angle == -2.0
    assert harness.latest_hurdle_ready is False
    assert list(harness.line_buffer) == [1, 2, 2, 2, 3]
    assert list(harness.ball_buffer) == [99, 99, 12, 99, 12]
    assert list(harness.hurdle_buffer) == [99, 26, 99, 99, 26]


def test_hurdle_detection_stays_latched_until_hurdle_go_finishes():
    harness = _make_harness()
    hurdle_msg = SimpleNamespace(
        status=27,
        angle=0.0,
        hurdle_ready=False,
    )

    MainDecision.HurdleResultCallback(harness, hurdle_msg)

    assert harness.hurdle_detected is True

    harness.hurdle_go_active = True
    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=False),
    )
    assert harness.hurdle_detected is True
    assert harness.hurdle_go_started is True

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )
    assert harness.hurdle_detected is False
    assert harness.hurdle_go_active is False


def test_latched_hurdle_mode_cannot_be_preempted_by_ball_detection():
    harness = SimpleNamespace(
        motion_ready=True,
        line_data=True,
        ball_data=True,
        hurdle_data=True,
        hurdle_detected=True,
        hurdle_count=0,
        hurdle_step=0,
        hurdle_ready=False,
        hurdle_status=27,
        pick_done=False,
        turn_after_pick=False,
        turn_after_shoot=False,
        ball_status=12,
        lost_count=0,
        lost_step=0,
        lost_found_dir=0,
        lost_body_turn_count=0,
        line_status=1,
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.selected_mode = None
    harness.HurdleMode = lambda: setattr(
        harness,
        "selected_mode",
        "hurdle",
    )
    harness.BallMode = lambda: setattr(
        harness,
        "selected_mode",
        "ball",
    )
    harness.LostMode = lambda: setattr(
        harness,
        "selected_mode",
        "lost",
    )
    harness.LineTracking = lambda: setattr(
        harness,
        "selected_mode",
        "line",
    )

    MainDecision.Decision(harness)

    assert harness.selected_mode == "hurdle"
