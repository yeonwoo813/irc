from collections import deque
from types import SimpleNamespace

from decision.main_decision import MainDecision, Motion


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _Publisher:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, message):
        self.events.append((self.name, bool(message.data)))


def _activity_harness(ball_active=True, hoop_active=False):
    events = []
    logger = _Logger()
    harness = SimpleNamespace(
        ball_vision_active=ball_active,
        hoop_vision_active=hoop_active,
        ball_active_pub=_Publisher("ball", events),
        hoop_active_pub=_Publisher("hoop", events),
        get_logger=lambda: logger,
        goal_last_seen_time=None,
        goal_loss_waiting=False,
    )
    harness._now_seconds = lambda: 10.0
    harness._reset_goal_loss_state = lambda: (
        MainDecision._reset_goal_loss_state(harness)
    )
    harness._finish_turn_after_shoot = lambda reason: (
        MainDecision._finish_turn_after_shoot(harness, reason)
    )
    return harness, events, logger


def test_ball_to_hoop_transition_disables_ball_first():
    harness, events, _ = _activity_harness()

    changed = MainDecision._set_vision_activity(
        harness,
        ball_active=False,
        hoop_active=True,
        reason="test",
    )

    assert changed is True
    assert events == [("ball", False), ("hoop", True)]
    assert harness.ball_vision_active is False
    assert harness.hoop_vision_active is True


def test_repeated_activity_state_is_not_republished():
    harness, events, _ = _activity_harness()

    changed = MainDecision._set_vision_activity(
        harness,
        ball_active=True,
        hoop_active=False,
    )

    assert changed is False
    assert events == []


def test_pick_result_selects_the_matching_detector():
    success, success_events, _ = _activity_harness()
    success.pick_done = True
    success.ball_in_hand = True
    success.has_ball = False

    assert MainDecision.CheckBall(success) is True
    assert success_events == [("ball", False), ("hoop", True)]

    failed, failed_events, _ = _activity_harness(
        ball_active=False,
        hoop_active=True,
    )
    failed.pick_done = True
    failed.ball_in_hand = False
    failed.has_ball = False

    assert MainDecision.CheckBall(failed) is False
    assert failed_events == [("hoop", False), ("ball", True)]


def test_shoot_completion_keeps_ball_detection_disabled():
    harness, events, logger = _activity_harness(
        ball_active=False,
        hoop_active=True,
    )
    harness.test_mode = False
    harness.motion_ready = True
    harness.motion_end = True
    harness.hurdle_go_active = False
    harness.shoot_in_progress = True
    harness.shoot_motion_started = False
    harness._try_decision_from_cached_results = lambda: None

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=False),
    )
    assert harness.shoot_motion_started is True
    assert events == []

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )
    assert harness.shoot_in_progress is False
    assert harness.shoot_motion_started is False
    assert events == [("hoop", False)]
    assert harness.ball_vision_active is False
    assert harness.hoop_vision_active is False
    assert any(
        "wait for post-shoot turn" in message
        for message in logger.messages
    )


def test_post_shoot_turn_completion_clears_buffer_then_enables_ball():
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=False,
    )
    harness.turn_after_shoot = True
    harness.turn_count = 0
    harness.goal_count = 0
    harness.line_status = 99
    harness.ball_data = True
    harness.ball_buffer = deque([32, 32, 32], maxlen=5)
    harness.commands = []
    harness.line_tracking_calls = 0
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness.LineTracking = lambda: setattr(
        harness,
        "line_tracking_calls",
        harness.line_tracking_calls + 1,
    )

    # 첫 호출에서는 강제회전만 실행하고 공 검출기는 계속 꺼 둔다.
    MainDecision.TurnAfterShoot(harness)
    assert harness.commands == [Motion.Right_Turn_Afterpick]
    assert harness.turn_count == 1
    assert events == []
    assert list(harness.ball_buffer) == [32, 32, 32]

    # 회전 후 라인을 찾은 시점에 기존 결과를 지운 뒤 공 검출을 켠다.
    harness.line_status = Motion.Forward_4step
    MainDecision.TurnAfterShoot(harness)

    assert harness.turn_after_shoot is False
    assert harness.turn_count == 0
    assert harness.ball_data is False
    assert list(harness.ball_buffer) == []
    assert events == [("ball", True)]
    assert harness.ball_vision_active is True
    assert harness.hoop_vision_active is False
    assert harness.line_tracking_calls == 1
