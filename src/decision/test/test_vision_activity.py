from types import SimpleNamespace

from decision.main_decision import MainDecision


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


def test_shoot_completion_reenables_ball_detection():
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
    assert events == [("hoop", False), ("ball", True)]
    assert any(
        "shoot motion completed" in message
        for message in logger.messages
    )
