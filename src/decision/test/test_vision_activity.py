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
        ball_data=True,
        ball_buffer=deque([12, 12, 12], maxlen=5),
        ball_tracking_active=True,
        ball_last_seen_time=9.0,
        ball_loss_waiting=False,
        post_pick_failure_ball_suppressed=False,
        current_mode="BallMode",
        line_data=True,
        hurdle_data=True,
        line_buffer=deque([1, 1, 1], maxlen=5),
        hurdle_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_ready_buffer=deque([False, False, False], maxlen=5),
    )
    harness._now_seconds = lambda: 10.0
    harness._reset_goal_loss_state = lambda: (
        MainDecision._reset_goal_loss_state(harness)
    )
    harness._finish_turn_after_shoot = lambda reason: (
        MainDecision._finish_turn_after_shoot(harness, reason)
    )
    harness._reset_vision_decision_cycle = lambda: (
        MainDecision._reset_vision_decision_cycle(harness)
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
    assert failed_events == [("hoop", False)]
    assert failed.post_pick_failure_ball_suppressed is True
    assert list(failed.ball_buffer) == []
    assert failed.ball_vision_active is False
    assert failed.hoop_vision_active is False


def test_failed_pick_reenables_ball_immediately_after_recovery():
    harness, events, _ = _activity_harness(
        ball_active=True,
        hoop_active=False,
    )
    harness.pick_done = True
    harness.ball_in_hand = False
    harness.has_ball = False

    assert MainDecision.CheckBall(harness) is False
    assert events == [("ball", False)]

    # Back_To_Walk 완료 직전까지 쌓인 OFF 상태 값도 모두 버린다.
    harness.ball_buffer.extend([99, 99, 99])
    MainDecision._finish_post_pick_failure_recovery(
        harness,
        "Back_To_Walk completed",
    )
    assert events == [("ball", False), ("ball", True)]
    assert harness.post_pick_failure_ball_suppressed is False
    assert list(harness.ball_buffer) == []
    assert harness.current_mode == "LineTrackingMode"


def test_failed_pick_turn_limit_restores_original_lost_mode_branch():
    commands = []
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=False,
    )
    harness.turn_count = 10
    harness.turn_after_pick = True
    harness.backward_after_pick = False
    harness.back_to_walk_after_pick = False
    harness.pick_try_count = 2
    harness.post_pick_failure_ball_suppressed = True
    harness.line_status = 99
    harness.status = None
    harness.MotionCommand = lambda: commands.append(harness.status)
    harness.LostMode = lambda: commands.append("lost")

    MainDecision.TurnAfterPick(harness)

    assert commands == ["lost"]
    assert Motion.Back_To_Walk not in commands
    assert events == [("ball", True)]
    assert harness.back_to_walk_after_pick is False
    assert harness.turn_after_pick is False
    assert harness.pick_try_count == 0
    assert harness.post_pick_failure_ball_suppressed is False


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


def test_post_shoot_return_uses_fresh_frames_after_back_to_walk():
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=False,
    )
    harness.turn_after_shoot = True
    harness.back_to_walk_after_shoot = False
    harness.turn_count = 0
    harness.post_shoot_min_turn_count = 3
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

    # 라인이 계속 보여도 Shoot 이후 회전을 최소 3회 수행한다.
    harness.line_status = Motion.Forward_4step
    for _ in range(harness.post_shoot_min_turn_count - 1):
        MainDecision.TurnAfterShoot(harness)

    expected_turns = [
        Motion.Right_Turn_Afterpick,
    ] * harness.post_shoot_min_turn_count
    assert harness.commands == expected_turns
    assert harness.turn_count == harness.post_shoot_min_turn_count
    assert harness.back_to_walk_after_shoot is False

    # 최소 회전 이후 라인이 보이면 라인트래킹 명령을 바로 보내지 않고
    # 보행 자세로 먼저 복귀한다.
    MainDecision.TurnAfterShoot(harness)

    assert harness.turn_after_shoot is False
    assert harness.back_to_walk_after_shoot is True
    assert harness.turn_count == 0
    assert harness.commands == expected_turns + [Motion.Back_To_Walk]
    assert list(harness.ball_buffer) == [32, 32, 32]
    assert events == []
    assert harness.ball_vision_active is False
    assert harness.hoop_vision_active is False
    assert harness.line_tracking_calls == 0

    # Back_To_Walk 완료 판단에 사용된 모션 중 결과는 모두 버린다.
    MainDecision.BallMode(harness)

    assert harness.back_to_walk_after_shoot is False
    assert harness.current_mode == "LineTrackingMode"
    assert list(harness.line_buffer) == []
    assert list(harness.ball_buffer) == []
    assert list(harness.hurdle_buffer) == []
    assert events == [("ball", True)]
    assert harness.ball_vision_active is True
    assert harness.line_tracking_calls == 0

    # motion_end 이후 새 결과가 3개 모이기 전에는 라인 명령을 보내지 않는다.
    harness.motion_ready = True
    harness.motion_end = True
    harness.hurdle_detected = False
    harness.latest_line_angle = 0.0
    harness.latest_line_follow_point = False
    harness.latest_ball_angle = 0.0
    harness.latest_ball_in_hand = False
    harness.latest_hurdle_angle = 0.0
    harness.Decision = lambda: harness.LineTracking()

    for _ in range(2):
        harness.line_buffer.append(Motion.Right_Half_Forward)
        harness.ball_buffer.append(99)
        harness.hurdle_buffer.append(99)
        harness.hurdle_ready_buffer.append(False)
        assert MainDecision._try_decision_from_cached_results(harness) is False

    harness.line_buffer.append(Motion.Right_Half_Forward)
    harness.ball_buffer.append(99)
    harness.hurdle_buffer.append(99)
    harness.hurdle_ready_buffer.append(False)

    assert MainDecision._try_decision_from_cached_results(harness) is True
    assert harness.line_status == Motion.Right_Half_Forward
    assert harness.line_tracking_calls == 1


def test_post_shoot_turn_allows_tenth_rotation_before_lost_mode():
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=False,
    )
    harness.turn_after_shoot = True
    harness.back_to_walk_after_shoot = False
    harness.turn_count = 9
    harness.goal_count = 1
    harness.turn_shoot = Motion.Right_Turn_Afterpick
    harness.line_status = 99
    harness.commands = []
    harness.lost_mode_calls = 0
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness.LostMode = lambda: setattr(
        harness,
        "lost_mode_calls",
        harness.lost_mode_calls + 1,
    )

    MainDecision.TurnAfterShoot(harness)

    assert harness.commands == [Motion.Right_Turn_Afterpick]
    assert harness.turn_count == 10
    assert harness.lost_mode_calls == 0

    MainDecision.TurnAfterShoot(harness)

    assert harness.turn_after_shoot is False
    assert harness.turn_count == 0
    assert harness.lost_mode_calls == 1
    assert events == [("ball", True)]
