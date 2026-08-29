from collections import deque
from types import SimpleNamespace

from decision.main_decision import MainDecision, Motion


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def _startup_gate_harness():
    logger = _Logger()
    harness = SimpleNamespace(
        test_mode=False,
        webcam_yolo_ready=False,
        realsense_yolo_ready=False,
        line_data=True,
        ball_data=True,
        hurdle_data=True,
        line_buffer=deque([1, 1, 1], maxlen=5),
        line_vote_detail_buffer=deque([{"status": 1}], maxlen=5),
        ball_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_ready_buffer=deque([False, False, False], maxlen=5),
        get_logger=lambda: logger,
    )
    return harness, logger


def test_startup_gate_waits_for_both_yolo_first_inferences():
    harness, logger = _startup_gate_harness()

    assert MainDecision._vision_stack_ready(harness) is False

    MainDecision._set_yolo_readiness(harness, "webcam", True)
    assert harness.webcam_yolo_ready is True
    assert harness.realsense_yolo_ready is False
    assert MainDecision._vision_stack_ready(harness) is False
    assert list(harness.line_buffer) == []

    # 한쪽만 준비된 동안 들어온 값도 두 번째 YOLO가 준비되는 순간 버린다.
    harness.line_buffer.extend([2, 2, 2])
    harness.ball_buffer.extend([99, 99, 99])
    harness.hurdle_buffer.extend([99, 99, 99])
    MainDecision._set_yolo_readiness(harness, "realsense", True)

    assert MainDecision._vision_stack_ready(harness) is True
    assert list(harness.line_buffer) == []
    assert list(harness.ball_buffer) == []
    assert list(harness.hurdle_buffer) == []
    assert any("새 비전 프레임 3개" in message for message in logger.messages)


def test_motion_command_is_blocked_before_both_yolo_are_ready():
    harness, logger = _startup_gate_harness()
    harness.motion_ready = True
    harness.status = Motion.Forward_4step

    MainDecision.MotionCommand(harness)

    assert any("vision_startup=false" in message for message in logger.messages)


class _Publisher:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, message):
        self.events.append((self.name, bool(message.data)))


def _activity_harness(
    ball_active=True,
    hoop_active=False,
    webcam_ball_allowed=True,
):
    events = []
    logger = _Logger()
    harness = SimpleNamespace(
        ball_vision_active=ball_active,
        hoop_vision_active=hoop_active,
        webcam_ball_allowed=webcam_ball_allowed,
        ball_active_pub=_Publisher("ball", events),
        hoop_active_pub=_Publisher("hoop", events),
        webcam_ball_allowed_pub=_Publisher("webcam", events),
        get_logger=lambda: logger,
        ball_data=True,
        ball_buffer=deque([12, 12, 12], maxlen=5),
        post_pick_failure_ball_suppressed=False,
        current_mode="BallMode",
        line_data=True,
        hurdle_data=True,
        line_buffer=deque([1, 1, 1], maxlen=5),
        hurdle_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_ready_buffer=deque([False, False, False], maxlen=5),
        post_shoot_detection_suppressed=False,
    )
    harness._finish_turn_after_shoot = lambda reason: (
        MainDecision._finish_turn_after_shoot(harness, reason)
    )
    harness._reset_vision_decision_cycle = lambda: (
        MainDecision._reset_vision_decision_cycle(harness)
    )
    harness._finish_post_shoot_detection_suppression = lambda reason: (
        MainDecision._finish_post_shoot_detection_suppression(
            harness,
            reason,
        )
    )
    harness._set_webcam_ball_allowed = lambda allowed, reason='': (
        MainDecision._set_webcam_ball_allowed(
            harness,
            allowed,
            reason,
        )
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
    assert success_events == [
        ("webcam", False),
        ("ball", False),
        ("hoop", True),
    ]

    failed, failed_events, _ = _activity_harness()
    failed.pick_done = True
    failed.ball_in_hand = False
    failed.has_ball = False

    assert MainDecision.CheckBall(failed) is False
    assert failed_events == [("webcam", False)]
    assert failed.post_pick_failure_ball_suppressed is True
    assert list(failed.ball_buffer) == []
    assert failed.ball_vision_active is True
    assert failed.hoop_vision_active is False
    assert failed.webcam_ball_allowed is False


def test_failed_pick_rearms_webcam_distance_gate_after_recovery():
    harness, events, _ = _activity_harness(
        ball_active=True,
        hoop_active=False,
    )
    harness.pick_done = True
    harness.ball_in_hand = False
    harness.has_ball = False

    assert MainDecision.CheckBall(harness) is False
    assert events == [("webcam", False)]
    assert harness.ball_vision_active is True
    assert harness.webcam_ball_allowed is False

    # 복귀 중에는 웹캠 공 결과를 막고 RealSense 공 검출은 유지한다.
    harness.ball_buffer.extend([99, 99, 99])
    MainDecision._finish_post_pick_failure_recovery(
        harness,
        "Back_To_Walk completed",
    )
    assert events == [("webcam", False), ("webcam", True)]
    assert harness.post_pick_failure_ball_suppressed is False
    assert list(harness.ball_buffer) == []
    assert harness.current_mode == "LineTrackingMode"
    assert harness.webcam_ball_allowed is True


def test_failed_pick_turn_limit_restores_original_lost_mode_branch():
    commands = []
    harness, events, _ = _activity_harness(
        ball_active=True,
        hoop_active=False,
        webcam_ball_allowed=False,
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
    assert events == [("webcam", True)]
    assert harness.back_to_walk_after_pick is False
    assert harness.turn_after_pick is False
    assert harness.pick_try_count == 0
    assert harness.post_pick_failure_ball_suppressed is False


def test_shoot_completion_keeps_ball_detection_disabled():
    harness, events, logger = _activity_harness(
        ball_active=False,
        hoop_active=True,
        webcam_ball_allowed=False,
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


def test_post_shoot_return_immediately_uses_cached_line_after_back_to_walk():
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=False,
        webcam_ball_allowed=False,
    )
    harness.turn_after_shoot = True
    harness.post_shoot_detection_suppressed = True
    harness.back_to_walk_after_shoot = False
    harness.turn_count = 0
    harness.post_shoot_min_turn_count = 4
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

    # 라인이 계속 보여도 Shoot 이후 회전을 최소 4회 수행한다.
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
    assert list(harness.ball_buffer) == []
    assert events == [("ball", True), ("webcam", True)]
    assert harness.ball_vision_active is True
    assert harness.hoop_vision_active is False
    assert harness.line_tracking_calls == 0

    # Back_To_Walk 완료 판단에 사용한 직전 라인 결과로 즉시 복귀한다.
    MainDecision.BallMode(harness)

    assert harness.back_to_walk_after_shoot is False
    assert harness.current_mode == "LineTrackingMode"
    assert list(harness.line_buffer) == [1, 1, 1]
    assert list(harness.ball_buffer) == []
    assert list(harness.hurdle_buffer) == [99, 99, 99]
    assert events == [("ball", True), ("webcam", True)]
    assert harness.ball_vision_active is True
    assert harness.line_tracking_calls == 1


def test_post_pick_return_immediately_uses_cached_line_after_back_to_walk():
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=True,
    )
    harness.back_to_walk_after_pick = True
    harness.line_status = Motion.Left_Half_Forward
    harness.line_tracking_statuses = []
    harness.LineTracking = lambda: harness.line_tracking_statuses.append(
        harness.line_status
    )

    MainDecision.BallMode(harness)

    assert harness.back_to_walk_after_pick is False
    assert harness.current_mode == "LineTrackingMode"
    assert harness.line_tracking_statuses == [Motion.Left_Half_Forward]
    assert list(harness.line_buffer) == [1, 1, 1]
    assert list(harness.ball_buffer) == [12, 12, 12]
    assert list(harness.hurdle_buffer) == [99, 99, 99]
    assert events == []


def test_post_shoot_turn_allows_tenth_rotation_before_lost_mode():
    harness, events, _ = _activity_harness(
        ball_active=False,
        hoop_active=False,
        webcam_ball_allowed=False,
    )
    harness.turn_after_shoot = True
    harness.post_shoot_detection_suppressed = True
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
    assert events == [("ball", True), ("webcam", True)]
    assert harness.post_shoot_detection_suppressed is False
