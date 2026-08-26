from collections import deque
from types import MethodType, SimpleNamespace

from decision.main_decision import Ball, MainDecision, Motion


class _Logger:

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def _make_harness():
    clock = SimpleNamespace(now=10.0)
    harness = SimpleNamespace(
        test_mode=False,
        motion_ready=True,
        motion_end=False,
        current_mode="LineTrackingMode",
        line_data=False,
        ball_data=False,
        hurdle_data=False,
        line_buffer=deque([1, 2, 2, 2, 3], maxlen=5),
        ball_buffer=deque([99, 99, 12, 99, 12], maxlen=5),
        hurdle_buffer=deque([99, 26, 99, 99, 26], maxlen=5),
        hurdle_ready_buffer=deque([False, True, False, True, True], maxlen=5),
        latest_line_angle=11.5,
        latest_line_follow_point=False,
        latest_ball_angle=-7.25,
        latest_ball_in_hand=True,
        latest_goal_distance_cm=0.0,
        latest_hurdle_angle=3.5,
        latest_hurdle_ready=True,
        hurdle_count=0,
        hurdle_detected=False,
        hurdle_go_active=False,
        hurdle_go_started=False,
        shoot_fresh_vision_active=False,
        shoot_fresh_vision_armed=True,
        shoot_fresh_vision_distance_cm=80.0,
        shoot_fresh_vision_settle_sec=0.5,
        shoot_fresh_vision_settle_until=0.0,
        goal_lost_timeout_sec=0.5,
        goal_last_seen_time=None,
        goal_loss_waiting=False,
    )
    harness.logger = _Logger()
    harness.clock = clock
    harness.decision_count = 0
    harness.get_logger = lambda: harness.logger
    harness._now_seconds = lambda: harness.clock.now
    harness._reset_goal_loss_state = MethodType(
        MainDecision._reset_goal_loss_state,
        harness,
    )

    def record_decision():
        harness.decision_count += 1

    harness.Decision = record_decision
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness._is_before_pick = MethodType(
        MainDecision._is_before_pick,
        harness,
    )
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


def test_line_status_uses_mode_of_latest_three_samples():
    harness = _make_harness()
    harness.motion_end = True
    harness.line_buffer = deque([1, 1, 2, 3, 2], maxlen=5)

    harness._try_decision_from_cached_results()

    assert harness.line_status == 2


def test_line_status_uses_latest_when_latest_three_are_all_different():
    harness = _make_harness()
    harness.motion_end = True
    harness.line_buffer = deque([1, 1, 2, 3, 7], maxlen=5)

    harness._try_decision_from_cached_results()

    assert harness.line_status == 7


def test_goal_at_80cm_clears_only_ball_buffer_at_motion_end():
    harness = _make_harness()
    harness.current_mode = "BallMode"
    harness.has_ball = True

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Ball.Ball_None,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=80.0,
        ),
    )

    assert harness.shoot_fresh_vision_active is True

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )

    assert harness.decision_count == 0
    assert list(harness.ball_buffer) == []
    assert harness.shoot_fresh_vision_settle_until == 10.5
    assert list(harness.line_buffer) == [1, 2, 2, 2, 3]
    assert list(harness.hurdle_buffer) == [99, 26, 99, 99, 26]


def test_goal_at_80cm_waits_for_settle_then_three_fresh_results():
    harness = _make_harness()
    harness.current_mode = "BallMode"
    harness.has_ball = True
    harness.shoot_fresh_vision_active = True

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )

    message = SimpleNamespace(
        status=Ball.Ball_None,
        angle=0.0,
        ball_in_hand=True,
        goal_distance_cm=65.0,
    )
    MainDecision.BallResultCallback(harness, message)
    MainDecision.BallResultCallback(harness, message)
    MainDecision.BallResultCallback(harness, message)
    assert harness.decision_count == 0
    assert list(harness.ball_buffer) == []

    harness.clock.now = 10.5
    MainDecision.BallResultCallback(harness, message)
    MainDecision.BallResultCallback(harness, message)
    assert harness.decision_count == 0
    MainDecision.BallResultCallback(harness, message)
    assert harness.decision_count == 1


def test_shoot_fresh_vision_uses_80cm_threshold():
    harness = _make_harness()
    harness.motion_ready = False
    harness.has_ball = True

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Ball.Ball_None,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=80.1,
        ),
    )
    assert harness.shoot_fresh_vision_active is False

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Ball.Ball_None,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=80.0,
        ),
    )
    assert harness.shoot_fresh_vision_active is True


def test_fresh_vision_activation_while_stopped_starts_settle_immediately():
    harness = _make_harness()
    harness.current_mode = "BallMode"
    harness.motion_end = True
    harness.has_ball = True

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Ball.Ball_None,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=80.0,
        ),
    )

    assert harness.shoot_fresh_vision_active is True
    assert harness.shoot_fresh_vision_settle_until == 10.5
    assert list(harness.ball_buffer) == []
    assert harness.decision_count == 0


def test_ball_mode_uses_mode_of_latest_three_ball_statuses():
    harness = _make_harness()
    harness.motion_end = True
    harness.current_mode = "BallMode"
    harness.ball_buffer = deque([99, 99, 23, 8, 23], maxlen=5)

    harness._try_decision_from_cached_results()

    assert harness.ball_status == 23


def test_ball_mode_uses_latest_for_one_to_one_to_one_without_pick():
    harness = _make_harness()
    harness.motion_end = True
    harness.current_mode = "BallMode"
    harness.ball_buffer = deque([99, 99, 23, 8, 24], maxlen=5)

    assert harness._try_decision_from_cached_results() is True

    assert harness.decision_count == 1
    assert harness.ball_status == 24
    assert list(harness.ball_buffer) == [99, 99, 23, 8, 24]


def test_ball_mode_defers_one_to_one_to_one_when_pick_is_included():
    harness = _make_harness()
    harness.motion_end = True
    harness.current_mode = "BallMode"
    harness.ball_buffer = deque(
        [99, 99, Ball.Pick_Ready, Motion.Right_Turn_Mission_5,
         Motion.Forward_half],
        maxlen=5,
    )

    assert harness._try_decision_from_cached_results() is False

    assert harness.decision_count == 0
    assert harness.ball_data is False
    assert list(harness.ball_buffer) == []
    assert any(
        "[PickVoteDeferred]" in message
        for message in harness.logger.messages
    )


def test_ball_mode_motion_end_uses_cached_samples_when_vote_is_not_tied():
    harness = _make_harness()
    harness.current_mode = "BallMode"

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )

    assert harness.decision_count == 1
    assert harness.ball_status == Ball.Pick_Ready
    assert list(harness.ball_buffer) == [99, 99, 12, 99, 12]


def test_ball_mode_pick_requires_two_of_three_fresh_results():
    harness = _make_harness()
    harness.motion_end = True
    harness.current_mode = "BallMode"
    harness.ball_buffer.clear()

    for status in (Ball.Pick_Ready, Motion.Forward_half):
        harness.ball_buffer.append(status)
        assert harness._try_decision_from_cached_results() is False

    harness.ball_buffer.append(Ball.Pick_Ready)

    assert harness._try_decision_from_cached_results() is True
    assert harness.ball_status == Ball.Pick_Ready
    assert harness.decision_count == 1


def test_ball_mode_single_pick_vote_does_not_override_two_approach_votes():
    harness = _make_harness()
    harness.motion_end = True
    harness.current_mode = "BallMode"
    harness.ball_buffer = deque(
        [Ball.Pick_Ready, Motion.Forward_half, Motion.Forward_half],
        maxlen=5,
    )

    assert harness._try_decision_from_cached_results() is True
    assert harness.ball_status == Motion.Forward_half


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
    assert list(harness.hurdle_ready_buffer) == [False, True, False, True, True]


def test_confirmed_hurdle_result_stays_latched_until_crossing_completes():
    harness = _make_harness()

    MainDecision.HurdleResultCallback(
        harness,
        SimpleNamespace(status=99, angle=0.0, hurdle_ready=False),
    )
    assert harness.hurdle_detected is False

    MainDecision.HurdleResultCallback(
        harness,
        SimpleNamespace(
            status=27,
            angle=0.0,
            hurdle_ready=False,
        ),
    )
    assert harness.hurdle_detected is True

    # 비전 다수결 확정 결과를 받은 뒤에는 후속 상태가 None이어도 유지합니다.
    for _ in range(5):
        MainDecision.HurdleResultCallback(
            harness,
            SimpleNamespace(
                status=99,
                angle=0.0,
                hurdle_ready=False,
            ),
        )
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


def test_hurdle_none_does_not_enter_hurdle_mode():
    harness = _make_harness()

    for _ in range(5):
        MainDecision.HurdleResultCallback(
            harness,
            SimpleNamespace(
                status=99,
                angle=0.0,
                hurdle_ready=False,
            ),
        )

    assert harness.hurdle_detected is False


def test_confirmed_hurdle_ignores_stale_none_in_status_vote():
    harness = _make_harness()
    harness.motion_end = True
    harness.hurdle_detected = True
    harness.hurdle_buffer = deque([99, 99, 99, 99, 27], maxlen=5)
    harness.hurdle_ready_buffer = deque([False] * 5, maxlen=5)

    assert harness._try_decision_from_cached_results() is True
    assert harness.hurdle_status == Motion.Back_To_Initial


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


def _make_goal_grace_harness(now_seconds):
    harness = SimpleNamespace(
        motion_ready=True,
        motion_end=True,
        line_data=True,
        ball_data=True,
        hurdle_data=True,
        hurdle_detected=False,
        has_ball=True,
        pick_done=False,
        turn_after_pick=False,
        back_to_walk_after_pick=False,
        turn_after_shoot=False,
        ball_status=Ball.Ball_None,
        line_status=Motion.Forward_4step,
        lost_step=0,
        goal_last_seen_time=10.0,
        goal_loss_waiting=False,
        goal_lost_timeout_sec=0.5,
        current_mode="BallMode",
        line_buffer=deque([1, 1, 1], maxlen=5),
        ball_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_ready_buffer=deque([False, False, False], maxlen=5),
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness._now_seconds = lambda: now_seconds
    harness._hold_BallMode = lambda: False
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness._reset_goal_loss_state = MethodType(
        MainDecision._reset_goal_loss_state,
        harness,
    )
    harness._reset_vision_decision_cycle = MethodType(
        MainDecision._reset_vision_decision_cycle,
        harness,
    )
    harness._hold_goal_BallMode = MethodType(
        MainDecision._hold_goal_BallMode,
        harness,
    )
    harness.selected_mode = None
    harness.HurdleMode = lambda: setattr(harness, "selected_mode", "hurdle")
    harness.BallMode = lambda: setattr(harness, "selected_mode", "ball")
    harness.LostMode = lambda: setattr(harness, "selected_mode", "lost")
    harness.LineTracking = lambda: setattr(harness, "selected_mode", "line")
    return harness


def test_goal_loss_under_half_second_holds_ball_mode():
    harness = _make_goal_grace_harness(now_seconds=10.499)

    MainDecision.Decision(harness)

    assert harness.selected_mode is None
    assert harness.current_mode == "BallMode"
    assert harness.goal_loss_waiting is True


def test_goal_loss_at_half_second_releases_to_previous_line_logic():
    harness = _make_goal_grace_harness(now_seconds=10.5)

    MainDecision.Decision(harness)

    assert harness.selected_mode == "line"
    assert harness.goal_last_seen_time is None
    assert harness.goal_loss_waiting is False


def test_hurdle_ready_uses_majority_instead_of_latest_frame():
    harness = _make_harness()
    harness.motion_end = True
    harness.hurdle_ready_buffer = deque(
        [True, True, True, True, False],
        maxlen=5,
    )

    assert harness._try_decision_from_cached_results() is True
    assert harness.hurdle_ready is True


def test_hurdle_ready_rejects_single_true_frame_and_tie():
    for ready_samples in (
        [False, False, False, False, True],
        [True, False, True, False],
    ):
        harness = _make_harness()
        harness.motion_end = True
        harness.hurdle_ready_buffer = deque(ready_samples, maxlen=5)

        assert harness._try_decision_from_cached_results() is True
        assert harness.hurdle_ready is False


def test_hurdle_ready_requires_three_true_frames_with_partial_buffer():
    for ready_samples, expected_ready in (
        ([True, True, False], False),
        ([True, True, True], True),
        ([True, True, False, False], False),
        ([True, True, True, False], True),
    ):
        harness = _make_harness()
        harness.motion_end = True
        harness.hurdle_ready_buffer = deque(ready_samples, maxlen=5)

        assert harness._try_decision_from_cached_results() is True
        assert harness.hurdle_ready is expected_ready


def test_hurdle_ready_false_excludes_forward_20_from_status_vote():
    harness = _make_harness()
    harness.motion_end = True
    harness.hurdle_buffer = deque([26, 26, 23, 24, 23], maxlen=5)
    harness.hurdle_ready_buffer = deque(
        [True, False, False, False, False],
        maxlen=5,
    )

    assert harness._try_decision_from_cached_results() is True
    assert harness.hurdle_ready is False
    assert harness.hurdle_status == Motion.Left_Turn_Mission_10


def test_hurdle_ready_false_waits_when_all_statuses_are_forward_20():
    harness = _make_harness()
    harness.motion_end = True
    harness.hurdle_buffer = deque([26, 26, 26, 26, 26], maxlen=5)
    harness.hurdle_ready_buffer = deque(
        [False, False, False, False, False],
        maxlen=5,
    )

    assert harness._try_decision_from_cached_results() is False
    assert harness.decision_count == 0
    assert harness.line_data is False
    assert harness.ball_data is False
    assert harness.hurdle_data is False
    assert any(
        "새 프레임을 기다립니다" in message
        for message in harness.logger.messages
    )


def test_hurdle_mode_runs_forward_20_once_then_hurdle_go():
    harness = SimpleNamespace(
        hurdle_step=0,
        hurdle_ready=True,
        hurdle_status=Motion.Left_Turn_Mission_10,
        hurdle_count=0,
        hurdle_go_active=False,
        hurdle_go_started=False,
    )
    harness.commands = []
    harness.MotionCommand = lambda: harness.commands.append(harness.status)

    MainDecision.HurdleMode(harness)

    assert harness.commands == [Motion.Hurdle_Forward_20]
    assert harness.hurdle_step == 1

    # Forward 20 실행 중 ready가 false로 바뀌어도 다음은 허들 넘기입니다.
    harness.hurdle_ready = False
    harness.hurdle_status = Motion.Right_Turn_Mission_10
    MainDecision.HurdleMode(harness)

    assert harness.commands == [Motion.Hurdle_Forward_20, Motion.Hurdle_Go]
    assert harness.hurdle_step == 0
    assert harness.hurdle_count == 1
    assert harness.hurdle_go_active is True
    assert harness.hurdle_go_started is False


def test_hurdle_mode_does_not_publish_none_as_motion_zero():
    harness = SimpleNamespace(
        hurdle_step=0,
        hurdle_ready=False,
        hurdle_status=99,
        current_mode=None,
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.commands = []
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness._reset_vision_decision_cycle = lambda: None

    MainDecision.HurdleMode(harness)

    assert harness.commands == []
    assert any("status=99" in message for message in harness.logger.messages)


def _make_ball_grace_harness(now_seconds, ball_status=Ball.Ball_None):
    harness = SimpleNamespace(
        motion_ready=True,
        motion_end=True,
        line_data=True,
        ball_data=True,
        hurdle_data=True,
        hurdle_detected=False,
        has_ball=False,
        pick_done=False,
        turn_after_pick=False,
        turn_after_shoot=False,
        pick_try_count=0,
        ball_status=ball_status,
        line_status=Motion.Forward_4step,
        lost_step=0,
        ball_tracking_active=True,
        ball_last_seen_time=10.0,
        ball_loss_grace_waiting=False,
        ball_lost_timeout_sec=0.5,
        current_mode="BallMode",
        line_buffer=deque([1, 1, 1], maxlen=5),
        ball_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_buffer=deque([99, 99, 99], maxlen=5),
        hurdle_ready_buffer=deque([False, False, False], maxlen=5),
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness._now_seconds = lambda: now_seconds
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness._pre_pick_ball_grace_enabled = MethodType(
        MainDecision._pre_pick_ball_grace_enabled,
        harness,
    )
    harness._reset_pre_pick_ball_tracking = MethodType(
        MainDecision._reset_pre_pick_ball_tracking,
        harness,
    )
    harness._reset_vision_decision_cycle = MethodType(
        MainDecision._reset_vision_decision_cycle,
        harness,
    )
    harness._hold_pre_pick_ball_mode_for_grace_period = MethodType(
        MainDecision._hold_pre_pick_ball_mode_for_grace_period,
        harness,
    )
    return harness


def test_pre_pick_ball_loss_under_half_second_holds_ball_mode():
    harness = _make_ball_grace_harness(now_seconds=10.499)

    held = harness._hold_pre_pick_ball_mode_for_grace_period()

    assert held is True
    assert harness.current_mode == "BallMode"
    assert harness.ball_tracking_active is True
    assert harness.ball_loss_grace_waiting is True
    assert list(harness.line_buffer) == []
    assert list(harness.ball_buffer) == []


def test_decision_does_not_start_line_motion_during_ball_loss_grace():
    harness = _make_ball_grace_harness(now_seconds=10.25)
    harness.selected_mode = None
    harness.HurdleMode = lambda: setattr(harness, "selected_mode", "hurdle")
    harness.BallMode = lambda: setattr(harness, "selected_mode", "ball")
    harness.LostMode = lambda: setattr(harness, "selected_mode", "lost")
    harness.LineTracking = lambda: setattr(harness, "selected_mode", "line")

    MainDecision.Decision(harness)

    assert harness.selected_mode is None
    assert harness.current_mode == "BallMode"
    assert harness.ball_loss_grace_waiting is True


def test_pre_pick_ball_loss_at_half_second_releases_ball_mode():
    harness = _make_ball_grace_harness(now_seconds=10.5)

    held = harness._hold_pre_pick_ball_mode_for_grace_period()

    assert held is False
    assert harness.ball_tracking_active is False
    assert harness.ball_last_seen_time is None
    assert harness.ball_loss_grace_waiting is False


def test_ball_lost_status_is_also_treated_as_missing():
    harness = _make_ball_grace_harness(
        now_seconds=10.25,
        ball_status=Ball.Ball_Lost,
    )

    assert harness._hold_pre_pick_ball_mode_for_grace_period() is True


def test_timeout_timer_releases_grace_and_rechecks_cached_vision():
    harness = _make_ball_grace_harness(now_seconds=10.5)
    harness.ball_loss_grace_waiting = True
    harness.recheck_count = 0

    def record_recheck():
        harness.recheck_count += 1

    harness._try_decision_from_cached_results = record_recheck

    MainDecision._check_pre_pick_ball_loss_timeout(harness)

    assert harness.ball_tracking_active is False
    assert harness.ball_loss_grace_waiting is False
    assert harness.line_data is False
    assert harness.ball_data is False
    assert harness.hurdle_data is False
    assert harness.recheck_count == 1


def test_each_pick_resets_then_allows_a_new_pre_pick_grace_cycle():
    harness = SimpleNamespace(
        current_mode="BallMode",
        pick_done=False,
        turn_after_pick=False,
        neck_down_pending=False,
        turn_after_shoot=False,
        has_ball=False,
        ball_status=Ball.Pick_Ready,
        pick_try_count=0,
        ball_tracking_active=True,
        ball_last_seen_time=20.0,
        ball_loss_grace_waiting=True,
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.commands = []
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness._reset_pre_pick_ball_tracking = MethodType(
        MainDecision._reset_pre_pick_ball_tracking,
        harness,
    )
    harness._activate_pre_pick_ball_tracking = MethodType(
        MainDecision._activate_pre_pick_ball_tracking,
        harness,
    )
    harness._now_seconds = lambda: 30.0

    MainDecision.BallMode(harness)

    assert harness.commands == [Motion.Pick]
    assert harness.ball_tracking_active is False
    assert harness.ball_last_seen_time is None

    # 첫 번째 공의 Pick 후 회전/슛 과정이 끝나 두 번째 공 접근이 시작된 상태.
    harness.pick_done = False
    harness.turn_after_pick = False
    harness.turn_after_shoot = False
    harness.pick_try_count = 0
    harness.ball_last_seen_time = 30.0
    harness._activate_pre_pick_ball_tracking()

    assert harness.ball_tracking_active is True
    assert harness.ball_last_seen_time == 30.0


def _make_post_pick_harness(ball_in_hand):
    harness = SimpleNamespace(
        current_mode="BallMode",
        pick_done=True,
        turn_after_pick=False,
        backward_after_pick=False,
        ball_in_hand=ball_in_hand,
        has_ball=False,
        goal_last_seen_time=None,
        goal_loss_waiting=False,
        ball_data=True,
        ball_buffer=deque([12, 12, 12], maxlen=5),
        post_pick_failure_ball_suppressed=False,
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.commands = []
    harness.turn_after_pick_calls = 0
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness._now_seconds = lambda: 20.0
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness._reset_goal_loss_state = MethodType(
        MainDecision._reset_goal_loss_state,
        harness,
    )
    harness.CheckBall = MethodType(MainDecision.CheckBall, harness)

    def record_turn_after_pick():
        harness.turn_after_pick_calls += 1

    harness.TurnAfterPick = record_turn_after_pick
    return harness


def test_successful_pick_runs_neck_up_then_backward_then_turn():
    harness = _make_post_pick_harness(ball_in_hand=True)

    MainDecision.BallMode(harness)
    assert harness.commands == [Motion.Neck_Up]
    assert harness.backward_after_pick is True
    assert harness.goal_last_seen_time == 20.0

    MainDecision.BallMode(harness)
    assert harness.commands == [Motion.Neck_Up, Motion.Backward_half]
    assert harness.backward_after_pick is False

    MainDecision.BallMode(harness)
    assert harness.turn_after_pick_calls == 1


def test_failed_pick_runs_backward_then_turn_without_neck_up():
    harness = _make_post_pick_harness(ball_in_hand=False)

    MainDecision.BallMode(harness)
    assert harness.commands == [Motion.Backward_half]
    assert harness.backward_after_pick is False
    assert harness.goal_last_seen_time is None

    MainDecision.BallMode(harness)
    assert harness.turn_after_pick_calls == 1


def test_confirmed_ball_is_not_checked_again_before_shoot():
    harness = SimpleNamespace(
        current_mode="BallMode",
        pick_done=False,
        turn_after_pick=False,
        back_to_walk_after_pick=False,
        neck_down_pending=False,
        turn_after_shoot=False,
        has_ball=True,
        # 후속 비전 값이 false여도 이미 확정한 has_ball은 유지한다.
        ball_in_hand=False,
        ball_status=Ball.Shoot,
        turn_count=3,
        shoot_fresh_vision_active=True,
        shoot_fresh_vision_armed=True,
        goal_last_seen_time=12.0,
        goal_loss_waiting=True,
    )
    harness.commands = []
    harness.check_ball_calls = 0
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness._reset_goal_loss_state = MethodType(
        MainDecision._reset_goal_loss_state,
        harness,
    )

    def record_check_ball():
        harness.check_ball_calls += 1
        return False

    harness.CheckBall = record_check_ball

    MainDecision.BallMode(harness)

    assert harness.check_ball_calls == 0
    assert harness.commands == [Motion.Shoot]
    assert harness.has_ball is False
    assert harness.neck_down_pending is True
    assert harness.turn_after_shoot is True
    assert harness.turn_count == 0
    assert harness.shoot_fresh_vision_active is False
    assert harness.shoot_fresh_vision_armed is False


def test_shoot_disarms_fresh_vision_until_next_successful_pick():
    harness = SimpleNamespace(
        current_mode="BallMode",
        pick_done=False,
        turn_after_pick=False,
        back_to_walk_after_pick=False,
        backward_after_pick=False,
        neck_down_pending=False,
        turn_after_shoot=False,
        has_ball=True,
        ball_in_hand=True,
        ball_status=Ball.Shoot_Close,
        turn_count=0,
        shoot_fresh_vision_active=True,
        shoot_fresh_vision_armed=True,
        shoot_fresh_vision_distance_cm=80.0,
        shoot_fresh_vision_settle_sec=0.5,
        shoot_fresh_vision_settle_until=0.0,
        goal_lost_timeout_sec=0.5,
        goal_last_seen_time=15.0,
        goal_loss_waiting=True,
        motion_ready=False,
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.commands = []
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness._now_seconds = lambda: 20.0
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness._reset_goal_loss_state = MethodType(
        MainDecision._reset_goal_loss_state,
        harness,
    )

    MainDecision.BallMode(harness)

    assert harness.commands == [Motion.Shoot_Close]
    assert harness.shoot_fresh_vision_active is False
    assert harness.shoot_fresh_vision_armed is False

    # Shoot 직후에는 publisher의 ball_in_hand/goal 값이 잠시 남아도
    # FreshVision이 다시 켜지면 안 된다.
    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Ball.Ball_None,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=53.6,
        ),
    )
    assert harness.shoot_fresh_vision_active is False

    # 다음 Pick 성공이 확인되면 재무장하고, 그 뒤 골대가 80cm 이내로
    # 검출될 때 두 번째 Shoot용 FreshVision을 다시 활성화한다.
    harness.ball_in_hand = True
    harness.turn_after_shoot = False
    harness.neck_down_pending = False
    harness.shoot_in_progress = False
    MainDecision.CheckBall(harness)
    assert harness.has_ball is True
    assert harness.shoot_fresh_vision_armed is True
    assert harness.shoot_fresh_vision_active is False

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Ball.Shoot_Close,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=80.0,
        ),
    )
    assert harness.shoot_fresh_vision_active is True
