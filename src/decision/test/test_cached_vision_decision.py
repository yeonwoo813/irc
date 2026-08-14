from collections import deque
from types import MethodType, SimpleNamespace

from decision.main_decision import Ball, MainDecision, Motion


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
        hurdle_ready_buffer=deque([False, True, False, True, True], maxlen=5),
        hurdle_detection_buffer=deque(maxlen=5),
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
    assert list(harness.hurdle_ready_buffer) == [False, True, False, True, True]
    assert list(harness.hurdle_detection_buffer) == []


def test_hurdle_detection_uses_five_sample_majority_and_stays_latched():
    harness = _make_harness()

    for detected in (True, False, True, False):
        MainDecision.HurdleResultCallback(
            harness,
            SimpleNamespace(
                status=27 if detected else 99,
                angle=0.0,
                hurdle_ready=False,
            ),
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

    # True가 확정된 뒤에는 후속 5개가 모두 False여도 횡단 완료 전까지 유지합니다.
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
    assert list(harness.hurdle_detection_buffer) == []


def test_hurdle_detection_stays_false_when_five_sample_majority_is_false():
    harness = _make_harness()

    for detected in (True, False, False, True, False):
        MainDecision.HurdleResultCallback(
            harness,
            SimpleNamespace(
                status=27 if detected else 99,
                angle=0.0,
                hurdle_ready=False,
            ),
        )

    assert list(harness.hurdle_detection_buffer) == [
        True,
        False,
        False,
        True,
        False,
    ]
    assert harness.hurdle_detected is False


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
