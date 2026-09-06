from collections import deque
import math
import time
from types import MethodType, SimpleNamespace

import pytest

from decision.main_decision import Ball, MainDecision, Motion


class _Logger:

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _Publisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _make_harness():
    harness = SimpleNamespace(
        test_mode=False,
        motion_ready=True,
        motion_end=False,
        current_mode="LineTrackingMode",
        line_data=False,
        ball_data=False,
        hurdle_data=False,
        line_buffer=deque([1, 2, 2, 2, 3], maxlen=5),
        line_vote_detail_buffer=deque(maxlen=5),
        line_frame_sequence=0,
        ball_buffer=deque([99, 99, 12, 99, 12], maxlen=5),
        ball_vote_detail_buffer=deque(maxlen=5),
        ball_frame_sequence=0,
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
        pre_shoot_result_waiting=False,
        pre_shoot_verified_result=None,
    )
    harness.logger = _Logger()
    harness.decision_count = 0
    harness.get_logger = lambda: harness.logger

    def record_decision():
        harness.decision_count += 1

    harness.Decision = record_decision
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness._try_decision_from_cached_results = MethodType(
        MainDecision._try_decision_from_cached_results,
        harness,
    )
    harness._reset_vision_decision_cycle = MethodType(
        MainDecision._reset_vision_decision_cycle,
        harness,
    )
    harness._consume_pre_shoot_verified_result = MethodType(
        MainDecision._consume_pre_shoot_verified_result,
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


def test_hoop_approach_motion_end_uses_cached_results_immediately():
    harness = _make_harness()
    harness.current_mode = "BallMode"
    harness.has_ball = True
    harness.ball_buffer = deque(
        [Motion.Forward_3step, Motion.Forward_half,
         Motion.Forward_3step, Motion.Forward_3step],
        maxlen=5,
    )

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Motion.Forward_3step,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=70.0,
        ),
    )

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )

    assert harness.decision_count == 1
    assert harness.ball_status == Motion.Forward_3step
    assert list(harness.ball_buffer) == [
        Motion.Forward_3step,
        Motion.Forward_half,
        Motion.Forward_3step,
        Motion.Forward_3step,
        Motion.Forward_3step,
    ]


@pytest.mark.parametrize('command', [Motion.Back_To_Initial, Motion.Shoot_Forward])
def test_pre_shoot_motion_end_waits_for_verified_ball_result(command):
    harness = _make_harness()
    harness.current_mode = "BallMode"
    harness.has_ball = True
    harness.status = command
    harness.motion_pub = _Publisher()
    MainDecision.MotionCommand(harness)
    assert harness.pre_shoot_result_waiting is True
    harness.ball_mode_calls = 0
    harness.BallMode = lambda: setattr(
        harness,
        "ball_mode_calls",
        harness.ball_mode_calls + 1,
    )

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Motion.Shoot,
            angle=0.0,
            ball_in_hand=True,
            goal_distance_cm=70.0,
            pre_shoot_verified=False,
        ),
    )
    assert harness.ball_mode_calls == 0

    MainDecision.MotionEndCallback(
        harness,
        SimpleNamespace(motion_ready=True, motion_end=True),
    )
    assert harness.pre_shoot_result_waiting is True
    assert harness.ball_mode_calls == 0
    assert list(harness.ball_buffer) == []

    MainDecision.BallResultCallback(
        harness,
        SimpleNamespace(
            status=Motion.Shoot_Close,
            angle=-1.5,
            ball_in_hand=True,
            detected_angle=-1.7,
            goal_x_px=411.25,
            goal_y_px=126.5,
            goal_distance_cm=60.0,
            pre_shoot_verified=True,
        ),
    )

    assert harness.pre_shoot_result_waiting is False
    assert harness.pre_shoot_verified_result is None
    assert harness.ball_status == Motion.Shoot_Close
    assert harness.ball_angle == -1.5
    assert harness.latest_goal_distance_cm == 60.0
    assert harness.ball_mode_calls == 1
    assert len(harness.ball_vote_detail_buffer) == 1
    detail = harness.ball_vote_detail_buffer[-1]
    assert detail['goal_x_px'] == 411.25
    assert detail['goal_y_px'] == 126.5
    assert detail['measured_angle'] == -1.7


@pytest.mark.parametrize('command', [Motion.Back_To_Initial, Motion.Shoot_Forward])
def test_has_ball_positioning_motion_arms_verified_result_wait(command):
    publisher = _Publisher()
    harness = _make_harness()
    harness.status = command
    harness.has_ball = True
    harness.current_mode = "BallMode"
    harness.motion_pub = publisher
    harness.pre_shoot_result_waiting = False
    harness.pre_shoot_verified_result = (Motion.Shoot, 0.0, True, 70.0)

    MainDecision.MotionCommand(harness)

    assert publisher.messages[-1].command == command
    assert harness.pre_shoot_result_waiting is True
    assert harness.pre_shoot_verified_result is None
    assert harness.motion_end is False


def test_shoot_forward_preserves_possession_and_does_not_start_post_shoot():
    harness = _make_harness()
    harness.has_ball = True
    harness.ball_status = Motion.Shoot_Forward
    harness.pick_done = False
    harness.backward_after_pick = False
    harness.turn_after_pick = False
    harness.turn_after_shoot = False
    harness.neck_down_pending = False
    harness.shoot_in_progress = False
    harness.post_shoot_detection_suppressed = False
    harness.motion_pub = _Publisher()
    harness.MotionCommand = MethodType(MainDecision.MotionCommand, harness)

    MainDecision.BallMode(harness)

    assert harness.motion_pub.messages[-1].command == Motion.Shoot_Forward
    assert harness.has_ball is True
    assert harness.pre_shoot_result_waiting is True
    assert harness.neck_down_pending is False
    assert harness.turn_after_shoot is False
    assert harness.shoot_in_progress is False
    assert harness.post_shoot_detection_suppressed is False


def test_lost_back_to_initial_does_not_arm_pre_shoot_wait():
    publisher = _Publisher()
    harness = _make_harness()
    harness.status = Motion.Back_To_Initial
    harness.has_ball = True
    harness.current_mode = "LostMode"
    harness.motion_pub = publisher
    harness.pre_shoot_result_waiting = False
    harness.pre_shoot_verified_result = None

    MainDecision.MotionCommand(harness)

    assert publisher.messages[-1].command == Motion.Back_To_Initial
    assert harness.pre_shoot_result_waiting is False
    assert harness.motion_end is False


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
    harness.hurdle_ignore_until = time.monotonic() - 0.1
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


def test_pick_evidence_log_contains_raw_pixels_offsets_and_angle():
    harness = _make_harness()
    harness.ball_vote_detail_buffer = deque(
        [
            {
                'sequence': 10,
                'status': Ball.Pick_Ready,
                'decision_angle': 0.0,
                'measured_angle': -1.25,
                'x_distance_px': -12.5,
                'y_distance_px': 82.25,
                'ball_x_px': 332.5,
                'ball_y_px': 396.75,
                'goal_x_px': math.nan,
                'goal_y_px': math.nan,
                'goal_distance_cm': 0.0,
                'ball_in_hand': False,
            },
        ],
        maxlen=5,
    )

    MainDecision._log_ball_action_evidence(harness, Ball.Pick_Ready)

    message = harness.logger.messages[-1]
    assert '[PickDecisionEvidence]' in message
    assert 'ball_raw_x=332.50px' in message
    assert 'ball_raw_y=396.75px' in message
    assert 'x_offset=-12.50px' in message
    assert 'y_offset=82.25px' in message
    assert 'measured_angle=-1.25deg' in message
    assert 'decision_angle=0.00deg' in message


def test_shoot_evidence_log_contains_goal_pixels_distance_and_angle():
    harness = _make_harness()
    harness.ball_vote_detail_buffer = deque(
        [
            {
                'sequence': 20,
                'status': Motion.Shoot,
                'decision_angle': 0.0,
                'measured_angle': 2.75,
                'x_distance_px': math.nan,
                'y_distance_px': math.nan,
                'ball_x_px': math.nan,
                'ball_y_px': math.nan,
                'goal_x_px': 418.25,
                'goal_y_px': 127.5,
                'goal_distance_cm': 69.5,
                'ball_in_hand': True,
            },
        ],
        maxlen=5,
    )

    MainDecision._log_ball_action_evidence(harness, Motion.Shoot)

    message = harness.logger.messages[-1]
    assert '[ShootDecisionEvidence]' in message
    assert 'goal_raw_x=418.25px' in message
    assert 'goal_raw_y=127.50px' in message
    assert 'goal_distance=69.50cm' in message
    assert 'measured_angle=2.75deg' in message
    assert 'decision_angle=0.00deg' in message


def test_line_vote_log_contains_the_exact_three_frame_inputs():
    harness = _make_harness()
    harness.line_buffer.clear()
    harness.line_vote_detail_buffer.clear()

    frames = [
        (21, 5, "curve", -26.9, 68.0, 1.45e-3),
        (21, 6, "curve", -36.8, 97.0, 2.27e-3),
        (21, 6, "curve", -34.5, -6.0, 1.30e-3),
    ]
    for status, point_count, decision_type, angle, distance, curve_a in frames:
        MainDecision.LineResultCallback(
            harness,
            SimpleNamespace(
                status=status,
                angle=abs(angle),
                follow_point=False,
                point_count=point_count,
                decision_type=decision_type,
                decision_angle=angle,
                line_distance=distance,
                curve_a=curve_a,
            ),
        )

    harness.motion_end = True
    assert harness._try_decision_from_cached_results() is True

    vote_logs = [
        message
        for message in harness.logger.messages
        if "[LineVoteFrames]" in message
    ]
    assert len(vote_logs) == 1
    assert "selected=21" in vote_logs[0]
    assert "seq=1 status=21 type=curve pc=5 angle=-26.9deg distance=+68.0px" in vote_logs[0]
    assert "seq=2 status=21 type=curve pc=6 angle=-36.8deg distance=+97.0px" in vote_logs[0]
    assert "seq=3 status=21 type=curve pc=6 angle=-34.5deg distance=-6.0px" in vote_logs[0]


def test_confirmed_hurdle_result_stays_latched_until_crossing_completes():
    harness = _make_harness()
    harness.hurdle_ignore_until = time.monotonic() - 0.1

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


def test_hurdle_is_ignored_for_three_seconds_after_first_motion_starts():
    harness = _make_harness()
    harness.hurdle_ignore_until = None
    harness.status = Motion.Initial_Pose
    harness.motion_pub = _Publisher()

    # 시작 전에 남아 있던 양성 상태가 있더라도 gate가 강제로 지워야 한다.
    harness.hurdle_detected = True
    harness.hurdle_ready = True
    harness.hurdle_go_active = True
    harness.hurdle_go_started = True
    harness.hurdle_status = 27
    harness.hurdle_angle = 12.0

    # 첫 모션이 시작되기 전에는 deadline 자체가 없으며, 이 상태도
    # 허들 검출 금지 구간이어야 한다.
    MainDecision.HurdleResultCallback(
        harness,
        SimpleNamespace(status=27, angle=12.0, hurdle_ready=True),
    )
    assert harness.hurdle_detected is False
    assert harness.hurdle_ready is False
    assert harness.hurdle_go_active is False
    assert harness.hurdle_status == 99
    assert harness.hurdle_angle == 0.0
    assert list(harness.hurdle_buffer) == [99]
    assert list(harness.hurdle_ready_buffer) == [False]

    MainDecision.MotionCommand(harness)
    assert harness.hurdle_ignore_until is None

    # 초기자세 명령만 실행된 뒤에도 첫 실제 모션 전이므로 계속 막는다.
    MainDecision.HurdleResultCallback(
        harness,
        SimpleNamespace(status=27, angle=12.0, hurdle_ready=True),
    )
    assert harness.hurdle_detected is False
    assert harness.hurdle_buffer[-1] == 99
    assert harness.hurdle_ready_buffer[-1] is False

    harness.status = Motion.Forward_4step
    MainDecision.MotionCommand(harness)
    assert 2.9 <= harness.hurdle_ignore_until - time.monotonic() <= 3.0

    MainDecision.HurdleResultCallback(
        harness,
        SimpleNamespace(status=27, angle=12.0, hurdle_ready=True),
    )
    assert harness.hurdle_detected is False
    assert harness.hurdle_buffer[-1] == 99
    assert harness.hurdle_ready_buffer[-1] is False

    harness.hurdle_ignore_until = time.monotonic() - 0.1

    MainDecision.HurdleResultCallback(
        harness,
        SimpleNamespace(status=27, angle=12.0, hurdle_ready=True),
    )
    assert harness.hurdle_detected is True
    # gate 이전의 강제 False 샘플도 남기지 않고 첫 유효 프레임부터 시작한다.
    assert list(harness.hurdle_buffer) == [27]
    assert list(harness.hurdle_ready_buffer) == [True]


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


def _make_missing_post_pick_goal_harness():
    harness = SimpleNamespace(
        motion_ready=True,
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
        current_mode="BallMode",
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness.selected_mode = None
    harness.HurdleMode = lambda: setattr(harness, "selected_mode", "hurdle")
    harness.BallMode = lambda: setattr(harness, "selected_mode", "ball")
    harness.LostMode = lambda: setattr(harness, "selected_mode", "lost")
    harness.LineTracking = lambda: setattr(harness, "selected_mode", "line")
    return harness


def test_post_pick_missing_goal_immediately_leaves_ball_mode():
    harness = _make_missing_post_pick_goal_harness()

    MainDecision.Decision(harness)

    assert harness.selected_mode == "line"


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


def _make_missing_pre_pick_ball_harness(ball_status):
    harness = SimpleNamespace(
        motion_ready=True,
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
        current_mode="BallMode",
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
    harness.selected_mode = None
    harness.HurdleMode = lambda: setattr(harness, "selected_mode", "hurdle")
    harness.BallMode = lambda: setattr(harness, "selected_mode", "ball")
    harness.LostMode = lambda: setattr(harness, "selected_mode", "lost")
    harness.LineTracking = lambda: setattr(harness, "selected_mode", "line")
    return harness


def test_pre_pick_missing_ball_immediately_leaves_ball_mode():
    for ball_status in (Ball.Ball_None, Ball.Ball_Lost):
        harness = _make_missing_pre_pick_ball_harness(ball_status)

        MainDecision.Decision(harness)

        assert harness.selected_mode == "line"


def _make_post_pick_harness(ball_in_hand):
    harness = SimpleNamespace(
        current_mode="BallMode",
        pick_done=True,
        turn_after_pick=False,
        backward_after_pick=False,
        ball_in_hand=ball_in_hand,
        has_ball=False,
        ball_data=True,
        ball_buffer=deque([12, 12, 12], maxlen=5),
        post_pick_failure_ball_suppressed=False,
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.commands = []
    harness.turn_after_pick_calls = 0
    harness.MotionCommand = lambda: harness.commands.append(harness.status)
    harness._ball_status_is_detected = MainDecision._ball_status_is_detected
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
    )
    harness.logger = _Logger()
    harness.get_logger = lambda: harness.logger
    harness.ball_vote_detail_buffer = deque(maxlen=5)
    harness.commands = []
    harness.check_ball_calls = 0
    harness.MotionCommand = lambda: harness.commands.append(harness.status)

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
    assert harness.post_shoot_detection_suppressed is True
    assert harness.ball_vision_active is False
    assert harness.hoop_vision_active is False
