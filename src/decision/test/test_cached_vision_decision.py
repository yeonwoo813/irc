from collections import deque
from types import MethodType, SimpleNamespace

from decision.main_decision import MainDecision, Motion


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
    assert harness.hurdle_status == Motion.Left_Turn_Mission


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
        hurdle_status=Motion.Left_Turn_Mission,
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
    harness.hurdle_status = Motion.Right_Turn_Mission
    MainDecision.HurdleMode(harness)

    assert harness.commands == [Motion.Hurdle_Forward_20, Motion.Hurdle_Go]
    assert harness.hurdle_step == 0
    assert harness.hurdle_count == 1
    assert harness.hurdle_go_active is True
    assert harness.hurdle_go_started is False
