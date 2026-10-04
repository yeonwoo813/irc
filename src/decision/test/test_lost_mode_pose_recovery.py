from collections import deque
from types import SimpleNamespace

import pytest

from line_status_publisher import LineTrackingController

from decision.main_decision import Line, MainDecision, Motion


def _lost_harness(**overrides):
    harness = SimpleNamespace(
        line_tracking=LineTrackingController(),
        current_mode="LineTrackingMode",
        line_status=Line.Line_None,
        lost_count=0,
        lost_step=0,
        lost_found_dir=0,
        lost_body_turn_count=0,
        lost_initial_pose_done=False,
        lost_back_to_walk_pending=False,
        lost_left_line_seen=False,
        lost_right_line_seen=False,
        lost_neck_scan_side=0,
        status=None,
        commands=[],
        line_tracking_calls=0,
    )
    for name, value in overrides.items():
        setattr(harness, name, value)

    harness.MotionCommand = lambda: harness.commands.append(harness.status)

    def line_tracking():
        harness.line_tracking_calls += 1
        MainDecision.LineTracking(harness)

    harness.LineTracking = line_tracking
    harness._finish_post_shoot_detection_suppression = lambda reason: False
    return harness


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def _send_line_result(harness, status, point_count):
    logger = _Logger()
    harness.test_mode = True
    harness.motion_ready = True
    harness.motion_end = False
    harness.line_buffer = deque(maxlen=5)
    harness.line_vote_detail_buffer = deque(maxlen=5)
    harness.line_frame_sequence = 0
    harness.get_logger = lambda: logger

    MainDecision.LineResultCallback(
        harness,
        SimpleNamespace(
            status=status,
            angle=0.0,
            follow_point=False,
            point_count=point_count,
            decision_type="straight",
            decision_angle=0.0,
            line_distance=0.0,
            curve_a=0.0,
        ),
    )
    return logger


def test_lost_mode_starts_with_back_to_initial_once():
    harness = _lost_harness()

    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Back_To_Initial]
    assert harness.lost_initial_pose_done is True
    assert harness.lost_step == 0

    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Back_To_Initial, Motion.Neck_Left]
    assert harness.lost_step == 1


def test_lost_left_scan_latches_line_seen_during_motion():
    harness = _lost_harness(
        current_mode="LostMode",
        status=Motion.Neck_Left,
        lost_step=1,
        lost_initial_pose_done=True,
        lost_neck_scan_side=-1,
    )

    logger = _send_line_result(harness, Motion.Forward_4step, 3)

    assert harness.lost_left_line_seen is True
    assert harness.lost_right_line_seen is False
    assert any("side=left" in message for message in logger.messages)

    harness.line_status = Line.Line_None
    harness.commands.clear()
    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Neck_Center]
    assert harness.lost_found_dir == -1
    assert harness.lost_step == 3
    assert harness.lost_neck_scan_side == 0


def test_lost_right_scan_latches_line_seen_during_motion():
    harness = _lost_harness(
        current_mode="LostMode",
        status=Motion.Neck_Right,
        lost_step=2,
        lost_initial_pose_done=True,
        lost_neck_scan_side=1,
    )

    logger = _send_line_result(harness, Motion.Left_Turn, 1)

    assert harness.lost_left_line_seen is False
    assert harness.lost_right_line_seen is True
    assert any("side=right" in message for message in logger.messages)

    harness.line_status = Line.Line_None
    harness.commands.clear()
    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Neck_Center]
    assert harness.lost_found_dir == 1
    assert harness.lost_step == 3
    assert harness.lost_neck_scan_side == 0


@pytest.mark.parametrize("direction, command", [
    (-1, Motion.Left_Turn_Afterpick),
    (1, Motion.Right_Turn_Afterpick),
])
@pytest.mark.parametrize("line_status", [
    Line.Line_None, Motion.Forward_4step, Motion.Left_Turn,
])
def test_neck_center_always_starts_first_body_turn(
    direction, command, line_status
):
    harness = _lost_harness(
        line_status=line_status,
        lost_step=3,
        lost_found_dir=direction,
        lost_initial_pose_done=True,
    )

    MainDecision.LostMode(harness)

    assert harness.commands == [command]
    assert harness.lost_step == 4
    assert harness.lost_body_turn_count == 1
    assert harness.lost_found_dir == direction
    assert harness.lost_back_to_walk_pending is False


@pytest.mark.parametrize("direction, command", [
    (-1, Motion.Left_Turn_Afterpick),
    (1, Motion.Right_Turn_Afterpick),
])
@pytest.mark.parametrize("completed_turns", [1, 2])
@pytest.mark.parametrize("line_status", [
    Line.Line_None, Motion.Forward_4step, Motion.Left_Turn,
])
def test_lost_body_turn_cannot_recover_before_third_turn(
    direction, command, completed_turns, line_status
):
    harness = _lost_harness(
        line_status=line_status,
        lost_step=4,
        lost_found_dir=direction,
        lost_body_turn_count=completed_turns,
        lost_initial_pose_done=True,
    )

    MainDecision.LostMode(harness)

    assert harness.commands == [command]
    assert harness.lost_body_turn_count == completed_turns + 1
    assert harness.lost_found_dir == direction
    assert harness.lost_back_to_walk_pending is False
    assert harness.line_tracking_calls == 0


@pytest.mark.parametrize("completed_turns", [3, 4, 5, 6, 7, 8])
def test_lost_body_turn_line_recovery_runs_back_to_walk_first(completed_turns):
    harness = _lost_harness(
        line_status=Motion.Forward_4step,
        lost_step=4,
        lost_found_dir=1,
        lost_body_turn_count=completed_turns,
        lost_initial_pose_done=True,
    )

    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Back_To_Walk]
    assert harness.lost_back_to_walk_pending is True
    assert harness.line_tracking_calls == 0

    MainDecision.LostMode(harness)

    assert harness.line_tracking_calls == 1
    assert harness.commands == [Motion.Back_To_Walk, Motion.Forward_4step]
    assert harness.current_mode == "LineTrackingMode"
    assert harness.lost_initial_pose_done is False
    assert harness.lost_back_to_walk_pending is False


def test_lost_recovery_returns_to_initial_if_line_disappears_during_back_to_walk():
    harness = _lost_harness(
        line_status=Line.Line_None,
        lost_step=4,
        lost_found_dir=-1,
        lost_body_turn_count=3,
        lost_initial_pose_done=True,
        lost_back_to_walk_pending=True,
        lost_left_line_seen=True,
    )

    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Back_To_Initial]
    assert harness.lost_back_to_walk_pending is False
    assert harness.lost_initial_pose_done is True
    assert harness.lost_step == 0
    assert harness.lost_found_dir == 0
    assert harness.lost_body_turn_count == 0
    assert harness.lost_left_line_seen is False
    assert harness.lost_right_line_seen is False
    assert harness.line_tracking_calls == 0

    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Back_To_Initial, Motion.Neck_Left]
    assert harness.lost_step == 1


def test_failed_neck_scan_centers_then_backs_up_once_per_scan():
    harness = _lost_harness(lost_initial_pose_done=True)

    for _ in range(8):
        MainDecision.LostMode(harness)

    assert harness.commands == [
        Motion.Neck_Left,
        Motion.Neck_Right,
        Motion.Neck_Center,
        Motion.Backward_half,
    ] * 2
    assert harness.lost_initial_pose_done is True
    assert harness.lost_step == 0

    MainDecision.LostMode(harness)

    assert harness.commands[-1] == Motion.Neck_Left
    assert harness.commands.count(Motion.Backward_half) == 2
    assert Motion.Back_To_Initial not in harness.commands


def test_failed_neck_scan_recovers_line_after_backward():
    harness = _lost_harness(lost_initial_pose_done=True)

    for _ in range(4):
        MainDecision.LostMode(harness)

    assert harness.commands[-1] == Motion.Backward_half
    harness.line_status = Motion.Forward_4step

    MainDecision.LostMode(harness)
    MainDecision.LostMode(harness)

    assert harness.commands[-2:] == [
        Motion.Back_To_Walk, Motion.Forward_4step,
    ]
    assert harness.commands.count(Motion.Backward_half) == 1
    assert harness.line_tracking_calls == 1


@pytest.mark.parametrize("direction, command", [
    (-1, Motion.Left_Turn_Afterpick),
    (1, Motion.Right_Turn_Afterpick),
])
def test_eight_body_turns_restart_neck_scan_before_backward(direction, command):
    harness = _lost_harness(
        line_status=Line.Line_None,
        lost_step=3,
        lost_found_dir=direction,
        lost_initial_pose_done=True,
    )

    for _ in range(8):
        MainDecision.LostMode(harness)

    assert harness.commands == [command] * 8
    assert harness.lost_body_turn_count == 8

    MainDecision.LostMode(harness)

    assert harness.commands == [command] * 8 + [Motion.Neck_Left]
    assert harness.lost_initial_pose_done is True
    assert harness.lost_found_dir == 0
    assert harness.lost_body_turn_count == 0

    for _ in range(3):
        MainDecision.LostMode(harness)

    assert harness.commands[-4:] == [
        Motion.Neck_Left,
        Motion.Neck_Right,
        Motion.Neck_Center,
        Motion.Backward_half,
    ]
    assert harness.commands.count(command) == 8
    assert harness.commands.count(Motion.Backward_half) == 1
    assert Motion.Back_To_Initial not in harness.commands
