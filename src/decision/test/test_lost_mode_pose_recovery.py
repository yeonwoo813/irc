from collections import deque
from types import SimpleNamespace

from decision.main_decision import Line, MainDecision, Motion


def _lost_harness(**overrides):
    harness = SimpleNamespace(
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


def test_lost_body_turn_line_recovery_runs_back_to_walk_first():
    harness = _lost_harness(
        line_status=Motion.Forward_4step,
        lost_step=4,
        lost_found_dir=1,
        lost_body_turn_count=1,
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


def test_lost_recovery_stays_lost_if_line_disappears_during_back_to_walk():
    harness = _lost_harness(
        line_status=Line.Line_None,
        lost_step=4,
        lost_found_dir=-1,
        lost_body_turn_count=1,
        lost_initial_pose_done=True,
        lost_back_to_walk_pending=True,
    )

    MainDecision.LostMode(harness)

    assert harness.commands == [Motion.Left_Turn]
    assert harness.lost_back_to_walk_pending is False
    assert harness.lost_body_turn_count == 2
    assert harness.line_tracking_calls == 0
