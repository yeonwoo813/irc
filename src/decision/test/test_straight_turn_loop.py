"""Exercise direct line motions through real vision and MotionEnd callbacks."""

from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

import pytest

from decision.main_decision import MainDecision, Motion
from test_cached_vision_decision import _make_harness, _Publisher

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'vision/scripts'))
from line_status_publisher import LineStatusPublisher  # noqa: E402


@pytest.fixture
def robot():
    harness = _make_harness()
    harness.motion_pub = _Publisher()
    harness.pick_done = False
    harness.turn_after_pick = False
    harness.turn_after_shoot = False
    harness.lost_step = 0
    harness.hurdle_ignore_until = 0.0
    harness.hurdle_ignore_active = False
    harness.line_buffer.clear()
    harness.ball_buffer.clear()
    harness.hurdle_buffer.clear()
    harness.hurdle_ready_buffer.clear()
    for name in (
        'Decision', 'LineTracking', 'LostMode', 'MotionCommand',
        '_finish_post_shoot_detection_suppression',
    ):
        setattr(harness, name, MethodType(getattr(MainDecision, name), harness))
    return harness


def line(angle, distance=0.0, curve=False, points=4):
    published = _Publisher()
    publisher = LineStatusPublisher(SimpleNamespace(
        create_publisher=lambda *args: published,
    ))
    publisher.publish_line_status(
        point_count=points, line_angle=angle, tangent_angle=angle,
        follow_angle=angle, line_distance=distance,
        curve_a=0.001 if curve else 0.0,
    )
    return published.messages[-1]


def complete(robot, frames, *, ball=99, hurdle=99):
    if not isinstance(frames, list):
        frames = [frames] * 3
    previous = len(robot.motion_pub.messages)
    for frame in frames:
        MainDecision.LineResultCallback(robot, frame)
    for _ in range(3):
        MainDecision.BallResultCallback(robot, SimpleNamespace(
            status=ball, angle=0.0, ball_in_hand=False,
        ))
        MainDecision.HurdleResultCallback(robot, SimpleNamespace(
            status=hurdle, angle=0.0, hurdle_ready=False,
        ))
    assert len(robot.motion_pub.messages) == previous
    MainDecision.MotionEndCallback(robot, SimpleNamespace(
        motion_ready=True, motion_end=True,
    ))
    assert len(robot.motion_pub.messages) == previous + 1
    return robot.motion_pub.messages[-1].command


@pytest.mark.parametrize('sign, fine, half, turn', [(-1, 2, 4, 6), (1, 3, 5, 7)])
@pytest.mark.parametrize('angle, kind', [
    (25.0, 'fine'), (25.1, 'half'), (29.9, 'half'),
    (30.0, 'turn'), (43.0, 'turn'), (49.9, 'turn'),
])
def test_straight_angle_selects_original_motion(robot, sign, fine, half, turn, angle, kind):
    expected = {'fine': fine, 'half': half, 'turn': turn}[kind]
    frame = line(sign * angle)
    assert frame.status == expected
    assert complete(robot, frame) == expected


@pytest.mark.parametrize('angle, expected', [(-50.0, 21), (50.0, 22)])
def test_existing_large_angle_motion_is_preserved(robot, angle, expected):
    assert complete(robot, line(angle)) == expected


def test_each_motion_end_rechecks_angle_without_extra_pose_or_forced_turn(robot):
    angles = [-32.0, -28.0, -23.0, 33.0, 6.0]
    assert [complete(robot, line(angle)) for angle in angles] == [6, 4, 2, 7, 1]


def test_mid_motion_frames_do_not_publish_commands(robot):
    assert complete(robot, line(-32.0)) == 6
    for _ in range(3):
        MainDecision.LineResultCallback(robot, line(32.0))
    assert [msg.command for msg in robot.motion_pub.messages] == [6]
    assert complete(robot, line(6.0)) == 1


def test_voted_status_uses_its_own_frame_angle(robot):
    assert complete(robot, [line(-32.0), line(-32.0), line(23.0)]) == 6
    assert robot.line_decision_detail['decision_angle'] == -32.0


def test_distance_corrected_angle_selects_motion_directly(robot):
    corrected_half = line(-38.7, 159.1)
    assert corrected_half.angle == pytest.approx(29.66, abs=0.01)
    assert complete(robot, corrected_half) == 4
    corrected_turn = line(-40.7, 159.1)
    assert corrected_turn.angle == pytest.approx(31.66, abs=0.01)
    assert complete(robot, corrected_turn) == 6
    assert complete(robot, line(-22.0, 299.9)) == 1


@pytest.mark.parametrize('angle', [-55.0, -35.0, -23.0, 6.0, 23.0, 35.0, 55.0])
def test_curve_command_executes_immediately_after_straight_turn(robot, angle):
    assert complete(robot, line(-32.0)) == 6
    curved = line(angle, curve=True)
    assert complete(robot, curved) == curved.status


def test_single_point_command_executes_immediately_after_straight_turn(robot):
    assert complete(robot, line(-32.0)) == 6
    follow = line(-23.0, points=1)
    assert complete(robot, follow) == follow.status
    assert robot.current_mode == 'LineTrackingMode'


def test_lost_search_still_restores_initial_pose_and_scans(robot):
    assert complete(robot, line(-32.0)) == 6
    missing = line(0.0, -335.9, points=0)
    assert complete(robot, missing) == 27
    assert robot.current_mode == 'LostMode'
    assert complete(robot, missing) == 15


@pytest.mark.parametrize('mode, ball, hurdle, command', [
    ('BallMode', 2, 99, 2), ('HurdleMode', 99, 27, 27),
])
@pytest.mark.parametrize('distance', [0.0, -335.9])
def test_other_missions_take_priority_immediately_after_turn(robot, mode, ball, hurdle, command, distance):
    assert complete(robot, line(-32.0)) == 6

    def enter_mission():
        robot.current_mode = mode
        robot.status = command
        robot.MotionCommand()

    setattr(robot, mode, enter_mission)
    assert complete(robot, line(-24.0, distance), ball=ball, hurdle=hurdle) == command
    assert robot.current_mode == mode


@pytest.mark.parametrize('distance, turn', [
    (-335.9, 6), (-300.0, 6), (300.0, 7), (335.9, 7),
])
@pytest.mark.parametrize('angle', [-55.0, 0.0, 55.0])
def test_300px_straight_offset_turns_toward_distance(robot, distance, turn, angle):
    assert complete(robot, line(angle, distance)) == turn


@pytest.mark.parametrize('sign, turn', [(-1, 6), (1, 7)])
def test_distance_override_ends_at_300px_boundary(robot, sign, turn):
    angle = -sign * 9.8
    distances = [299.9, 300.0, 335.9, 299.9]
    assert [complete(robot, line(angle, sign * d)) for d in distances] == [1, turn, turn, 1]


@pytest.mark.parametrize('distance', [-335.9, 335.9])
@pytest.mark.parametrize('points, curve', [(1, False), (4, True)])
def test_300px_does_not_override_follow_or_curve(robot, distance, points, curve):
    angle = 55.0 if distance < 0.0 else -55.0
    ordinary = line(angle, distance, points=points, curve=curve)
    assert complete(robot, ordinary) == ordinary.status


def test_distance_override_uses_the_voted_frame(robot):
    near = line(0.0)
    far = line(0.0, 335.9)
    assert complete(robot, [near, near, far]) == 1
    corrected_forward = line(-9.8, 335.9)
    assert corrected_forward.status == 1
    assert complete(robot, [corrected_forward, corrected_forward, line(-32.0)]) == 7
