from types import MethodType, SimpleNamespace

import pytest

from decision.main_decision import Ball, Line, MainDecision, Motion
from test_cached_vision_decision import _make_harness, _Publisher


def _shoot_harness(shoot, goal_count=0, minimum=4):
    harness = _make_harness()
    harness.current_mode = 'BallMode'
    harness.has_ball = True
    harness.pick_done = False
    harness.turn_after_pick = False
    harness.neck_down_pending = False
    harness.turn_after_shoot = False
    harness.goal_count = goal_count
    harness.post_shoot_min_turn_count = minimum
    harness.ball_status = shoot
    harness.line_status = Line.Line_None
    harness.ball_vision_active = False
    harness.hoop_vision_active = True
    harness.webcam_ball_allowed = True
    harness.motion_pub = _Publisher()
    harness.lost_mode_calls = []
    harness.LostMode = lambda: harness.lost_mode_calls.append(True)
    for method in (
        'Decision', 'MotionCommand', 'BallMode', 'TurnAfterShoot',
        'LineTracking', '_finish_turn_after_shoot',
        '_finish_post_shoot_detection_suppression',
    ):
        setattr(harness, method, MethodType(getattr(MainDecision, method), harness))
    harness.BallMode()
    assert _commands(harness) == [shoot]
    _finish(harness)
    assert _commands(harness) == [shoot, Motion.Neck_Down]
    return harness


def _commands(harness):
    return [message.command for message in harness.motion_pub.messages]


def _finish(harness, line=Line.Line_None):
    count = len(_commands(harness))
    MainDecision.MotionEndCallback(
        harness, SimpleNamespace(motion_ready=True, motion_end=False),
    )
    for _ in range(5):
        MainDecision.LineResultCallback(harness, SimpleNamespace(
            status=line, angle=0.0, follow_point=False,
        ))
        MainDecision.BallResultCallback(harness, SimpleNamespace(
            status=Ball.Ball_None, angle=0.0, ball_in_hand=False,
        ))
        MainDecision.HurdleResultCallback(harness, SimpleNamespace(
            status=99, angle=0.0, hurdle_ready=False,
        ))
    # 입력 콜백만으로 진행하지 않고 각 모션의 완료를 기다린다.
    assert len(_commands(harness)) == count
    MainDecision.MotionEndCallback(
        harness, SimpleNamespace(motion_ready=True, motion_end=True),
    )


@pytest.mark.parametrize('goal_count', [0, 1, 2])
@pytest.mark.parametrize('recovery_turn', [5, 12])
def test_shoot13_forces_five_and_keeps_turning_until_line(goal_count, recovery_turn):
    harness = _shoot_harness(Motion.Shoot, goal_count, minimum=2)
    turn = (
        Motion.Right_Turn_Afterpick if goal_count == 0
        else Motion.Left_Turn_Afterpick
    )
    _finish(harness, line=Motion.Forward_4step)
    assert _commands(harness)[-1] == turn
    for completed in range(1, recovery_turn + 1):
        # 5회 전에는 라인이 보여도 회전한다. 이후 없으면 10회도 넘긴다.
        visible = completed < 5 or completed == recovery_turn
        _finish(harness, line=Motion.Forward_4step if visible else Line.Line_None)
        if completed < recovery_turn:
            assert _commands(harness)[-1] == turn
            assert harness.turn_count == completed + 1
            assert harness.post_shoot_detection_suppressed is True
        else:
            assert _commands(harness)[-1] == Motion.Back_To_Walk
    assert harness.lost_mode_calls == []
    assert _commands(harness) == [
        Motion.Shoot, Motion.Neck_Down, *([turn] * recovery_turn),
        Motion.Back_To_Walk,
    ]
    # 25번 종료 콜백만으로 최신 라인 명령을 발행한다.
    _finish(harness, line=Motion.Right_Half_Forward)
    assert _commands(harness)[-1] == Motion.Right_Half_Forward
    assert harness.current_mode == 'LineTrackingMode'
    assert harness.post_shoot_motion is None
    assert harness.post_shoot_detection_suppressed is False


@pytest.mark.parametrize('shoot', [Motion.Shoot_Close, Motion.Shoot_Mid, Motion.Shoot_62])
@pytest.mark.parametrize('goal_count', [0, 1])
@pytest.mark.parametrize('minimum', [2, 4])
def test_other_shots_keep_existing_minimum(shoot, goal_count, minimum):
    harness = _shoot_harness(shoot, goal_count, minimum)
    turn = Motion.Right_Turn_Afterpick if goal_count == 0 else Motion.Left_Turn_Afterpick
    for _ in range(minimum):
        _finish(harness, line=Motion.Forward_4step)
        assert _commands(harness)[-1] == turn
    _finish(harness, line=Motion.Forward_4step)
    assert _commands(harness) == [
        shoot, Motion.Neck_Down, *([turn] * minimum), Motion.Back_To_Walk,
    ]
    _finish(harness, line=Motion.Forward_4step)
    assert _commands(harness)[-1] == Motion.Forward_4step
    assert harness.current_mode == 'LineTrackingMode'


@pytest.mark.parametrize('shoot', [Motion.Shoot_Close, Motion.Shoot_Mid, Motion.Shoot_62])
def test_other_shots_keep_ten_turn_limit(shoot):
    harness = _shoot_harness(shoot)
    for _ in range(10):
        _finish(harness)
        assert harness.lost_mode_calls == []
    assert _commands(harness).count(Motion.Right_Turn_Afterpick) == 10
    _finish(harness)
    assert harness.lost_mode_calls == [True]
    assert harness.turn_after_shoot is False


@pytest.mark.parametrize('shoot', [Motion.Shoot_Close, Motion.Shoot_Mid, Motion.Shoot_62])
def test_other_shots_still_skip_turns_after_second_goal(shoot):
    harness = _shoot_harness(shoot, goal_count=2)
    _finish(harness, line=Motion.Forward_4step)
    assert _commands(harness) == [shoot, Motion.Neck_Down, Motion.Forward_4step]
    assert harness.current_mode == 'LineTrackingMode'
