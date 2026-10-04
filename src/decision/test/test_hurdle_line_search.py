from types import MethodType, SimpleNamespace
import time

import pytest

from decision.main_decision import Ball, Line, MainDecision, Motion
from test_cached_vision_decision import _make_harness, _Publisher


def _hurdle_harness():
    harness = _make_harness()
    harness.current_mode = 'HurdleMode'
    harness.hurdle_step = 0
    harness.hurdle_status = Motion.Neck_Left
    harness.hurdle_ready = False
    harness.hurdle_detected = True
    harness.hurdle_ignore_until = time.monotonic() - 1.0
    harness.hurdle_ignore_active = False
    harness.line_status = Line.Line_None
    harness.ball_vision_active = True
    harness.hoop_vision_active = False
    harness.motion_pub = _Publisher()
    harness.ball_mode_calls = []
    harness.BallMode = lambda: harness.ball_mode_calls.append(True)
    harness.LineTracking = lambda: pytest.fail('Search left HurdleMode')
    harness.LostMode = lambda: pytest.fail('Search entered LostMode')
    for name in ('MotionCommand', 'Decision', 'HurdleMode'):
        setattr(harness, name, MethodType(getattr(MainDecision, name), harness))
    MainDecision._reset_hurdle_line_search(harness)
    MainDecision.HurdleMode(harness)
    assert _commands(harness) == [Motion.Neck_Left]
    return harness


def _commands(harness):
    return [msg.command for msg in harness.motion_pub.messages]


def _line(harness, status):
    MainDecision.LineResultCallback(harness, SimpleNamespace(
        status=status, angle=0.0, follow_point=False,
    ))


def _finish(harness, line=Line.Line_None, hurdle=Motion.Neck_Left, ready=False):
    count = len(_commands(harness))
    MainDecision.MotionEndCallback(
        harness, SimpleNamespace(motion_ready=True, motion_end=False),
    )
    for _ in range(5):
        _line(harness, line)
        # 공 검출이 함께 와도 허들 내부 탐색을 중단하지 않는다.
        MainDecision.BallResultCallback(harness, SimpleNamespace(
            status=Motion.Pick, angle=0.0, ball_in_hand=False,
        ))
        MainDecision.HurdleResultCallback(harness, SimpleNamespace(
            status=hurdle, angle=0.0, hurdle_ready=ready,
        ))
    assert len(_commands(harness)) == count
    MainDecision.MotionEndCallback(
        harness, SimpleNamespace(motion_ready=True, motion_end=True),
    )


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('recovery_turn', [3, 10])
def test_hurdle_neck_search_forces_three_then_rotates_until_line(
    direction, recovery_turn,
):
    harness = _hurdle_harness()
    if direction == 1:
        _finish(harness)
        assert _commands(harness) == [15, 16]
    # 목 회전 중 한 번 본 라인은 종료 직전 다수결이 99여도 기억한다.
    _line(harness, Motion.Forward_4step)
    _finish(harness)
    assert _commands(harness)[-1] == Motion.Neck_Center
    assert harness.hurdle_line_found_dir == direction
    assert harness.hurdle_neck_scan_side == 0
    turn = Motion.Left_Turn_Afterpick if direction == -1 else Motion.Right_Turn_Afterpick

    _finish(harness, line=1, hurdle=26, ready=True)
    assert _commands(harness)[-1] == turn
    for completed in range(1, recovery_turn + 1):
        assert harness.hurdle_line_turn_count == completed
        before = len(_commands(harness))
        # 최소 3회는 라인/허들 ready가 보여도 끝까지 회전한다.
        visible = completed <= 3 or completed == recovery_turn
        _finish(harness, line=1 if visible else 99, hurdle=26, ready=True)
        if completed == 3:
            assert len(_commands(harness)) == before
            assert list(harness.line_buffer) == []
            # 세 번째 회전의 종료 후 3개 라인 확인은 기존 Lost와 동일하다.
            fresh_line = 1 if recovery_turn == 3 else 99
            for index in range(3):
                _line(harness, fresh_line)
                assert len(_commands(harness)) == before + (index == 2)
        if completed < recovery_turn:
            assert _commands(harness)[-1] == turn
        else:
            assert _commands(harness)[-1] == Motion.Hurdle_Forward_20

    assert _commands(harness).count(turn) == recovery_turn
    assert Motion.Back_To_Walk not in _commands(harness)
    assert Motion.Backward_half not in _commands(harness)
    assert harness.hurdle_line_search_step == 0
    assert harness.hurdle_step == 1
    assert harness.hurdle_count == 0
    assert harness.current_mode == 'HurdleMode'
    assert harness.ball_mode_calls == []
    _finish(harness, line=1, hurdle=23)
    assert _commands(harness)[-1] == Motion.Hurdle_Go
    assert harness.hurdle_count == 1


def test_hurdle_both_neck_sides_fail_back_up_once_then_search_again():
    harness = _hurdle_harness()
    for cycle in range(2):
        _finish(harness)
        assert _commands(harness)[-1] == Motion.Neck_Right
        _finish(harness)
        assert _commands(harness)[-1] == Motion.Neck_Center
        assert _commands(harness).count(Motion.Backward_half) == cycle
        _finish(harness)
        assert _commands(harness)[-1] == Motion.Backward_half
        assert _commands(harness).count(Motion.Backward_half) == cycle + 1
        _finish(harness)
        assert _commands(harness)[-1] == Motion.Neck_Left
    assert _commands(harness) == [15, 16, 17, 9, 15, 16, 17, 9, 15]
    assert harness.current_mode == 'HurdleMode'


@pytest.mark.parametrize('hurdle', [99, 26])
def test_missing_or_unconfirmed_hurdle_results_do_not_stall_neck_search(hurdle):
    harness = _hurdle_harness()
    _finish(harness, hurdle=hurdle, ready=False)
    assert _commands(harness) == [15, 16]


def test_stale_search_request_does_not_restart_after_front_line_returns():
    harness = _hurdle_harness()
    _line(harness, 1)
    _finish(harness)
    _finish(harness)
    _finish(harness)
    _finish(harness)
    _finish(harness)
    assert harness.hurdle_line_turn_count == 3
    for _ in range(3):
        _line(harness, 1)
    assert _commands(harness) == [15, 17, 30, 30, 30]
    assert harness.hurdle_line_search_step == 0
    # 정렬 결과가 들어오면 추가 목 회전이나 25번 없이 기존 허들 정렬 실행.
    for _ in range(3):
        _line(harness, 1)
        MainDecision.BallResultCallback(harness, SimpleNamespace(
            status=Ball.Ball_None, angle=0.0, ball_in_hand=False,
        ))
        MainDecision.HurdleResultCallback(harness, SimpleNamespace(
            status=23, angle=-12.0, hurdle_ready=False,
        ))
    assert _commands(harness) == [15, 17, 30, 30, 30, 23]
