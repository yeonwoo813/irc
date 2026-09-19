"""Limit large-tangent priority to the near-offset curve band."""

from collections import Counter
import math
from pathlib import Path
import sys

import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from line_status_publisher import LineDecision, LineFeatures, LineStatus  # noqa: E402


def curve_features(distance, angle):
    return LineFeatures(
        point_count=8,
        line_angle=angle,
        tangent_angle=angle,
        curve_a=2.0e-3,
        line_distance=distance,
    )


@pytest.mark.parametrize("distance", [-129.99, -100.0, -90.0, 90.0, 100.0, 129.99])
@pytest.mark.parametrize("angle", [-60.0, -40.01, 40.01, 60.0])
def test_large_tangent_overrides_distance_only_in_near_curve_band(distance, angle):
    status, result_angle = LineDecision().decide(curve_features(distance, angle))

    expected = (
        LineStatus.Left_Turn_Curve if angle < 0 else LineStatus.Right_Turn_Curve
    )
    assert status == expected
    assert result_angle == pytest.approx(abs(angle))


@pytest.mark.parametrize("distance", [-100.0, 100.0])
@pytest.mark.parametrize("angle", [-40.0, -39.99, 39.99, 40.0])
def test_exactly_40_degrees_and_below_keep_distance_priority(distance, angle):
    status, result_angle = LineDecision().decide(curve_features(distance, angle))

    expected = (
        LineStatus.Left_Half_Forward
        if distance < 0 else LineStatus.Right_Half_Forward
    )
    assert status == expected
    assert result_angle == 0.0


@pytest.mark.parametrize("distance", [-89.99, 0.0, 89.99])
@pytest.mark.parametrize("angle", [-44.5, 44.5])
def test_inside_90px_keeps_existing_angle_decision(distance, angle):
    decision = LineDecision()

    assert decision.decide(curve_features(distance, angle)) == (
        decision._status_from_curve_angle(angle)
    )


@pytest.mark.parametrize("distance", [-250.0, -130.0, 130.0, 250.0])
@pytest.mark.parametrize("angle", [-44.5, 44.5])
def test_at_and_beyond_130px_keeps_distance_steering(distance, angle):
    decision = LineDecision()
    distance_angle = max(-16.0, min(16.0, math.degrees(math.atan(distance / 700.0))))

    assert decision.decide(curve_features(distance, angle)) == (
        decision._status_from_curve_angle(angle + distance_angle)
    )


@pytest.mark.parametrize("distance, angle, expected", [
    (179.1, -44.5, LineStatus.Left_Turn),
    (-141.8, 41.4, LineStatus.Right_Turn),
])
def test_logged_far_offset_turns_are_unchanged(distance, angle, expected):
    status, _ = LineDecision().decide(curve_features(distance, angle))

    assert status == expected


@pytest.mark.parametrize("point_count, curve_a", [
    (2, 2.0e-3),
    (3, 2.0e-3),
    (8, 0.0),
    (8, 2.0e-4),
    (8, -2.0e-4),
])
@pytest.mark.parametrize("distance, angle, expected", [
    (100.0, -44.5, LineStatus.Right_Half_Forward),
    (-100.0, 44.5, LineStatus.Left_Half_Forward),
])
def test_straight_and_low_point_count_decisions_are_unchanged(
    point_count, curve_a, distance, angle, expected
):
    features = curve_features(distance, angle)
    features.point_count = point_count
    features.curve_a = curve_a

    assert LineDecision().decide(features) == (expected, 0.0)


@pytest.mark.parametrize("point_count, expected", [
    (0, LineStatus.Line_Lost),
    (1, LineStatus.Right_Turn),
])
def test_lost_and_single_point_follow_are_unchanged(point_count, expected):
    features = curve_features(100.0, -44.5)
    features.point_count = point_count
    features.follow_angle = 60.0

    status, _ = LineDecision().decide(features)

    assert status == expected


def test_logged_three_results_now_agree_without_changing_majority_vote():
    decision = LineDecision()
    samples = [(102.3, -44.2), (100.8, -44.1), (-2.4, -40.3)]

    statuses = [
        decision.decide(curve_features(distance, angle))[0]
        for distance, angle in samples
    ]

    assert statuses == [LineStatus.Left_Turn_Curve] * 3
    assert Counter(statuses).most_common(1)[0] == (LineStatus.Left_Turn_Curve, 3)


@pytest.mark.parametrize("distance, angle", [(95.0, -15.0), (-95.0, 15.0)])
def test_existing_small_angle_conflict_steering_is_unchanged(distance, angle):
    decision = LineDecision()
    expected_angle = angle + math.degrees(math.atan(distance / 700.0))

    assert decision.decide(curve_features(distance, angle)) == (
        decision._status_from_curve_angle(expected_angle)
    )


@pytest.mark.parametrize("distance, angle", [(None, -44.5), (100.0, None)])
def test_missing_measurements_do_not_enter_priority_rule(distance, angle):
    decision = LineDecision()
    previous_rule = LineDecision()
    previous_rule.curve_angle_priority_threshold = math.inf
    features = curve_features(distance, angle)

    assert decision.decide(features) == previous_rule.decide(features)
