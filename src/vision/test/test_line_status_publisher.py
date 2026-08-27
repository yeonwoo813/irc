#!/usr/bin/env python3
"""Focused tests for line status distance and curve priority."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from line_status_publisher import (  # noqa: E402
    LineDecision,
    LineFeatures,
    LineStatus,
)


class CurveDistancePriorityTest(unittest.TestCase):
    def setUp(self) -> None:
        self.decision = LineDecision()

    def test_straight_and_curve_far_steering_thresholds_are_130px(self) -> None:
        self.assertEqual(self.decision.steering_distance_max, 130.0)
        self.assertEqual(
            self.decision.curve_steering_distance_max,
            130.0,
        )

    def _curve_features(self, distance: float) -> LineFeatures:
        return LineFeatures(
            point_count=5,
            line_angle=-35.0,
            curve_a=2.0e-3,
            tangent_angle=-35.0,
            line_distance=distance,
        )

    def test_curve_a_threshold_rejects_borderline_straight_fit(self) -> None:
        features = self._curve_features(0.0)
        features.curve_a = 1.03e-4
        features.line_angle = 34.1
        features.tangent_angle = 34.1

        status, angle = self.decision.decide(features)

        self.assertEqual(self.decision.curve_a, 1.2e-4)
        self.assertEqual(status, LineStatus.Right_Turn)
        self.assertAlmostEqual(angle, 34.1)

    def test_curve_a_above_new_threshold_is_curve(self) -> None:
        features = self._curve_features(0.0)
        features.curve_a = 1.21e-4
        features.line_angle = 34.1
        features.tangent_angle = 34.1

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Turn_Curve)
        self.assertAlmostEqual(angle, 34.1)

    def test_curve_far_right_prioritizes_right_distance_correction(self) -> None:
        status, angle = self.decision.decide(
            self._curve_features(self.decision.curve_move_distance)
        )

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_curve_far_left_prioritizes_left_distance_correction(self) -> None:
        status, angle = self.decision.decide(
            self._curve_features(-self.decision.curve_move_distance)
        )

        self.assertEqual(status, LineStatus.Left_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_curve_inside_distance_limit_uses_tangent_angle(self) -> None:
        status, angle = self.decision.decide(
            self._curve_features(self.decision.curve_move_distance - 1.0)
        )

        self.assertEqual(status, LineStatus.Left_Turn)
        self.assertEqual(angle, 35.0)

    def test_curve_near_opposite_errors_use_combined_steering(self) -> None:
        features = self._curve_features(95.0)
        features.line_angle = -15.0
        features.tangent_angle = -15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Forward_4step)
        self.assertEqual(angle, 0.0)

    def test_curve_below_far_limit_same_direction_keeps_distance_priority(
        self,
    ) -> None:
        features = self._curve_features(
            self.decision.curve_steering_distance_max - 1.0
        )
        features.tangent_angle = 15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_curve_far_same_direction_uses_combined_steering(self) -> None:
        features = self._curve_features(
            self.decision.curve_steering_distance_max
        )
        features.tangent_angle = 15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Turn_Half)
        self.assertEqual(angle, 25.0)

    def test_curve_far_opposite_direction_uses_combined_steering(self) -> None:
        features = self._curve_features(
            self.decision.curve_steering_distance_max
        )
        features.tangent_angle = -15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Forward_4step)
        self.assertEqual(angle, 0.0)

    def test_curve_far_above_30_degrees_uses_combined_steering(self) -> None:
        features = self._curve_features(
            self.decision.curve_steering_distance_max
        )
        features.tangent_angle = 35.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Turn_Curve)
        self.assertEqual(angle, 45.0)

    def test_curve_far_zero_angle_still_uses_distance_steering(self) -> None:
        features = self._curve_features(
            self.decision.curve_steering_distance_max
        )
        features.tangent_angle = 0.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 10.0)

    def test_curve_thresholds_are_independent_from_straight_thresholds(
        self,
    ) -> None:
        self.assertEqual(
            self.decision.curve_forward_angle,
            self.decision.forward_angle,
        )
        self.assertEqual(
            self.decision.curve_fine_turn_angle,
            self.decision.fine_turn_angle,
        )
        self.assertEqual(
            self.decision.curve_half_turn_angle,
            self.decision.half_turn_angle,
        )
        self.assertEqual(
            self.decision.curve_large_turn_angle,
            self.decision.large_turn_angle,
        )

        self.decision.curve_forward_angle = 8.0
        self.decision.curve_fine_turn_angle = 24.0
        self.decision.curve_half_turn_angle = 32.0
        self.decision.curve_large_turn_angle = 50.0
        self.decision.curve_move_distance = 110.0

        self.assertEqual(self.decision.forward_angle, 7.0)
        self.assertEqual(self.decision.fine_turn_angle, 22.5)
        self.assertEqual(self.decision.half_turn_angle, 30.0)
        self.assertEqual(self.decision.large_turn_angle, 45.0)
        self.assertEqual(self.decision.move_distance, 90.0)

    def test_curve_angle_bands_use_curve_thresholds(self) -> None:
        cases = [
            (7.0, LineStatus.Forward_4step),
            (7.1, LineStatus.Right_Half_Forward),
            (22.5, LineStatus.Right_Half_Forward),
            (22.6, LineStatus.Right_Turn_Half),
            (29.9, LineStatus.Right_Turn_Half),
            (30.0, LineStatus.Right_Turn),
            (44.9, LineStatus.Right_Turn),
            (45.0, LineStatus.Right_Turn_Curve),
            (-7.0, LineStatus.Forward_4step),
            (-7.1, LineStatus.Left_Half_Forward),
            (-22.5, LineStatus.Left_Half_Forward),
            (-22.6, LineStatus.Left_Turn_Half),
            (-29.9, LineStatus.Left_Turn_Half),
            (-30.0, LineStatus.Left_Turn),
            (-44.9, LineStatus.Left_Turn),
            (-45.0, LineStatus.Left_Turn_Curve),
        ]

        for angle, expected_status in cases:
            with self.subTest(angle=angle):
                status, _ = self.decision._status_from_curve_angle(angle)
                self.assertEqual(status, expected_status)

    def test_straight_far_same_direction_keeps_distance_priority(self) -> None:
        features = self._curve_features(self.decision.move_distance)
        features.curve_a = 0.0
        features.line_angle = 17.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_straight_inside_distance_limit_uses_only_line_angle(self) -> None:
        features = self._curve_features(59.0)
        features.curve_a = 0.0
        features.line_angle = 17.1

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertAlmostEqual(angle, 17.1)

    def test_straight_conflicting_errors_can_cancel(self) -> None:
        features = self._curve_features(-118.0)
        features.curve_a = 0.0
        features.line_angle = 10.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Forward_4step)
        self.assertEqual(angle, 0.0)

    def test_straight_conflict_includes_90_pixel_boundary(self) -> None:
        features = LineFeatures(
            point_count=3,
            line_angle=-15.0,
            line_distance=self.decision.move_distance,
        )

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Forward_4step)
        self.assertEqual(angle, 0.0)

    def test_straight_conflict_uses_combined_direction(self) -> None:
        features = self._curve_features(118.0)
        features.curve_a = 0.0
        features.line_angle = -20.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Left_Half_Forward)
        expected_angle = abs(
            features.line_angle
            + self.decision.steering_limit
        )
        self.assertAlmostEqual(angle, expected_angle)

    def test_straight_far_conflict_uses_combined_steering(self) -> None:
        features = self._curve_features(
            self.decision.steering_distance_max
        )
        features.curve_a = 0.0
        features.line_angle = -15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Forward_4step)
        self.assertEqual(angle, 0.0)

    def test_straight_below_far_limit_same_direction_keeps_distance_priority(
        self,
    ) -> None:
        features = self._curve_features(
            self.decision.steering_distance_max - 1.0
        )
        features.curve_a = 0.0
        features.line_angle = 15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_straight_far_same_direction_uses_combined_steering(self) -> None:
        features = self._curve_features(
            self.decision.steering_distance_max
        )
        features.curve_a = 0.0
        features.line_angle = 15.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Turn_Half)
        self.assertEqual(angle, 25.0)

    def test_straight_far_above_30_degrees_uses_combined_steering(self) -> None:
        features = self._curve_features(
            self.decision.steering_distance_max
        )
        features.curve_a = 0.0
        features.line_angle = 35.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Turn)
        self.assertEqual(angle, 45.0)

    def test_straight_far_zero_angle_still_uses_distance_steering(self) -> None:
        features = self._curve_features(
            self.decision.steering_distance_max
        )
        features.curve_a = 0.0
        features.line_angle = 0.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 10.0)

    def test_straight_turn_half_does_not_use_steering(self) -> None:
        features = self._curve_features(118.0)
        features.curve_a = 0.0
        features.line_angle = -(self.decision.half_turn_angle - 0.1)

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_straight_opposite_half_forward_uses_steering(self) -> None:
        features = self._curve_features(118.0)
        features.curve_a = 0.0
        features.line_angle = -self.decision.fine_turn_angle

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Left_Half_Forward)
        self.assertAlmostEqual(
            angle,
            self.decision.fine_turn_angle
            - self.decision.steering_limit,
        )

    def test_straight_conflict_above_30_degrees_uses_distance(self) -> None:
        features = self._curve_features(118.0)
        features.curve_a = 0.0
        features.line_angle = -(self.decision.half_turn_angle + 0.1)

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_straight_small_opposite_angle_keeps_distance_priority(self) -> None:
        features = self._curve_features(self.decision.move_distance)
        features.curve_a = 0.0
        features.line_angle = -3.5

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_straight_without_distance_keeps_angle_decision(self) -> None:
        features = self._curve_features(0.0)
        features.curve_a = 0.0
        features.line_angle = -15.0
        features.line_distance = None

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Left_Half_Forward)
        self.assertEqual(angle, 15.0)

    def test_straight_angle_bands(self) -> None:
        cases = [
            (7.0, LineStatus.Forward_4step),
            (7.1, LineStatus.Right_Half_Forward),
            (22.5, LineStatus.Right_Half_Forward),
            (22.6, LineStatus.Right_Turn_Half),
            (29.9, LineStatus.Right_Turn_Half),
            (30.0, LineStatus.Right_Turn),
            (30.1, LineStatus.Right_Turn),
            (-7.0, LineStatus.Forward_4step),
            (-7.1, LineStatus.Left_Half_Forward),
            (-22.5, LineStatus.Left_Half_Forward),
            (-22.6, LineStatus.Left_Turn_Half),
            (-29.9, LineStatus.Left_Turn_Half),
            (-30.0, LineStatus.Left_Turn),
            (-30.1, LineStatus.Left_Turn),
        ]

        for angle, expected_status in cases:
            with self.subTest(angle=angle):
                status, _ = self.decision._status_from_line_angle(angle)
                self.assertEqual(status, expected_status)

    def test_one_point_keeps_follow_angle_turn_logic(self) -> None:
        features = LineFeatures(
            point_count=1,
            line_angle=0.0,
            line_distance=68.0,
            follow_angle=-0.3,
        )

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Left_Turn)
        self.assertAlmostEqual(angle, 0.3)

    def test_two_points_use_straight_steering_conflict(self) -> None:
        features = LineFeatures(
            point_count=2,
            line_angle=-15.0,
            line_distance=100.0,
            follow_angle=0.3,
        )

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Forward_4step)
        self.assertEqual(angle, 0.0)


if __name__ == "__main__":
    unittest.main()
