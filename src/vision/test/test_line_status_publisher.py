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

    def _curve_features(self, distance: float) -> LineFeatures:
        return LineFeatures(
            point_count=5,
            line_angle=-35.0,
            curve_a=2.0e-3,
            tangent_angle=-35.0,
            line_distance=distance,
        )

    def test_curve_far_right_prioritizes_right_distance_correction(self) -> None:
        status, angle = self.decision.decide(
            self._curve_features(self.decision.curve_distance)
        )

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_curve_far_left_prioritizes_left_distance_correction(self) -> None:
        status, angle = self.decision.decide(
            self._curve_features(-self.decision.curve_distance)
        )

        self.assertEqual(status, LineStatus.Left_Half_Forward)
        self.assertEqual(angle, 0.0)

    def test_curve_inside_distance_limit_uses_tangent_angle(self) -> None:
        status, angle = self.decision.decide(
            self._curve_features(self.decision.curve_distance - 1.0)
        )

        self.assertEqual(status, LineStatus.Left_Turn_Curve)
        self.assertEqual(angle, 35.0)

    def test_straight_line_keeps_move_distance_limit(self) -> None:
        features = self._curve_features(self.decision.move_distance)
        features.curve_a = 0.0
        features.line_angle = 0.0

        status, angle = self.decision.decide(features)

        self.assertEqual(status, LineStatus.Right_Half_Forward)
        self.assertEqual(angle, 0.0)


if __name__ == "__main__":
    unittest.main()
