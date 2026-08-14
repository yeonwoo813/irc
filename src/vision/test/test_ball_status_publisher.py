#!/usr/bin/env python3
"""Tests for webcam ball approach decisions and fusion feature conversion."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from std_msgs.msg import String


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from ball_status_publisher import (  # noqa: E402
    BallDecision,
    BallFeatures,
    BallStatus,
)
from ball_vision_fusion import BallVisionFusionNode  # noqa: E402


class BallDecisionTest(unittest.TestCase):
    @staticmethod
    def _features(**overrides) -> BallFeatures:
        values = {
            "webcam_ball_detected": True,
            "webcam_ball_x_offset": 0.0,
            "webcam_ball_x_distance": 0.0,
            "webcam_ball_y_distance": 100.0,
            "webcam_ball_angle_error": 0.0,
        }
        values.update(overrides)
        return BallFeatures(**values)

    def setUp(self) -> None:
        self.decision = BallDecision()

    def test_y_distance_has_priority_while_ball_is_far(self) -> None:
        result = self.decision.decide(
            self._features(
                webcam_ball_x_offset=100.0,
                webcam_ball_x_distance=100.0,
                webcam_ball_y_distance=78.01,
                webcam_ball_angle_error=0.0,
            )
        )

        self.assertEqual(result, (BallStatus.Forward_half, 0.0))

    def test_far_ball_uses_angle_boundaries(self) -> None:
        cases = (
            (-4.01, BallStatus.Left_Turn_Ball, -4.01),
            (-4.0, BallStatus.Forward_half, 0.0),
            (4.0, BallStatus.Forward_half, 0.0),
            (4.01, BallStatus.Right_Turn_Ball, 4.01),
        )

        for input_angle, expected_status, expected_angle in cases:
            with self.subTest(angle=input_angle):
                result = self.decision.decide(
                    self._features(webcam_ball_angle_error=input_angle)
                )
                self.assertEqual(result, (expected_status, expected_angle))

    def test_pick_distance_uses_signed_x_boundaries(self) -> None:
        cases = (
            (-40.01, BallStatus.Left_Move, -2.0),
            (-40.0, BallStatus.Pick_Ready, 0.0),
            (35.0, BallStatus.Pick_Ready, 0.0),
            (35.01, BallStatus.Right_Move, -2.0),
        )

        for x_offset, expected_status, expected_angle in cases:
            with self.subTest(x_offset=x_offset):
                result = self.decision.decide(
                    self._features(
                        webcam_ball_x_offset=x_offset,
                        webcam_ball_x_distance=abs(x_offset),
                        webcam_ball_y_distance=78.0,
                        webcam_ball_angle_error=-2.0,
                    )
                )
                self.assertEqual(result, (expected_status, expected_angle))

    def test_missing_webcam_axis_does_not_issue_motion(self) -> None:
        cases = (
            {
                "webcam_ball_x_offset": None,
                "webcam_ball_x_distance": None,
            },
            {"webcam_ball_y_distance": None},
        )

        for missing_axis in cases:
            with self.subTest(missing_axis=missing_axis):
                self.assertEqual(
                    self.decision.decide(self._features(**missing_axis)),
                    (BallStatus.Ball_None, 0.0),
                )

    def test_realsense_decision_is_unchanged(self) -> None:
        result = self.decision.decide(
            BallFeatures(
                realsense_ball_detected=True,
                realsense_ball_distance_cm=50.0,
                realsense_ball_angle_error=-5.0,
            )
        )
        self.assertEqual(result, (BallStatus.Forward_3step, 0.0))


class BallVisionFusionWebcamTest(unittest.TestCase):
    def test_webcam_state_passes_x_and_positive_y_distance(self) -> None:
        harness = SimpleNamespace(
            webcam_robot_center_x=320.0,
            webcam_robot_center_y=420.0,
            webcam_fov_x_deg=60.0,
            webcam_frame_width=640.0,
        )
        message = String(
            data=json.dumps(
                {
                    "ball_detected": True,
                    "ball_x": 280.0,
                    "ball_y": 342.0,
                    "ball_conf": 0.9,
                    "ball_bbox": [270, 332, 290, 352],
                    "ball_x_offset_px": -40.0,
                    "ball_x_distance_px": 40.0,
                    "ball_y_distance_px": 78.0,
                    "ball_distance_px": 87.66,
                }
            )
        )

        BallVisionFusionNode.cb_webcam_state(harness, message)

        self.assertEqual(harness.latest_webcam["webcam_ball_x_offset"], -40.0)
        self.assertEqual(harness.latest_webcam["webcam_ball_x_distance"], 40.0)
        self.assertEqual(harness.latest_webcam["webcam_ball_y_distance"], 78.0)


if __name__ == "__main__":
    unittest.main()
