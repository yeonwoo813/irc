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
    BallStatusPublisher,
)
from ball_vision_fusion import BallVisionFusionNode  # noqa: E402


class BallDecisionTest(unittest.TestCase):
    @staticmethod
    def _features(**overrides) -> BallFeatures:
        values = {
            "webcam_ball_detected": True,
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
                webcam_ball_x_distance=100.0,
                webcam_ball_y_distance=78.01,
                webcam_ball_angle_error=0.0,
            )
        )

        self.assertEqual(result, (BallStatus.Forward_half, 0.0))

    def test_far_ball_uses_angle_boundaries(self) -> None:
        cases = (
            (-4.01, BallStatus.Left_Turn_5, -4.01),
            (-4.0, BallStatus.Forward_half, 0.0),
            (4.0, BallStatus.Forward_half, 0.0),
            (4.01, BallStatus.Right_Turn_5, 4.01),
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

        for x_distance, expected_status, expected_angle in cases:
            with self.subTest(x_distance=x_distance):
                result = self.decision.decide(
                    self._features(
                        webcam_ball_x_distance=x_distance,
                        webcam_ball_y_distance=78.0,
                        webcam_ball_angle_error=-2.0,
                    )
                )
                self.assertEqual(result, (expected_status, expected_angle))

    def test_missing_webcam_axis_does_not_issue_motion(self) -> None:
        cases = (
            {"webcam_ball_x_distance": None},
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
    def test_webcam_state_passes_signed_x_and_positive_y_distance(self) -> None:
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
                    "ball_x_distance_px": -40.0,
                    "ball_y_distance_px": 78.0,
                    "ball_distance_px": 87.66,
                }
            )
        )

        BallVisionFusionNode.cb_webcam_state(harness, message)

        self.assertEqual(harness.latest_webcam["webcam_ball_x_distance"], -40.0)
        self.assertEqual(harness.latest_webcam["webcam_ball_y_distance"], 78.0)


class BallStatusPublisherWebcamMajorityTest(unittest.TestCase):
    class Recorder:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    class Logger:
        def __init__(self):
            self.messages = []

        def info(self, message):
            self.messages.append(message)

    class FakeNode:
        def __init__(self):
            self.recorder = BallStatusPublisherWebcamMajorityTest.Recorder()
            self.logger = BallStatusPublisherWebcamMajorityTest.Logger()
            self.callbacks = {}

        def create_publisher(self, _msg_type, _topic_name, _depth):
            return self.recorder

        def create_subscription(
            self,
            _msg_type,
            topic_name,
            callback,
            _depth,
        ):
            self.callbacks[topic_name] = callback
            return object()

        def get_logger(self):
            return self.logger

    def setUp(self) -> None:
        self.node = self.FakeNode()
        self.publisher = BallStatusPublisher(self.node)

    def _publish(self, detected, ball_in_hand=False):
        return self.publisher.publish_ball_status(
            webcam_ball_detected=detected,
            webcam_ball_x_distance=0.0,
            webcam_ball_y_distance=100.0,
            webcam_ball_angle_error=0.0,
            ball_in_hand=ball_in_hand,
        )

    def _send_motion(self, command):
        self.node.callbacks["motion_command"](
            SimpleNamespace(command=command)
        )

    def _confirm_webcam_ball(self):
        return [
            self._publish(detected)
            for detected in (True, False, True, False, True)
        ]

    def test_three_of_five_webcam_detections_request_initial_pose(self):
        results = self._confirm_webcam_ball()

        self.assertEqual(
            results[:4],
            [(BallStatus.Ball_None, 0.0)] * 4,
        )
        self.assertEqual(
            results[4],
            (BallStatus.Back_To_Initial, 0.0),
        )
        self.assertTrue(self.publisher.webcam_ball_confirmed)
        self.assertTrue(self.publisher.back_to_initial_waiting)

    def test_two_of_five_webcam_detections_do_not_request_initial_pose(self):
        results = [
            self._publish(detected)
            for detected in (True, False, True, False, False)
        ]

        self.assertEqual(results, [(BallStatus.Ball_None, 0.0)] * 5)
        self.assertFalse(self.publisher.webcam_ball_confirmed)
        self.assertFalse(self.publisher.back_to_initial_waiting)

    def test_initial_pose_is_locked_after_motion_command_confirmation(self):
        self._confirm_webcam_ball()
        self.assertEqual(
            self._publish(False),
            (BallStatus.Back_To_Initial, 0.0),
        )

        self._send_motion(BallStatus.Back_To_Initial)

        self.assertTrue(self.publisher.back_to_initial_done)
        self.assertFalse(self.publisher.back_to_initial_waiting)
        self.assertEqual(
            self._publish(True),
            (BallStatus.Forward_half, 0.0),
        )

    def test_lock_is_released_only_after_pick_result_check_command(self):
        self._confirm_webcam_ball()
        self._send_motion(BallStatus.Back_To_Initial)
        self._send_motion(BallStatus.Pick_Ready)

        self.assertTrue(self.publisher.pick_command_seen)
        self.assertTrue(self.publisher.back_to_initial_done)

        self._send_motion(BallStatus.Forward_half)
        self.assertTrue(self.publisher.back_to_initial_done)

        self._send_motion(BallStatus.Neck_Up)
        self.assertFalse(self.publisher.pick_command_seen)
        self.assertFalse(self.publisher.back_to_initial_done)
        self.assertFalse(self.publisher.webcam_ball_confirmed)
        self.assertEqual(list(self.publisher.webcam_detection_buffer), [])

    def test_failed_pick_turn_also_releases_lock(self):
        self._confirm_webcam_ball()
        self._send_motion(BallStatus.Back_To_Initial)
        self._send_motion(BallStatus.Pick_Ready)

        self._send_motion(BallStatus.Right_Turn_Afterpick)

        self.assertFalse(self.publisher.pick_command_seen)
        self.assertFalse(self.publisher.back_to_initial_done)

    def test_failed_pick_backward_also_releases_lock(self):
        self._confirm_webcam_ball()
        self._send_motion(BallStatus.Back_To_Initial)
        self._send_motion(BallStatus.Pick_Ready)

        self._send_motion(BallStatus.Backward_half)

        self.assertFalse(self.publisher.pick_command_seen)
        self.assertFalse(self.publisher.back_to_initial_done)

    def test_ball_in_hand_is_not_counted_as_next_webcam_detection(self):
        self._confirm_webcam_ball()
        self._send_motion(BallStatus.Back_To_Initial)
        self._send_motion(BallStatus.Pick_Ready)

        # Pick 이후 true를 한 번 확인하면 공 소유 상태를 고정한다.
        self._publish(True, ball_in_hand=True)
        self._send_motion(BallStatus.Neck_Up)

        results = [
            self._publish(True, ball_in_hand=False)
            for _ in range(5)
        ]

        self.assertEqual(results, [(BallStatus.Ball_None, 0.0)] * 5)
        self.assertTrue(self.publisher.ball_in_hand)
        self.assertTrue(self.node.recorder.messages[-1].ball_in_hand)
        self.assertFalse(self.publisher.webcam_ball_confirmed)
        self.assertEqual(
            list(self.publisher.webcam_detection_buffer),
            [False] * 5,
        )

    def test_ball_in_hand_is_released_after_shoot_completes(self):
        self._confirm_webcam_ball()
        self._send_motion(BallStatus.Back_To_Initial)
        self._send_motion(BallStatus.Pick_Ready)
        self._publish(False, ball_in_hand=True)
        self._send_motion(BallStatus.Neck_Up)

        self._publish(False, ball_in_hand=False)
        self.assertTrue(self.publisher.ball_in_hand)
        self.assertTrue(self.node.recorder.messages[-1].ball_in_hand)

        self._send_motion(BallStatus.Shoot)
        self.assertTrue(self.publisher.ball_in_hand)
        self._send_motion(BallStatus.Neck_Down)

        self.assertFalse(self.publisher.ball_in_hand)
        self._publish(False, ball_in_hand=False)
        self.assertFalse(self.node.recorder.messages[-1].ball_in_hand)


if __name__ == "__main__":
    unittest.main()
