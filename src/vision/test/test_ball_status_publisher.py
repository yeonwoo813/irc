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

    def test_realsense_center_range_is_minus_10_to_plus_10(self) -> None:
        cases = (
            (-10.01, BallStatus.Left_Half_Forward, -10.01),
            (-10.0, BallStatus.Forward_4step, 0.0),
            (10.0, BallStatus.Forward_4step, 0.0),
            (10.01, BallStatus.Right_Half_Forward, 10.01),
        )

        for input_angle, expected_status, expected_angle in cases:
            with self.subTest(angle=input_angle):
                result = self.decision.decide(
                    BallFeatures(
                        realsense_ball_detected=True,
                        realsense_ball_distance_cm=50.0,
                        realsense_ball_angle_error=input_angle,
                    )
                )
                self.assertEqual(
                    result,
                    (expected_status, expected_angle),
                )

    def test_goal_pre_shoot_range_extends_through_80_cm(self) -> None:
        at_boundary = self.decision.decide(
            BallFeatures(
                ball_in_hand=True,
                realsense_goal_distance_cm=80.0,
                realsense_goal_angle=0.0,
            )
        )
        outside_boundary = self.decision.decide(
            BallFeatures(
                ball_in_hand=True,
                realsense_goal_distance_cm=80.01,
                realsense_goal_angle=0.0,
            )
        )

        self.assertEqual(at_boundary, (BallStatus.Shoot, 0.0))
        self.assertEqual(
            outside_boundary,
            (BallStatus.Forward_4step, 0.0),
        )

    def test_goal_approach_uses_asymmetric_center_boundaries(self) -> None:
        cases = (
            (-60.01, BallStatus.Left_Turn, -60.01),
            (-60.0, BallStatus.Left_Half_Forward, -60.0),
            (-5.01, BallStatus.Left_Half_Forward, -5.01),
            (-5.0, BallStatus.Forward_4step, 0.0),
            (5.0, BallStatus.Forward_4step, 0.0),
            (5.01, BallStatus.Right_Half_Forward, 5.01),
            (60.0, BallStatus.Right_Half_Forward, 60.0),
            (60.01, BallStatus.Right_Turn, 60.01),
        )

        for angle, expected_status, expected_angle in cases:
            with self.subTest(angle=angle):
                self.assertEqual(
                    self.decision._goal_status_from_angle(angle),
                    (expected_status, expected_angle),
                )


class BallVisionFusionWebcamTest(unittest.TestCase):
    def test_webcam_state_passes_signed_x_and_positive_y_distance(self) -> None:
        harness = SimpleNamespace(
            ball_detection_active=True,
            webcam_robot_center_x=320.0,
            webcam_robot_center_y=420.0,
            webcam_fov_x_deg=60.0,
            webcam_frame_width=640.0,
            _finite=BallVisionFusionNode._finite,
            _empty_webcam_state=BallVisionFusionNode._empty_webcam_state,
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

        self.assertEqual(
            harness.latest_webcam["webcam_ball_x_distance"], -40.0
        )
        self.assertEqual(
            harness.latest_webcam["webcam_ball_y_distance"], 78.0
        )
        self.assertEqual(harness.latest_webcam["raw_ball_x"], 280.0)
        self.assertEqual(harness.latest_webcam["raw_ball_y"], 342.0)

    def test_hoop_state_keeps_raw_goal_center_pixels(self) -> None:
        harness = SimpleNamespace(
            _finite=BallVisionFusionNode._finite,
            _empty_hoop_state=BallVisionFusionNode._empty_hoop_state,
        )
        message = String(
            data=json.dumps(
                {
                    "detected": True,
                    "center_x": 415.5,
                    "center_y": 128.25,
                    "realsense_goal_distance_cm": 72.0,
                    "realsense_goal_angle": -1.75,
                    "stamp_sec": 123.5,
                    "raw_detected": True,
                    "held_previous_detection": False,
                }
            )
        )

        BallVisionFusionNode.cb_hoop_state(harness, message)

        self.assertEqual(harness.latest_hoop["realsense_goal_x_px"], 415.5)
        self.assertEqual(harness.latest_hoop["realsense_goal_y_px"], 128.25)
        self.assertEqual(
            harness.latest_hoop["realsense_goal_frame_stamp_sec"], 123.5
        )
        self.assertTrue(
            harness.latest_hoop["realsense_goal_raw_detected"]
        )
        self.assertFalse(
            harness.latest_hoop[
                "realsense_goal_held_previous_detection"
            ]
        )

    def test_reset_generation_rejects_old_source_messages(self) -> None:
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg.data)

        recorder = Recorder()
        harness = SimpleNamespace(
            required_mission_reset_token='new-start',
            mission_reset_waiting=True,
            mission_reset_sources_seen=set(),
            mission_reset_ack_pub=recorder,
            get_logger=lambda: type(
                'Logger',
                (),
                {'info': lambda *_args: None},
            )(),
        )

        self.assertFalse(
            BallVisionFusionNode._accept_mission_reset_source(
                harness,
                'webcam',
                {'mission_reset_token': 'old-start'},
            )
        )
        self.assertTrue(harness.mission_reset_waiting)
        self.assertEqual(recorder.messages, [])

        for source in ('webcam', 'realsense'):
            self.assertTrue(
                BallVisionFusionNode._accept_mission_reset_source(
                    harness,
                    source,
                    {'mission_reset_token': 'new-start'},
                )
            )

        self.assertFalse(harness.mission_reset_waiting)
        self.assertEqual(recorder.messages, ['ball_fusion|new-start'])


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

    def _send_motion_end(self, motion_end):
        self.node.callbacks["motion_end"](
            SimpleNamespace(motion_end=motion_end)
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

    def test_disabled_detection_does_not_accumulate_webcam_votes(self):
        self.publisher.set_detection_enabled(False)

        results = [self._publish(True) for _ in range(5)]

        self.assertEqual(results, [(BallStatus.Ball_None, 0.0)] * 5)
        self.assertEqual(list(self.publisher.webcam_detection_buffer), [])
        self.assertFalse(self.publisher.webcam_ball_confirmed)

        self.publisher.set_detection_enabled(True)
        self._confirm_webcam_ball()
        self.assertTrue(self.publisher.webcam_ball_confirmed)

    def test_mission_start_reset_clears_every_prestore_latch(self):
        self._confirm_webcam_ball()
        self.publisher.back_to_initial_done = True
        self.publisher.pick_command_seen = True
        self.publisher.ball_in_hand = True
        self.publisher.shoot_initial_waiting = True
        self.publisher.shoot_initial_done = True
        self.publisher.shoot_command_seen = True
        self.publisher.pre_shoot_distance_buffer.extend([70.0, 71.0])

        self.publisher.reset_for_mission_start()

        self.assertEqual(list(self.publisher.webcam_detection_buffer), [])
        self.assertFalse(self.publisher.webcam_ball_confirmed)
        self.assertFalse(self.publisher.back_to_initial_waiting)
        self.assertFalse(self.publisher.back_to_initial_done)
        self.assertFalse(self.publisher.pick_command_seen)
        self.assertFalse(self.publisher.ball_in_hand)
        self.assertFalse(self.publisher.shoot_initial_waiting)
        self.assertFalse(self.publisher.shoot_initial_done)
        self.assertFalse(self.publisher.shoot_command_seen)
        self.assertEqual(list(self.publisher.pre_shoot_distance_buffer), [])

    def test_goal_distance_is_published_for_decision_threshold(self):
        self.publisher.publish_ball_status(
            realsense_goal_distance_cm=89.5,
            realsense_goal_angle=0.0,
            realsense_goal_x_px=401.25,
            realsense_goal_y_px=122.75,
        )

        self.assertAlmostEqual(
            self.node.recorder.messages[-1].goal_distance_cm,
            89.5,
        )
        self.assertAlmostEqual(
            self.node.recorder.messages[-1].goal_x_px,
            401.25,
        )
        self.assertAlmostEqual(
            self.node.recorder.messages[-1].goal_y_px,
            122.75,
        )

    def test_ball_raw_pixel_coordinates_are_published(self):
        self.publisher.publish_ball_status(
            webcam_ball_x_px=400.0,
            webcam_ball_y_px=300.0,
        )

        self.assertAlmostEqual(
            self.node.recorder.messages[-1].ball_x_px,
            400.0,
        )
        self.assertAlmostEqual(
            self.node.recorder.messages[-1].ball_y_px,
            300.0,
        )

    def test_goal_pre_shoot_requests_initial_pose_once_within_80_cm(self):
        self.publisher.ball_in_hand = True

        results = [
            self.publisher.publish_ball_status(
                realsense_goal_distance_cm=distance,
                realsense_goal_angle=0.0,
                realsense_goal_frame_stamp_sec=stamp,
                realsense_goal_raw_detected=True,
            )
            for stamp, distance in enumerate((81.0, 80.0, 79.0), 1)
        ]
        self.assertEqual(
            results,
            [
                (BallStatus.Forward_4step, 0.0),
                (BallStatus.Forward_4step, 0.0),
                (BallStatus.Back_To_Initial, 0.0),
            ],
        )

        self._send_motion(BallStatus.Back_To_Initial)
        after_initial = self.publisher.publish_ball_status(
            realsense_goal_distance_cm=80.0,
            realsense_goal_angle=0.0,
        )
        self.assertEqual(after_initial, (BallStatus.Shoot, 0.0))
        self.assertFalse(
            self.node.recorder.messages[-1].pre_shoot_verified
        )

        self._send_motion_end(False)
        self._send_motion_end(True)
        verified = self.publisher.publish_ball_status(
            realsense_goal_distance_cm=80.0,
            realsense_goal_angle=0.0,
        )
        self.assertEqual(verified, (BallStatus.Shoot, 0.0))
        self.assertTrue(
            self.node.recorder.messages[-1].pre_shoot_verified
        )

        self.publisher.publish_ball_status(
            realsense_goal_distance_cm=80.0,
            realsense_goal_angle=0.0,
        )
        self.assertFalse(
            self.node.recorder.messages[-1].pre_shoot_verified
        )

    def test_too_close_goal_still_sets_pose_before_backward(self):
        self.publisher.ball_in_hand = True

        results = [
            self.publisher.publish_ball_status(
                realsense_goal_distance_cm=50.0,
                realsense_goal_angle=0.0,
                realsense_goal_frame_stamp_sec=float(stamp),
                realsense_goal_raw_detected=True,
            )
            for stamp in (1, 2, 3)
        ]
        self.assertEqual(
            results[-1], (BallStatus.Back_To_Initial, 0.0)
        )

        self._send_motion(BallStatus.Back_To_Initial)
        after_initial = self.publisher.publish_ball_status(
            realsense_goal_distance_cm=50.0,
            realsense_goal_angle=0.0,
        )
        self.assertEqual(after_initial, (BallStatus.Backward_half, 0.0))

    def test_single_low_distance_outlier_does_not_request_initial_pose(self):
        self.publisher.ball_in_hand = True

        results = [
            self.publisher.publish_ball_status(
                realsense_goal_distance_cm=distance,
                realsense_goal_angle=0.0,
                realsense_goal_frame_stamp_sec=stamp,
                realsense_goal_raw_detected=True,
            )
            for stamp, distance in enumerate((90.0, 79.0, 88.0), 1)
        ]

        self.assertNotIn(
            (BallStatus.Back_To_Initial, 0.0), results
        )
        self.assertFalse(self.publisher.shoot_initial_waiting)

    def test_duplicate_and_held_goal_frames_are_not_counted(self):
        self.publisher.ball_in_hand = True

        first = self.publisher.publish_ball_status(
            realsense_goal_distance_cm=79.0,
            realsense_goal_angle=0.0,
            realsense_goal_frame_stamp_sec=1.0,
            realsense_goal_raw_detected=True,
        )
        duplicate = self.publisher.publish_ball_status(
            realsense_goal_distance_cm=79.0,
            realsense_goal_angle=0.0,
            realsense_goal_frame_stamp_sec=1.0,
            realsense_goal_raw_detected=True,
        )
        held = self.publisher.publish_ball_status(
            realsense_goal_distance_cm=79.0,
            realsense_goal_angle=0.0,
            realsense_goal_frame_stamp_sec=2.0,
            realsense_goal_raw_detected=False,
            realsense_goal_held_previous_detection=True,
        )

        self.assertEqual(
            [first, duplicate, held],
            [(BallStatus.Forward_4step, 0.0)] * 3,
        )
        self.assertEqual(
            list(self.publisher.pre_shoot_distance_buffer), [79.0]
        )
        self.assertFalse(self.publisher.shoot_initial_waiting)

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
