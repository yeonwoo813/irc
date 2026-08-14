#!/usr/bin/env python3
"""Tests for the actual-motion overlay shown on the webcam YOLO image."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from yolo_detector import (  # noqa: E402
    MotionDisplayState,
    add_ball_geometry,
    motion_overlay_lines,
)
from ball_status_publisher import BallStatus, BallStatusPublisher  # noqa: E402


class MotionDisplayStateTest(unittest.TestCase):
    def test_command_is_shown_only_after_motion_starts(self) -> None:
        state = MotionDisplayState()

        state.on_command(6)
        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:-- UNKNOWN", "run:IDLE ready:0"],
        )

        state.on_motion_state(motion_end=False, motion_ready=True)
        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:6 Left_Turn", "run:RUNNING ready:1"],
        )

        state.on_motion_state(motion_end=True, motion_ready=True)
        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:6 Left_Turn", "run:IDLE ready:1"],
        )

    def test_motion_state_may_arrive_before_command(self) -> None:
        state = MotionDisplayState()

        state.on_motion_state(motion_end=False, motion_ready=True)
        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:-- UNKNOWN", "run:RUNNING ready:1"],
        )

        state.on_command(19)
        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:19 Hurdle_Go", "run:RUNNING ready:1"],
        )

    def test_unknown_motion_id_keeps_numeric_command(self) -> None:
        state = MotionDisplayState()
        state.on_command(88)
        state.on_motion_state(motion_end=False, motion_ready=True)

        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:88 Unknown", "run:RUNNING ready:1"],
        )


class BallGeometryTest(unittest.TestCase):
    def test_geometry_uses_center_plus_25_and_bottom_of_frame(self) -> None:
        payload = add_ball_geometry(
            {
                "ball_detected": True,
                "ball_x": 400.0,
                "ball_y": 300.0,
            },
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["robot_center_x"], 345.0)
        self.assertEqual(payload["robot_center_y"], 479.0)
        self.assertEqual(payload["ball_x_offset_px"], 55.0)
        self.assertEqual(payload["ball_x_distance_px"], 55.0)
        self.assertEqual(payload["ball_y_distance_px"], 179.0)
        self.assertAlmostEqual(payload["ball_distance_px"], 187.259, places=3)
        self.assertAlmostEqual(payload["ball_angle_deg"], 17.080, places=3)

    def test_geometry_is_empty_when_ball_is_not_detected(self) -> None:
        payload = add_ball_geometry(
            {"ball_detected": False},
            frame_w=640,
            frame_h=480,
        )

        self.assertIsNone(payload["ball_x_distance_px"])
        self.assertIsNone(payload["ball_y_distance_px"])
        self.assertIsNone(payload["ball_angle_deg"])


class _FakePublisher:
    def __init__(self) -> None:
        self.last_message = None

    def publish(self, message) -> None:
        self.last_message = message


class _FakeNode:
    def __init__(self) -> None:
        self.publisher = _FakePublisher()

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher


class BallStatusPublisherTest(unittest.TestCase):
    def test_publishes_measured_angle_and_xy_distances(self) -> None:
        node = _FakeNode()
        publisher = BallStatusPublisher(node)

        status, angle = publisher.publish_ball_status(
            webcam_ball_detected=True,
            webcam_ball_x_offset=55.0,
            webcam_ball_x_distance=55.0,
            webcam_ball_y_distance=179.0,
            webcam_ball_angle_error=17.08,
            webcam_ball_distance_px=187.259,
        )

        self.assertEqual(status, BallStatus.Right_Move)
        self.assertAlmostEqual(angle, 17.08, places=2)
        self.assertAlmostEqual(node.publisher.last_message.detected_angle, 17.08)
        self.assertAlmostEqual(node.publisher.last_message.x_distance_px, 55.0)
        self.assertAlmostEqual(node.publisher.last_message.y_distance_px, 179.0)

    def test_x_distance_is_absolute_but_offset_keeps_left_direction(self) -> None:
        payload = add_ball_geometry(
            {
                "ball_detected": True,
                "ball_x": 300.0,
                "ball_y": 300.0,
            },
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["ball_x_offset_px"], -45.0)
        self.assertEqual(payload["ball_x_distance_px"], 45.0)
        self.assertEqual(payload["ball_y_distance_px"], 179.0)

        node = _FakeNode()
        publisher = BallStatusPublisher(node)
        status, _angle = publisher.publish_ball_status(
            webcam_ball_detected=True,
            webcam_ball_x_offset=payload["ball_x_offset_px"],
            webcam_ball_x_distance=payload["ball_x_distance_px"],
            webcam_ball_y_distance=payload["ball_y_distance_px"],
            webcam_ball_angle_error=payload["ball_angle_deg"],
            webcam_ball_distance_px=payload["ball_distance_px"],
        )

        self.assertEqual(status, BallStatus.Left_Move)
        self.assertEqual(node.publisher.last_message.x_distance_px, 45.0)
        self.assertEqual(node.publisher.last_message.y_distance_px, 179.0)


if __name__ == "__main__":
    unittest.main()
