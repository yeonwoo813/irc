#!/usr/bin/env python3
"""Tests for the actual-motion overlay shown on the webcam YOLO image."""

from pathlib import Path
import math
import sys
from types import SimpleNamespace
import unittest

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from yolo_detector import (  # noqa: E402
    LineStatus,
    MotionDisplayState,
    add_ball_geometry,
    apply_line_status,
    make_line_payload,
    make_vision_payload,
    motion_overlay_lines,
)
from ball_status_publisher import BallStatus, BallStatusPublisher  # noqa: E402


class MotionDisplayStateTest(unittest.TestCase):
    def test_turn_half_name_is_shown(self) -> None:
        state = MotionDisplayState()
        state.on_command(4)
        state.on_motion_state(motion_end=False, motion_ready=True)

        self.assertEqual(
            motion_overlay_lines(state),
            ["motion:4 Left_Turn_Half", "run:RUNNING ready:1"],
        )

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


class LineGeometryTest(unittest.TestCase):
    def test_line_uses_same_center_plus_25_as_ball(self) -> None:
        payload = make_line_payload(
            line_points=[(345.0, 400.0)],
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["line_distance"], 0.0)
        self.assertEqual(payload["follow_angle"], 0.0)

    def test_line_at_image_center_is_left_of_calibrated_center(self) -> None:
        payload = make_line_payload(
            line_points=[(320.0, 400.0)],
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["line_distance"], -25.0)
        self.assertLess(payload["follow_angle"], 0.0)

    def test_two_line_points_publish_fitted_line_angle(self) -> None:
        payload = make_line_payload(
            line_points=[(520.0, 420.0), (500.0, 300.0)],
            frame_w=640,
            frame_h=480,
        )

        self.assertLess(payload["line_angle"], 0.0)
        self.assertGreater(payload["follow_angle"], 0.0)

    def test_configured_offset_is_shared_by_line_and_ball(self) -> None:
        payload = make_vision_payload(
            dets=[],
            line_points=[(360.0, 400.0)],
            frame_w=640,
            frame_h=480,
            cfg={
                "robot_center_offset_x_px": 40.0,
                "ball_class": "ball",
                "ball_conf": 0.2,
                "hurdle_class": "hurdle",
                "hurdle_conf": 0.2,
            },
        )

        self.assertEqual(payload["robot_center_x"], 360.0)
        self.assertEqual(payload["line_distance"], 0.0)

    def test_line_decision_uses_calibrated_distance(self) -> None:
        payload = make_line_payload(
            line_points=[
                (410.0, 470.0),
                (410.0, 380.0),
                (410.0, 290.0),
            ],
            frame_w=640,
            frame_h=480,
        )
        payload = apply_line_status(payload, frame_w=640, frame_h=480)

        # 보정 전 중심(320) 기준이면 +90px이어서 오른쪽 보정 모션이지만,
        # 실제 로봇 중심(345) 기준으로는 +65px이므로 직진해야 한다.
        self.assertEqual(payload["line_distance"], 65.0)
        self.assertEqual(payload["status"], LineStatus.Forward_4step)

    def test_curve_distance_uses_first_and_tangent_uses_third_point(self) -> None:
        payload = make_line_payload(
            line_points=[
                (400.0, 450.0),
                (370.0, 350.0),
                (360.0, 250.0),
                (370.0, 150.0),
            ],
            frame_w=640,
            frame_h=480,
        )

        self.assertGreater(abs(payload["curve_a"]), 1.0e-4)
        # 곡선 거리도 로봇에 가장 가까운 첫 번째 점(400px)을 사용한다.
        self.assertEqual(payload["line_distance"], 55.0)

        a = payload["curve_a"]
        third_y = 250.0
        # 같은 피팅식의 세 번째 점 y에서 계산한 접선이어야 한다.
        coeffs = np.polyfit(
            [450.0, 350.0, 250.0, 150.0],
            [400.0, 370.0, 360.0, 370.0],
            2,
        )
        expected_tangent = math.degrees(
            math.atan2(-(2.0 * a * third_y + coeffs[1]), 1.0)
        )
        self.assertAlmostEqual(
            payload["tangent_angle"],
            expected_tangent,
            places=6,
        )

    def test_straight_with_four_points_keeps_nearest_point_distance(self) -> None:
        payload = make_line_payload(
            line_points=[
                (410.0, 470.0),
                (400.0, 380.0),
                (390.0, 290.0),
                (380.0, 200.0),
            ],
            frame_w=640,
            frame_h=480,
        )

        self.assertLessEqual(abs(payload["curve_a"]), 1.0e-4)
        self.assertEqual(payload["line_distance"], 65.0)

    def test_nearly_straight_four_points_keep_second_point_tangent(self) -> None:
        points = [
            (400.0, 450.0),
            (380.0, 350.0),
            (362.0, 250.0),
            (346.0, 150.0),
        ]
        payload = make_line_payload(
            line_points=points,
            frame_w=640,
            frame_h=480,
        )

        self.assertLessEqual(abs(payload["curve_a"]), 2.0e-4)
        coeffs = np.polyfit(
            [point[1] for point in points],
            [point[0] for point in points],
            2,
        )
        expected_tangent = math.degrees(
            math.atan2(-(2.0 * coeffs[0] * 350.0 + coeffs[1]), 1.0)
        )
        self.assertAlmostEqual(
            payload["tangent_angle"],
            expected_tangent,
            places=6,
        )

    def test_turn_half_status_name_is_exposed(self) -> None:
        payload = apply_line_status(
            {
                "point_count": 3,
                "line_angle": 25.0,
                "line_distance": 0.0,
            },
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["status"], LineStatus.Right_Turn_Half)
        self.assertEqual(payload["status_name"], "Right_Turn_Half")

class _FakePublisher:
    def __init__(self) -> None:
        self.last_message = None

    def publish(self, message) -> None:
        self.last_message = message


class _FakeNode:
    def __init__(self) -> None:
        self.publisher = _FakePublisher()
        self.callbacks = {}

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def create_subscription(
        self,
        _msg_type,
        topic_name,
        callback,
        _depth,
    ):
        self.callbacks[topic_name] = callback
        return object()


class BallStatusPublisherTest(unittest.TestCase):
    @staticmethod
    def _confirm_initial_pose(publisher, node, **ball_kwargs) -> None:
        for _ in range(5):
            publisher.publish_ball_status(**ball_kwargs)
        node.callbacks["motion_command"](
            SimpleNamespace(command=BallStatus.Back_To_Initial)
        )

    def test_publishes_measured_angle_and_xy_distances(self) -> None:
        node = _FakeNode()
        publisher = BallStatusPublisher(node)

        ball_kwargs = {
            "webcam_ball_detected": True,
            "webcam_ball_x_distance": 55.0,
            "webcam_ball_y_distance": 179.0,
            "webcam_ball_angle_error": 17.08,
            "webcam_ball_distance_px": 187.259,
        }
        self._confirm_initial_pose(publisher, node, **ball_kwargs)
        status, angle = publisher.publish_ball_status(
            **ball_kwargs,
        )

        self.assertEqual(status, BallStatus.Right_Turn_5)
        self.assertAlmostEqual(angle, 17.08, places=2)
        self.assertAlmostEqual(node.publisher.last_message.detected_angle, 17.08)
        self.assertAlmostEqual(node.publisher.last_message.x_distance_px, 55.0)
        self.assertAlmostEqual(node.publisher.last_message.y_distance_px, 179.0)

    def test_x_distance_keeps_left_direction(self) -> None:
        payload = add_ball_geometry(
            {
                "ball_detected": True,
                "ball_x": 300.0,
                "ball_y": 300.0,
            },
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["ball_x_distance_px"], -45.0)
        self.assertEqual(payload["ball_y_distance_px"], 179.0)

        node = _FakeNode()
        publisher = BallStatusPublisher(node)
        ball_kwargs = {
            "webcam_ball_detected": True,
            "webcam_ball_x_distance": payload["ball_x_distance_px"],
            "webcam_ball_y_distance": payload["ball_y_distance_px"],
            "webcam_ball_angle_error": payload["ball_angle_deg"],
            "webcam_ball_distance_px": payload["ball_distance_px"],
        }
        self._confirm_initial_pose(publisher, node, **ball_kwargs)
        status, _angle = publisher.publish_ball_status(
            **ball_kwargs,
        )

        self.assertEqual(status, BallStatus.Left_Turn_5)
        self.assertEqual(node.publisher.last_message.x_distance_px, -45.0)
        self.assertEqual(node.publisher.last_message.y_distance_px, 179.0)


if __name__ == "__main__":
    unittest.main()
