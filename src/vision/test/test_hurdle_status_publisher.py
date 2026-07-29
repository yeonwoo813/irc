#!/usr/bin/env python3
"""Tests for webcam hurdle distance/angle decisions."""

import math
from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from hurdle_status_publisher import (  # noqa: E402
    HurdleDecision,
    HurdleFeatures,
    HurdleStatus,
    HurdleStatusPublisher,
)
from yolo_detector import make_line_payload  # noqa: E402


class HurdleDecisionTest(unittest.TestCase):
    @staticmethod
    def _features(**overrides) -> HurdleFeatures:
        values = {
            "hurdle_detected": True,
            "line_point_count": 2,
            "line_follow_angle_deg": 0.0,
            "line_second_point_distance_px": 0.0,
            "line_angle_deg": 0.0,
        }
        values.update(overrides)
        return HurdleFeatures(**values)

    def setUp(self) -> None:
        self.decision = HurdleDecision()
        self.decision.back_to_initial_done = True

    def test_first_detection_keeps_back_to_initial_status(self) -> None:
        decision = HurdleDecision()

        first = decision.decide(self._features())
        repeated = decision.decide(self._features())

        self.assertEqual(
            first,
            (HurdleStatus.Back_To_Initial, 0.0, False),
        )
        self.assertEqual(repeated, first)

    def test_no_hurdle_publishes_none(self) -> None:
        result = self.decision.decide(
            self._features(hurdle_detected=False)
        )
        self.assertEqual(result, (HurdleStatus.Hurdle_None, 0.0, False))

    def test_hurdle_without_line_moves_backward(self) -> None:
        result = self.decision.decide(
            self._features(line_point_count=0)
        )
        self.assertEqual(result, (HurdleStatus.Backward_half, 0.0, False))

    def test_partial_line_turns_toward_line(self) -> None:
        left = self.decision.decide(
            self._features(
                line_point_count=1,
                line_follow_angle_deg=-12.0,
            )
        )
        right = self.decision.decide(
            self._features(
                line_point_count=1,
                line_follow_angle_deg=8.0,
            )
        )

        self.assertEqual(
            left,
            (HurdleStatus.Left_Turn_Hurdle, -12.0, False),
        )
        self.assertEqual(
            right,
            (HurdleStatus.Right_Turn_Hurdle, 8.0, False),
        )

    def test_two_line_points_do_not_require_intersection(self) -> None:
        result = self.decision.decide(
            self._features(
                line_second_point_distance_px=0.0,
                line_angle_deg=0.0,
            )
        )
        self.assertEqual(
            result,
            (HurdleStatus.Hurdle_Forward_20, 0.0, True),
        )

    def test_center_zone_accepts_angle_endpoints(self) -> None:
        for distance in (-100.0, 0.0, 100.0):
            for angle in (-10.0, 0.0, 10.0):
                with self.subTest(distance=distance, angle=angle):
                    status, published_angle, ready = self.decision.decide(
                        self._features(
                            line_second_point_distance_px=distance,
                            line_angle_deg=angle,
                        )
                    )
                    self.assertEqual(
                        status,
                        HurdleStatus.Hurdle_Forward_20,
                    )
                    self.assertEqual(published_angle, angle)
                    self.assertTrue(ready)

    def test_center_zone_turns_into_angle_range(self) -> None:
        left = self.decision.decide(
            self._features(line_angle_deg=-10.01)
        )
        right = self.decision.decide(
            self._features(line_angle_deg=10.01)
        )

        self.assertEqual(left[0], HurdleStatus.Left_Turn_Hurdle)
        self.assertFalse(left[2])
        self.assertEqual(right[0], HurdleStatus.Right_Turn_Hurdle)
        self.assertFalse(right[2])

    def test_right_zone_accepts_5_to_20_degrees(self) -> None:
        for angle in (5.0, 12.0, 20.0):
            with self.subTest(angle=angle):
                status, _, ready = self.decision.decide(
                    self._features(
                        line_second_point_distance_px=100.01,
                        line_angle_deg=angle,
                    )
                )
                self.assertEqual(status, HurdleStatus.Hurdle_Forward_20)
                self.assertTrue(ready)

        below = self.decision.decide(
            self._features(
                line_second_point_distance_px=100.01,
                line_angle_deg=4.99,
            )
        )
        above = self.decision.decide(
            self._features(
                line_second_point_distance_px=100.01,
                line_angle_deg=20.01,
            )
        )
        self.assertEqual(below[0], HurdleStatus.Left_Turn_Hurdle)
        self.assertEqual(above[0], HurdleStatus.Right_Turn_Hurdle)

    def test_left_zone_accepts_minus_20_to_minus_5_degrees(self) -> None:
        for angle in (-20.0, -12.0, -5.0):
            with self.subTest(angle=angle):
                status, _, ready = self.decision.decide(
                    self._features(
                        line_second_point_distance_px=-100.01,
                        line_angle_deg=angle,
                    )
                )
                self.assertEqual(status, HurdleStatus.Hurdle_Forward_20)
                self.assertTrue(ready)

        below = self.decision.decide(
            self._features(
                line_second_point_distance_px=-100.01,
                line_angle_deg=-20.01,
            )
        )
        above = self.decision.decide(
            self._features(
                line_second_point_distance_px=-100.01,
                line_angle_deg=-4.99,
            )
        )
        self.assertEqual(below[0], HurdleStatus.Left_Turn_Hurdle)
        self.assertEqual(above[0], HurdleStatus.Right_Turn_Hurdle)

    def test_turn_is_rechecked_with_next_angle(self) -> None:
        turn = self.decision.decide(
            self._features(
                line_second_point_distance_px=0.0,
                line_angle_deg=15.0,
            )
        )
        forward = self.decision.decide(
            self._features(
                line_second_point_distance_px=0.0,
                line_angle_deg=5.0,
            )
        )

        self.assertEqual(
            turn,
            (HurdleStatus.Right_Turn_Hurdle, 15.0, False),
        )
        self.assertEqual(
            forward,
            (HurdleStatus.Hurdle_Forward_20, 5.0, True),
        )


class HurdlePublisherTest(unittest.TestCase):
    def test_ready_result_is_published_with_forward_20(self) -> None:
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        class FakeNode:
            def __init__(self):
                self.recorder = Recorder()
                self.motion_command_callback = None
                self.motion_end_callback = None

            def create_publisher(self, _msg_type, _topic_name, _depth):
                return self.recorder

            def create_subscription(
                self,
                _msg_type,
                _topic_name,
                callback,
                _depth,
            ):
                if _topic_name == "motion_command":
                    self.motion_command_callback = callback
                elif _topic_name == "motion_end":
                    self.motion_end_callback = callback
                return object()

        node = FakeNode()
        publisher = HurdleStatusPublisher(node)

        back_to_initial = publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
            line_follow_angle_deg=0.0,
            line_second_point_distance_px=0.0,
            line_angle_deg=0.0,
        )

        class FakeMotionCommand:
            def __init__(self, command):
                self.command = command

        node.motion_command_callback(FakeMotionCommand(1))
        repeated = publisher.publish_hurdle_status(
            hurdle_detected=False,
            line_point_count=2,
            line_follow_angle_deg=0.0,
            line_second_point_distance_px=0.0,
            line_angle_deg=0.0,
        )
        node.motion_command_callback(
            FakeMotionCommand(HurdleStatus.Back_To_Initial)
        )

        result = publisher.publish_hurdle_status(
            hurdle_detected=False,
            line_point_count=2,
            line_follow_angle_deg=0.0,
            line_second_point_distance_px=0.0,
            line_angle_deg=0.0,
        )

        self.assertEqual(
            back_to_initial,
            (HurdleStatus.Back_To_Initial, 0.0, False),
        )
        self.assertEqual(repeated, back_to_initial)
        self.assertEqual(
            result,
            (HurdleStatus.Hurdle_Forward_20, 0.0, True),
        )
        self.assertEqual(len(node.recorder.messages), 3)
        self.assertEqual(
            node.recorder.messages[0].status,
            HurdleStatus.Back_To_Initial,
        )
        self.assertEqual(
            node.recorder.messages[1].status,
            HurdleStatus.Back_To_Initial,
        )
        self.assertEqual(
            node.recorder.messages[2].status,
            HurdleStatus.Hurdle_Forward_20,
        )
        self.assertEqual(node.recorder.messages[2].angle, 0.0)
        self.assertTrue(node.recorder.messages[2].hurdle_ready)

    def test_hurdle_go_suppresses_detection_until_two_seconds_after_end(
        self,
    ) -> None:
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, msg):
                self.messages.append(msg)

        class FakeNode:
            def __init__(self):
                self.recorder = Recorder()
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

        class FakeClock:
            def __init__(self):
                self.now = 100.0

            def __call__(self):
                return self.now

        class FakeMotionCommand:
            def __init__(self, command):
                self.command = command

        class FakeMotionEnd:
            def __init__(self, motion_end):
                self.motion_end = motion_end
                self.motion_ready = True

        clock = FakeClock()
        node = FakeNode()
        publisher = HurdleStatusPublisher(
            node,
            post_crossing_cooldown_sec=2.0,
            monotonic_clock=clock,
        )

        publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
        )
        self.assertTrue(publisher.hurdle_detected)
        node.callbacks["motion_command"](
            FakeMotionCommand(HurdleStatus.Back_To_Initial)
        )
        node.callbacks["motion_command"](
            FakeMotionCommand(HurdleStatus.Hurdle_Go)
        )
        during_crossing = publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
        )

        # 서로 다른 토픽의 전달 순서가 뒤집혀 직전 모션의 완료 상태가
        # 늦게 도착해도 Hurdle_Go 완료로 오인하지 않아야 합니다.
        node.callbacks["motion_end"](FakeMotionEnd(True))
        after_stale_completion = publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
        )

        node.callbacks["motion_end"](FakeMotionEnd(False))
        self.assertTrue(publisher.hurdle_detected)
        still_crossing = publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
        )

        node.callbacks["motion_end"](FakeMotionEnd(True))
        self.assertFalse(publisher.hurdle_detected)
        clock.now = 101.999
        during_cooldown = publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
        )

        clock.now = 102.0
        after_cooldown = publisher.publish_hurdle_status(
            hurdle_detected=True,
            line_point_count=2,
        )

        suppressed = (HurdleStatus.Hurdle_None, 0.0, False)
        self.assertEqual(during_crossing, suppressed)
        self.assertEqual(after_stale_completion, suppressed)
        self.assertEqual(still_crossing, suppressed)
        self.assertEqual(during_cooldown, suppressed)
        self.assertEqual(
            after_cooldown,
            (HurdleStatus.Back_To_Initial, 0.0, False),
        )
        self.assertEqual(
            [msg.status for msg in node.recorder.messages],
            [
                HurdleStatus.Back_To_Initial,
                HurdleStatus.Hurdle_None,
                HurdleStatus.Hurdle_None,
                HurdleStatus.Hurdle_None,
                HurdleStatus.Hurdle_None,
                HurdleStatus.Back_To_Initial,
            ],
        )

    def test_motion_start_arriving_before_hurdle_go_is_not_missed(
        self,
    ) -> None:
        class Recorder:
            def publish(self, _msg):
                pass

        class FakeNode:
            def __init__(self):
                self.callbacks = {}

            def create_publisher(self, _msg_type, _topic_name, _depth):
                return Recorder()

            def create_subscription(
                self,
                _msg_type,
                topic_name,
                callback,
                _depth,
            ):
                self.callbacks[topic_name] = callback
                return object()

        class FakeClock:
            now = 10.0

            def __call__(self):
                return self.now

        class FakeMotionCommand:
            command = HurdleStatus.Hurdle_Go

        class FakeMotionEnd:
            motion_ready = True

            def __init__(self, motion_end):
                self.motion_end = motion_end

        clock = FakeClock()
        node = FakeNode()
        publisher = HurdleStatusPublisher(
            node,
            post_crossing_cooldown_sec=2.0,
            monotonic_clock=clock,
        )

        # motion 노드가 19번 명령을 먼저 처리한 전달 순서를 재현합니다.
        node.callbacks["motion_end"](FakeMotionEnd(False))
        node.callbacks["motion_command"](FakeMotionCommand())
        self.assertEqual(publisher.suppression_reason(), "crossing")

        node.callbacks["motion_end"](FakeMotionEnd(True))
        self.assertEqual(publisher.suppression_reason(), "cooldown")

        clock.now = 12.0
        self.assertIsNone(publisher.suppression_reason())


class HurdleLineGeometryTest(unittest.TestCase):
    def test_second_point_has_dedicated_distance_and_angle(self) -> None:
        payload = make_line_payload(
            line_points=[
                (100.0, 470.0),
                (420.0, 380.0),
            ],
            frame_w=640,
            frame_h=480,
        )

        self.assertEqual(payload["line_distance"], -220.0)
        self.assertEqual(payload["line_second_point_distance_px"], 100.0)
        self.assertAlmostEqual(
            payload["hurdle_line_angle_deg"],
            math.degrees(math.atan2(320.0, 90.0)),
        )


if __name__ == "__main__":
    unittest.main()
