#!/usr/bin/env python3
"""Tests for the actual-motion overlay shown on the webcam YOLO image."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from yolo_detector import MotionDisplayState, motion_overlay_lines  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
