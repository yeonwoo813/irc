#!/usr/bin/env python3

"""공/허들/후프 디버그 영상 중 현재 활성화된 화면 하나를 선택한다."""

import json
import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class RealSenseDebugSelector(Node):
    def __init__(self) -> None:
        super().__init__("realsense_debug_selector")

        self.declare_parameter(
            "ball_debug_topic",
            "/ball/realsense_debug_image",
        )
        self.declare_parameter(
            "hurdle_debug_topic",
            "/hurdle/realsense_debug_image",
        )
        self.declare_parameter("hoop_debug_topic", "/hoop/debug_image")
        self.declare_parameter("ball_state_topic", "/ball/vision_state")
        self.declare_parameter("hurdle_state_topic", "/hurdle/vision_state")
        self.declare_parameter("hoop_state_topic", "/hoop/vision_state")
        self.declare_parameter("ball_active_topic", "/vision/ball_active")
        self.declare_parameter("hoop_active_topic", "/vision/hoop_active")
        self.declare_parameter(
            "output_topic",
            "/vision/realsense_debug_image",
        )
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("show_window", True)
        self.declare_parameter(
            "window_name",
            "RealSense Ball / Hurdle / Hoop Vision",
        )

        self.state_timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.bridge = CvBridge()
        self.ball_detected = False
        self.hurdle_detected = False
        self.hoop_detected = False
        # None means that the decision node has not announced a mode yet.
        # In that brief startup state, any incoming debug stream may be shown.
        self.ball_enabled: Optional[bool] = None
        self.hoop_enabled: Optional[bool] = None
        self.ball_state_time = 0.0
        self.hurdle_state_time = 0.0
        self.hoop_state_time = 0.0
        self.latest_ball_image: Optional[Image] = None
        self.latest_hurdle_image: Optional[Image] = None
        self.latest_hoop_image: Optional[Image] = None

        self.pub_image = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("ball_debug_topic").value),
            self.cb_ball_image,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("hurdle_debug_topic").value),
            self.cb_hurdle_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("hoop_debug_topic").value),
            self.cb_hoop_image,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("ball_state_topic").value),
            self.cb_ball_state,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("hurdle_state_topic").value),
            self.cb_hurdle_state,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("hoop_state_topic").value),
            self.cb_hoop_state,
            10,
        )
        # main_decision publishes these as transient-local values. Matching
        # that durability also restores the current mode after this selector
        # is restarted independently.
        vision_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("ball_active_topic").value),
            self.cb_ball_active,
            vision_mode_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("hoop_active_topic").value),
            self.cb_hoop_active,
            vision_mode_qos,
        )

        self.get_logger().info(
            "RealSense debug selector started: "
            "/vision/realsense_debug_image, "
            f"show_window={self.show_window}"
        )

    def _publish_and_show(self, msg: Image) -> None:
        self.pub_image.publish(msg)
        if not self.show_window:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow(self.window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to show selected RealSense debug image: {exc}"
            )

    def cb_ball_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.ball_detected = bool(
            state.get("realsense_ball_detected", False)
        )
        self.ball_state_time = time.monotonic()

    def cb_hurdle_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.hurdle_detected = bool(
            state.get("realsense_valid", False)
            or state.get("fused_hurdle_detected", False)
        )
        self.hurdle_state_time = time.monotonic()

    def cb_hoop_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.hoop_detected = bool(state.get("detected", False))
        self.hoop_state_time = time.monotonic()

    def cb_ball_active(self, msg: Bool) -> None:
        self.ball_enabled = bool(msg.data)
        if not self.ball_enabled:
            # Do not let an image from the previous match phase block the
            # next detector's default stream.
            self.latest_ball_image = None

    def cb_hoop_active(self, msg: Bool) -> None:
        self.hoop_enabled = bool(msg.data)
        if not self.hoop_enabled:
            self.latest_hoop_image = None

    def _active_source(self) -> str:
        # Select the detector that is running, even while it currently has no
        # detection. Detection state is not an activity signal: using it here
        # made the window retain the last ball frame after switching to hoop.
        if self.hoop_enabled is True:
            return "hoop"
        if self.ball_enabled is True:
            return "ball"

        now = time.monotonic()
        hurdle_active = bool(
            self.hurdle_detected
            and now - self.hurdle_state_time <= self.state_timeout_sec
        )
        if hurdle_active:
            return "hurdle"
        return "default"

    def cb_ball_image(self, msg: Image) -> None:
        self.latest_ball_image = msg
        source = self._active_source()
        if source in {"ball", "default"}:
            self._publish_and_show(msg)

    def cb_hurdle_image(self, msg: Image) -> None:
        self.latest_hurdle_image = msg
        source = self._active_source()
        if source in {"hurdle", "default"}:
            self._publish_and_show(msg)

    def cb_hoop_image(self, msg: Image) -> None:
        self.latest_hoop_image = msg
        source = self._active_source()
        if source in {"hoop", "default"}:
            self._publish_and_show(msg)

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseDebugSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
