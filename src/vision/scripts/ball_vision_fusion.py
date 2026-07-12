#!/usr/bin/env python3

import json
import math
import time
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped


class BallVisionFusionNode(Node):
    """
    Vision 담당 노드.

    입력:
    - /basketball/position
      RealSense에서 검출한 공의 3D 위치 PointStamped
      point.x: 좌우 위치[m], 음수=왼쪽, 양수=오른쪽
      point.z: 공까지 depth[m]

    - /line_tracker/state
      yolo_detector.py가 publish하는 JSON String
      ball_detected, ball_x, ball_y, ball_bbox 사용

    출력:
    - /ball/vision_state
      알고리즘 BallDecision이 필요한 BallFeatures 값 JSON
    """

    def __init__(self):
        super().__init__("ball_vision_fusion")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("realsense_topic", "/basketball/position")
        self.declare_parameter("webcam_state_topic", "/line_tracker/state")
        self.declare_parameter("ball_in_hand_topic", "/ball/in_hand")
        self.declare_parameter("publish_topic", "/ball/vision_state")

        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)

        # 웹캠 화면에서 로봇 기준점.
        # 실제 화면에서 공을 집기 좋은 기준점을 보고 조정해야 함.
        self.declare_parameter("webcam_robot_center_x", 320.0)
        self.declare_parameter("webcam_robot_center_y", 420.0)

        # 웹캠 가로 화각. 정확하지 않으면 60도 정도로 시작.
        self.declare_parameter("webcam_fov_x_deg", 60.0)

        self.declare_parameter("realsense_timeout_sec", 0.5)
        self.declare_parameter("webcam_timeout_sec", 0.5)
        self.declare_parameter("publish_hz", 15.0)

        # True면 RealSense 거리 = sqrt(x^2 + z^2)
        # False면 RealSense 거리 = z depth
        self.declare_parameter("realsense_use_euclidean_distance", False)

        self.declare_parameter("print_every_n_frames", 10)

        self.realsense_topic = self.get_parameter("realsense_topic").value
        self.webcam_state_topic = self.get_parameter("webcam_state_topic").value
        self.ball_in_hand_topic = self.get_parameter("ball_in_hand_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value

        self.frame_width = float(self.get_parameter("frame_width").value)
        self.frame_height = float(self.get_parameter("frame_height").value)

        self.webcam_robot_center_x = float(self.get_parameter("webcam_robot_center_x").value)
        self.webcam_robot_center_y = float(self.get_parameter("webcam_robot_center_y").value)
        self.webcam_fov_x_deg = float(self.get_parameter("webcam_fov_x_deg").value)

        self.realsense_timeout_sec = float(self.get_parameter("realsense_timeout_sec").value)
        self.webcam_timeout_sec = float(self.get_parameter("webcam_timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.realsense_use_euclidean_distance = bool(
            self.get_parameter("realsense_use_euclidean_distance").value
        )

        self.print_every_n_frames = int(self.get_parameter("print_every_n_frames").value)

        # -----------------------------
        # State
        # -----------------------------
        self.latest_realsense: Optional[Dict[str, Any]] = None
        self.latest_realsense_time = 0.0

        self.latest_webcam: Optional[Dict[str, Any]] = None
        self.latest_webcam_time = 0.0

        self.ball_in_hand = False
        self.frame_count = 0

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.sub_rs = self.create_subscription(
            PointStamped,
            self.realsense_topic,
            self.cb_realsense,
            10,
        )

        self.sub_webcam = self.create_subscription(
            String,
            self.webcam_state_topic,
            self.cb_webcam_state,
            10,
        )

        self.sub_hand = self.create_subscription(
            Bool,
            self.ball_in_hand_topic,
            self.cb_ball_in_hand,
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            self.publish_topic,
            10,
        )

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_fused_state)

        self.get_logger().info("BallVisionFusionNode started.")
        self.get_logger().info(f"Subscribe RealSense: {self.realsense_topic}")
        self.get_logger().info(f"Subscribe Webcam YOLO state: {self.webcam_state_topic}")
        self.get_logger().info(f"Publish: {self.publish_topic}")

    # -----------------------------
    # Callback: RealSense
    # -----------------------------
    def cb_realsense(self, msg: PointStamped):
        x_m = float(msg.point.x)
        y_m = float(msg.point.y)
        z_m = float(msg.point.z)

        if z_m <= 0.0 or math.isnan(z_m):
            return

        if self.realsense_use_euclidean_distance:
            distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)
        else:
            distance_m = z_m

        distance_cm = distance_m * 100.0

        # 음수면 공이 왼쪽, 양수면 공이 오른쪽
        angle_error_deg = math.degrees(math.atan2(x_m, z_m))

        self.latest_realsense = {
            "realsense_ball_detected": True,
            "realsense_ball_distance_cm": distance_cm,
            "realsense_ball_angle_error": angle_error_deg,
            "raw_x_m": x_m,
            "raw_y_m": y_m,
            "raw_z_m": z_m,
        }
        self.latest_realsense_time = time.time()

    # -----------------------------
    # Callback: Webcam YOLO state
    # -----------------------------
    def cb_webcam_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to parse /line_tracker/state JSON")
            return

        ball_detected = bool(payload.get("ball_detected", False))

        if not ball_detected:
            self.latest_webcam = {
                "webcam_ball_detected": False,
                "webcam_ball_x_distance": None,
                "webcam_ball_angle_error": None,
                "webcam_ball_distance_px": None,
                "raw_ball_x": -1.0,
                "raw_ball_y": -1.0,
                "raw_ball_conf": 0.0,
                "raw_ball_bbox": [],
            }
            self.latest_webcam_time = time.time()
            return

        ball_x = float(payload.get("ball_x", -1.0))
        ball_y = float(payload.get("ball_y", -1.0))
        ball_conf = float(payload.get("ball_conf", 0.0))
        ball_bbox = payload.get("ball_bbox", [])

        if ball_x < 0 or ball_y < 0:
            return

        # x축 거리: 음수면 왼쪽, 양수면 오른쪽
        dx = ball_x - self.webcam_robot_center_x
        dy = ball_y - self.webcam_robot_center_y

        # 로봇 기준점과 공 중심점 사이 픽셀 거리
        distance_px = math.sqrt(dx * dx + dy * dy)

        # 웹캠 기준 각도. 필요 없으면 알고리즘에서 0처럼 써도 됨.
        focal_px = self.frame_width / (
            2.0 * math.tan(math.radians(self.webcam_fov_x_deg) / 2.0)
        )
        angle_error_deg = math.degrees(math.atan2(dx, focal_px))

        self.latest_webcam = {
            "webcam_ball_detected": True,
            "webcam_ball_x_distance": dx,
            "webcam_ball_angle_error": angle_error_deg,
            "webcam_ball_distance_px": distance_px,
            "raw_ball_x": ball_x,
            "raw_ball_y": ball_y,
            "raw_ball_conf": ball_conf,
            "raw_ball_bbox": ball_bbox,
        }
        self.latest_webcam_time = time.time()

    # -----------------------------
    # Callback: ball in hand
    # -----------------------------
    def cb_ball_in_hand(self, msg: Bool):
        self.ball_in_hand = bool(msg.data)

    # -----------------------------
    # Publish fused state
    # -----------------------------
    def publish_fused_state(self):
        now = time.time()

        rs_valid = (
            self.latest_realsense is not None
            and now - self.latest_realsense_time <= self.realsense_timeout_sec
        )

        webcam_valid = (
            self.latest_webcam is not None
            and now - self.latest_webcam_time <= self.webcam_timeout_sec
            and bool(self.latest_webcam.get("webcam_ball_detected", False))
        )

        # 기본값: 알고리즘 BallFeatures와 이름 맞춤
        output = {
            "realsense_ball_detected": False,
            "realsense_ball_distance_cm": None,
            "realsense_ball_angle_error": None,

            "webcam_ball_detected": False,
            "webcam_ball_x_distance": None,
            "webcam_ball_angle_error": None,
            "webcam_ball_distance_px": None,

            "ball_in_hand": bool(self.ball_in_hand),

            # 디버깅용
            "source_priority": "none",
            "realsense_age_sec": None,
            "webcam_age_sec": None,
        }

        if rs_valid:
            output.update({
                "realsense_ball_detected": True,
                "realsense_ball_distance_cm": self.latest_realsense["realsense_ball_distance_cm"],
                "realsense_ball_angle_error": self.latest_realsense["realsense_ball_angle_error"],
                "realsense_raw": {
                    "x_m": self.latest_realsense["raw_x_m"],
                    "y_m": self.latest_realsense["raw_y_m"],
                    "z_m": self.latest_realsense["raw_z_m"],
                },
                "realsense_age_sec": now - self.latest_realsense_time,
            })

        if webcam_valid:
            output.update({
                "webcam_ball_detected": True,
                "webcam_ball_x_distance": self.latest_webcam["webcam_ball_x_distance"],
                "webcam_ball_angle_error": self.latest_webcam["webcam_ball_angle_error"],
                "webcam_ball_distance_px": self.latest_webcam["webcam_ball_distance_px"],
                "webcam_raw": {
                    "ball_x": self.latest_webcam["raw_ball_x"],
                    "ball_y": self.latest_webcam["raw_ball_y"],
                    "ball_conf": self.latest_webcam["raw_ball_conf"],
                    "ball_bbox": self.latest_webcam["raw_ball_bbox"],
                },
                "webcam_age_sec": now - self.latest_webcam_time,
            })

        # 알고리즘 문서 기준: webcam에서 공이 보이면 webcam 우선
        if webcam_valid:
            output["source_priority"] = "webcam"
        elif rs_valid:
            output["source_priority"] = "realsense"

        self.pub_state.publish(String(data=json.dumps(output, ensure_ascii=False)))

        self.frame_count += 1
        if self.frame_count % self.print_every_n_frames == 0:
            self.get_logger().info(
                "ball_vision "
                f"src={output['source_priority']} "
                f"rs={output['realsense_ball_detected']} "
                f"rs_dist={output['realsense_ball_distance_cm']} "
                f"rs_ang={output['realsense_ball_angle_error']} "
                f"webcam={output['webcam_ball_detected']} "
                f"webcam_x={output['webcam_ball_x_distance']} "
                f"webcam_dist={output['webcam_ball_distance_px']} "
                f"hand={output['ball_in_hand']}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BallVisionFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
