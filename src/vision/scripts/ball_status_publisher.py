from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

from rclpy.node import Node
from msgs.msg import BallResult, MotionCommand


class BallStatus:
    Forward_4step = 1
    Forward_3step = 20
    Left_Half_Forward = 2
    Right_Half_Forward = 3
    Left_Forward = 4
    Right_Forward = 5
    Left_Turn = 6
    Right_Turn = 7
    Forward_half = 8
    Backward_half = 9
    Left_Move = 10
    Right_Move = 11
    Pick_Ready = 12
    Shoot = 13
    Left_Half_Forward_3step = 21
    Right_Half_Forward_3step = 22
    Left_Turn_10 = 23
    Right_Turn_10 = 24
    Left_Turn_5 = 28
    Right_Turn_5 = 29
    Left_Turn_Afterpick = 30
    Right_Turn_Afterpick = 31
    Neck_Up = 14
    Back_To_Initial = 27
    Ball_In_Hand = 50
    Ball_Lost = 45
    Ball_None = 99


@dataclass
class BallFeatures:
    realsense_ball_detected: bool = False
    realsense_ball_distance_cm: Optional[float] = None
    realsense_ball_angle_error: Optional[float] = None

    webcam_ball_detected: bool = False
    # 로봇 중심선 기준 signed 거리: 왼쪽 음수, 오른쪽 양수.
    webcam_ball_x_distance: Optional[float] = None
    webcam_ball_y_distance: Optional[float] = None
    webcam_ball_angle_error: Optional[float] = None
    webcam_ball_distance_px: Optional[float] = None

    ball_in_hand: bool = False


class BallDecision:
    def __init__(self):
        #100cm 이하이면 공 모드
        self.ball_entry_distance_cm = 120.0

        # Realsense 직진, 회전 기준각 5도
        self.angle_center_tol = 5.0

        # Webcam 접근 및 pick 기준
        self.webcam_angle_center_tol = 5.0
        self.webcam_pick_y_max_px = 78.0
        self.webcam_pick_x_min_px = -48.0
        self.webcam_pick_x_max_px = 35.0

    def decide(self, features: BallFeatures) -> Tuple[int, float]:
        #공을 잡고 있으면 접근 명령을 보내지 않음
        if features.ball_in_hand:
            return BallStatus.Ball_None, 0.0

        if not self.Ball_mission_ready(features):
            return BallStatus.Ball_None, 0.0

        # webcam에서 공이 감지되면 webcam 기준으로 판단한다.
        if features.webcam_ball_detected:
            return self._decide_from_webcam(features)

        #realsense에서 공이 감지되면 realsense 기준으로 판단
        distance = features.realsense_ball_distance_cm
        if (
            features.realsense_ball_detected
            and distance is not None
            and distance <= self.ball_entry_distance_cm
        ):
            return self._status_from_angle(features.realsense_ball_angle_error)

        return BallStatus.Ball_None, 0.0

    #webcam에서 먼저 판단 후 없으면 realsense에서 판단
    def Ball_mission_ready(self, features: BallFeatures) -> bool:
        if features.webcam_ball_detected:
            return True

        if not features.realsense_ball_detected:
            return False

        distance = features.realsense_ball_distance_cm
        if distance is None:
            return False

        return distance <= self.ball_entry_distance_cm

    #Webcam 판단
    def _decide_from_webcam(self, features: BallFeatures) -> Tuple[int, float]:
        webcam_ball_x_distance = features.webcam_ball_x_distance
        webcam_ball_y_distance = features.webcam_ball_y_distance
        if (
            webcam_ball_x_distance is None
            or webcam_ball_y_distance is None
        ):
            return BallStatus.Ball_None, 0.0

        angle = self.webcam_angle(features.webcam_ball_angle_error)

        # y 거리를 먼저 판단한다. 입력 y 거리는 항상 양수로 들어온다.
        if webcam_ball_y_distance > self.webcam_pick_y_max_px:
            if angle < -10.0:
                return BallStatus.Left_Turn_10, angle
            if angle < -self.angle_center_tol:
                return BallStatus.Left_Turn_5, angle
            if angle <= self.angle_center_tol:
                return BallStatus.Forward_half, 0.0
            if angle <= 10.0:
                return BallStatus.Right_Turn_5, angle
            return BallStatus.Right_Turn_10, angle

        # pick 거리 안에서는 x 거리로 좌우 정렬 여부를 판단한다.
        if webcam_ball_x_distance < self.webcam_pick_x_min_px:
            return BallStatus.Left_Move, angle
        if webcam_ball_x_distance > self.webcam_pick_x_max_px:
            return BallStatus.Right_Move, angle
        return BallStatus.Pick_Ready, 0.0

    #realsense 기준으로 판단하는 각도
    def _status_from_angle(self, angle: Optional[float]) -> Tuple[int, float]:
        if angle is None:
            return BallStatus.Ball_None, 0.0

        if angle < -60.0:
            return BallStatus.Left_Turn, angle
        if angle < -self.angle_center_tol:
            return BallStatus.Left_Half_Forward, angle
        if angle <= self.angle_center_tol:
            return BallStatus.Forward_4step, 0.0
        if angle <= 60.0:
            return BallStatus.Right_Half_Forward, angle
        return BallStatus.Right_Turn, angle

    #webcam 각도 값이 없을 때 안전하게 처리, 값 있으면 그대로 반환
    def webcam_angle(self, angle: Optional[float]) -> float:
        if angle is None:
            return 0.0

        return angle

class BallStatusPublisher:
    def __init__(self, node: Node, topic_name: str = 'ball_result'):
        self.node = node
        self.ball_decision = BallDecision()
        # 웹캠 원본 검출 5개 중 3개 이상일 때 최초 공 검출을
        # 확정한다.
        self.webcam_detection_buffer = deque(maxlen=5)
        self.webcam_ball_confirmed = False
        # 27번이 실제 motion_command로 발행될 때까지 결과를 유지하고,
        # 실행 확인 후에는 Pick 결과 확인이 끝날 때까지 다시
        # 발행하지 않는다.
        self.back_to_initial_waiting = False
        self.back_to_initial_done = False
        self.pick_command_seen = False
        self.ball_pub = self.node.create_publisher(BallResult, topic_name, 10)
        self.motion_command_sub = self.node.create_subscription(
            MotionCommand,
            'motion_command',
            self._motion_command_callback,
            10,
        )

    def _log_info(self, message: str) -> None:
        get_logger = getattr(self.node, 'get_logger', None)
        if callable(get_logger):
            get_logger().info(message)

    def _reset_webcam_detection_cycle(self) -> None:
        self.webcam_detection_buffer.clear()
        self.webcam_ball_confirmed = False
        self.back_to_initial_waiting = False
        self.back_to_initial_done = False

    def _motion_command_callback(self, msg: MotionCommand) -> None:
        command = int(msg.command)

        if (
            self.back_to_initial_waiting
            and command == BallStatus.Back_To_Initial
        ):
            self.back_to_initial_waiting = False
            self.back_to_initial_done = True
            self._log_info(
                "Ball Back_To_Initial execution confirmed; "
                "initial-pose trigger locked until Pick result check."
            )

        if command == BallStatus.Pick_Ready:
            self.pick_command_seen = True
            return

        # MainDecision은 Pick 모션이 끝난 뒤 CheckBall()을 실행한
        # 다음,
        # 성공이면 Neck_Up, 실패이면 Backward_half를 발행한다.
        # 두 경우 모두 이후 Backward_half를 거쳐 Afterpick 회전으로 진행한다.
        if self.pick_command_seen and command in (
            BallStatus.Neck_Up,
            BallStatus.Backward_half,
            BallStatus.Left_Turn_Afterpick,
            BallStatus.Right_Turn_Afterpick,
        ):
            self.pick_command_seen = False
            self._reset_webcam_detection_cycle()
            self._log_info(
                "Pick result check completed; webcam ball detection "
                "lock released."
            )

    def publish_ball_status(
        self,
        realsense_ball_detected: bool = False,
        realsense_ball_distance_cm: Optional[float] = None,
        realsense_ball_angle_error: Optional[float] = None,
        webcam_ball_detected: bool = False,
        webcam_ball_x_distance: Optional[float] = None,
        webcam_ball_y_distance: Optional[float] = None,
        webcam_ball_angle_error: Optional[float] = None,
        webcam_ball_distance_px: Optional[float] = None,
        ball_in_hand: bool = False,
    ) -> Tuple[int, float]:
        if not self.back_to_initial_done and not self.webcam_ball_confirmed:
            # 손에 든 공은 다음 공의 최초 웹캠 검출로 집계하지
            # 않는다.
            detected_for_vote = bool(webcam_ball_detected and not ball_in_hand)
            self.webcam_detection_buffer.append(detected_for_vote)
            detected_count = sum(self.webcam_detection_buffer)
            if (
                len(self.webcam_detection_buffer) == 5
                and detected_count >= 3
            ):
                self.webcam_ball_confirmed = True
                self._log_info(
                    "Webcam ball confirmed by 5-frame majority: "
                    f"true={detected_count}, false={5 - detected_count}."
                )

        # 최초 웹캠 검출이 확정되기 전에는 webcam 기반 접근 명령을
        # 막는다. 확정 후에는 27번 모션의 실제 발행을 확인할 때까지
        # 27을 유지한다.
        webcam_enabled = bool(
            webcam_ball_detected and self.back_to_initial_done
        )
        features = BallFeatures(
            realsense_ball_detected=realsense_ball_detected,
            realsense_ball_distance_cm=realsense_ball_distance_cm,
            realsense_ball_angle_error=realsense_ball_angle_error,
            webcam_ball_detected=webcam_enabled,
            webcam_ball_x_distance=webcam_ball_x_distance,
            webcam_ball_y_distance=webcam_ball_y_distance,
            webcam_ball_angle_error=webcam_ball_angle_error,
            webcam_ball_distance_px=webcam_ball_distance_px,
            ball_in_hand=ball_in_hand,
        )

        status, angle = self.ball_decision.decide(features)

        if self.webcam_ball_confirmed and not self.back_to_initial_done:
            self.back_to_initial_waiting = True
            status = BallStatus.Back_To_Initial
            angle = 0.0

        msg = BallResult()
        msg.status = int(status)
        msg.angle = float(angle)
        if hasattr(msg, 'ball_in_hand'):
            msg.ball_in_hand = bool(ball_in_hand)
        measured_angle = (
            webcam_ball_angle_error
            if webcam_ball_angle_error is not None
            else realsense_ball_angle_error
        )
        if hasattr(msg, 'detected_angle'):
            msg.detected_angle = float(measured_angle or 0.0)
        if hasattr(msg, 'x_distance_px'):
            msg.x_distance_px = float(webcam_ball_x_distance or 0.0)
        if hasattr(msg, 'y_distance_px'):
            msg.y_distance_px = float(webcam_ball_y_distance or 0.0)

        self.ball_pub.publish(msg)

        return status, angle
