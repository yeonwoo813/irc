from dataclasses import dataclass
from typing import Tuple, Optional

from rclpy.node import Node
from msgs.msg import HurdleResult

class HurdleStatus:
    Left_Turn = 6
    Right_Turn = 7
    Backward_half = 9
    Hurdle_Forward_20 = 26
    Hurdle_1step = 25
    Hurdle_None = 99




@dataclass
class HurdleFeatures:
    #webcam에서 허들 발견 여부
    hurdle_detected: bool = False
    #라인 검출 정보
    line_point_count: int = 0
    line_follow_angle_deg: Optional[float] = None
    #교차점 검출 여부
    hurdle_line_intersection_valid: bool = False
    #교차점과 각도
    hurdle_intersection_angle_deg: Optional[float] = None
    #허들 영역 내에 들어왔는지 여부
    hurdle_inside: bool = False

class HurdleDecision:
    def __init__(self):
        self.center_angle = 10.0
        self.search_turn = HurdleStatus.Left_Turn

    def decide(self, features: HurdleFeatures) -> Tuple[int, float, bool]:

        # 1. 허들이 없음
        if not features.hurdle_detected:
            return HurdleStatus.Hurdle_None, 0.0, False

        #2. 허들은 보이는데, 교차점 없음
        if not features.hurdle_line_intersection_valid:

            #라인도 안보임
            if features.line_point_count <= 0:
                return HurdleStatus.Backward_half, 0.0, False

            #라인은 조금 보임 -> 라인있는방향으로 제자리회전
            line_angle = features.line_follow_angle_deg
            if line_angle is not None:
                if line_angle < 0.0:
                    self.search_turn = HurdleStatus.Left_Turn
                elif line_angle > 0.0:
                    self.search_turn = HurdleStatus.Right_Turn

            return self.search_turn, float(line_angle or 0.0), False


        # 3. 교차점 보임
        intersection_angle = features.hurdle_intersection_angle_deg

        if intersection_angle is None:
            return self.search_turn, 0.0, False

        # ±10도 영역 밖: 교차점 방향으로 제자리 회전
        if intersection_angle < -self.center_angle:
            self.search_turn = HurdleStatus.Left_Turn
            return HurdleStatus.Left_Turn, float(intersection_angle), False

        if intersection_angle > self.center_angle:
            self.search_turn = HurdleStatus.Right_Turn
            return HurdleStatus.Right_Turn, float(intersection_angle), False

        # 여기부터는 교차점이 정면 ±10도 영역 안

        #아직 접근영역에 도착하지 않음
        if not features.hurdle_inside:
            return HurdleStatus.Hurdle_1step, float(intersection_angle), False

        #접근영역에 도착, 20걸음 걷기(Ready)
        return HurdleStatus.Hurdle_None, float(intersection_angle), True


class HurdleStatusPublisher:
    def __init__(self, node: Node, topic_name: str = 'hurdle_result'):
        self.node = node
        self.hurdle_decision = HurdleDecision()
        self.hurdle_pub = self.node.create_publisher(HurdleResult, topic_name, 10)

    #publish 함수
    def publish_hurdle_status(
        self,
        hurdle_detected: bool,
        line_point_count: int,
        line_follow_angle_deg: Optional[float],
        intersection_valid: bool,
        intersection_angle_deg: Optional[float],
        hurdle_inside: bool,
    ) -> Tuple[int, float, bool]:

        features = HurdleFeatures(
            hurdle_detected=hurdle_detected,
            line_point_count=line_point_count,
            line_follow_angle_deg=line_follow_angle_deg,
            hurdle_line_intersection_valid=intersection_valid,
            hurdle_intersection_angle_deg=intersection_angle_deg,
            hurdle_inside=hurdle_inside,
        )

        status, angle, ready = self.hurdle_decision.decide(features)

        msg = HurdleResult()
        msg.status = int(status)
        msg.angle = float(angle)
        msg.hurdle_ready = bool(ready)

        self.hurdle_pub.publish(msg)

        return status, angle, ready