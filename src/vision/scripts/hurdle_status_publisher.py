from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Callable, Optional, Tuple

from rclpy.node import Node
from msgs.msg import HurdleResult, MotionCommand, MotionEnd


class HurdleStatus:
    Initial_Pose = 0
    Neck_Left = 15
    Hurdle_Go = 19
    Left_Turn_10 = 23
    Right_Turn_10 = 24
    Backward_half = 9
    Hurdle_Forward_20 = 26
    Back_To_Initial = 27
    Right_Turn_4 = 38
    Left_Turn_4 = 39
    Hurdle_Forward_320 = 42
    Hurdle_Forward_150 = 43
    Hurdle_Forward_400 = 44
    Hurdle_None = 99


@dataclass
class HurdleFeatures:
    # webcam에서 허들 발견 여부
    hurdle_detected: bool = False

    # 라인 검출 정보
    line_point_count: int = 0
    line_follow_angle_deg: Optional[float] = None

    # 로봇에서 두 번째로 가까운 라인점과 중심선 사이의 signed 거리
    # 왼쪽(-), 오른쪽(+)
    line_second_point_distance_px: float = 0.0

    # 검출된 라인점들을 이은 선과 로봇 중심선 사이의 signed 각도
    # 왼쪽(-), 오른쪽(+)
    line_angle_deg: float = 0.0

    # 라인-허들 교점에서 로봇 하단 중심점까지의 화면상 직선거리.
    hurdle_line_intersection_valid: bool = False
    hurdle_intersection_distance_px: Optional[float] = None


class HurdleDecision:
    def __init__(self):
        # 로봇 중심선에서 두 번째 라인점까지의 거리 기준
        self.center_distance_px = 100.0
        # 최초 허들 진입 거리와 정렬 후 전진 모션 선택 거리는 별도로 둔다.
        self.entry_distance_px = 480.0
        self.medium_forward_distance_px = 400.0
        self.short_forward_distance_px = 320.0
        self.near_forward_distance_px = 150.0

        # -100px 경계 양쪽에서 +8~+12도를 함께 허용해 왕복 보정을 줄인다.
        self.center_angle_range = (-10.0, 12.0)
        self.right_angle_range = (8.0, 18.0)
        self.left_angle_range = (-15.0, -8.0)

        self.search_turn = HurdleStatus.Left_Turn_10
        self.back_to_initial_waiting = False
        self.back_to_initial_done = False

    def reset_detection_cycle(self) -> None:
        """Forget the crossed hurdle so the next detection starts a new cycle."""
        self.search_turn = HurdleStatus.Left_Turn_10
        self.back_to_initial_waiting = False
        self.back_to_initial_done = False

    @staticmethod
    def intersection_distance(features: HurdleFeatures) -> Optional[float]:
        if not features.hurdle_line_intersection_valid:
            return None
        try:
            distance = float(features.hurdle_intersection_distance_px)
        except (TypeError, ValueError, OverflowError):
            return None
        return distance if math.isfinite(distance) and distance >= 0.0 else None

    def entry_reason(self, features: HurdleFeatures) -> str:
        """Filter initial confirmation only; preserve the raw webcam detection."""
        if not features.hurdle_detected:
            return 'no_hurdle'
        if features.line_point_count <= 0:
            return 'no_line'
        distance = self.intersection_distance(features)
        if features.line_point_count < 2 or distance is None:
            return 'invalid_intersection'
        return 'within_range' if distance <= self.entry_distance_px else 'too_far'

    def decide(self, features: HurdleFeatures) -> Tuple[int, float, bool]:
        # 1. 허들이 없음
        if not features.hurdle_detected:
            # 27번 명령 대기 중에는 검출이 잠깐 끊겨도 계속 27을 발행
            if self.back_to_initial_waiting:
                return HurdleStatus.Back_To_Initial, 0.0, False

            # 허들이 사라지면 다음 허들 검출을 위해 초기화
            self.back_to_initial_done = False
            return HurdleStatus.Hurdle_None, 0.0, False

        # 2. 허들 최초 검출 후 Back_To_Initial을 계속 발행
        if not self.back_to_initial_done:
            self.back_to_initial_waiting = True
            return HurdleStatus.Back_To_Initial, 0.0, False

        # 3. 라인이 없으면 HurdleMode 내부의 좌우 목 탐색을 요청한다.
        if features.line_point_count <= 0:
            return HurdleStatus.Neck_Left, 0.0, False

        # 4. 라인이 한 점만 보이면 라인이 있는 방향으로 제자리 회전
        # 회전 동작이 끝난 뒤 새 웹캠 프레임으로 다시 판단함
        if features.line_point_count < 2:
            return self._search_for_line(features.line_follow_angle_deg)

        distance = float(features.line_second_point_distance_px)
        angle = float(features.line_angle_deg)

        # 5. 두 번째 라인점의 거리 구간에 따라 목표 각도 범위를 선택
        # ±100px 경계는 먼저 선언된 중앙 구간에 포함함
        if abs(distance) <= self.center_distance_px:
            min_angle, max_angle = self.center_angle_range
        elif distance > self.center_distance_px:
            min_angle, max_angle = self.left_angle_range
        else:
            min_angle, max_angle = self.right_angle_range

        # 6. 목표 각도보다 왼쪽이면 좌회전, 오른쪽이면 우회전
        if angle < min_angle:
            self.search_turn = HurdleStatus.Left_Turn_4
            return HurdleStatus.Left_Turn_4, angle, False

        if angle > max_angle:
            self.search_turn = HurdleStatus.Right_Turn_4
            return HurdleStatus.Right_Turn_4, angle, False

        # 7. 정렬 후 유효한 교점 거리로 전진 모션을 선택한다.
        # 미검출/누락 값을 0px로 간주해 짧은 전진을 실행하지 않는다.
        cross_distance = self.intersection_distance(features)
        if cross_distance is None:
            return HurdleStatus.Hurdle_None, angle, False
        if cross_distance <= self.near_forward_distance_px:
            forward = HurdleStatus.Hurdle_Forward_150
        elif cross_distance <= self.short_forward_distance_px:
            forward = HurdleStatus.Hurdle_Forward_320
        elif cross_distance <= self.medium_forward_distance_px:
            forward = HurdleStatus.Hurdle_Forward_400
        else:
            forward = HurdleStatus.Hurdle_Forward_20
        return forward, angle, True

    def _search_for_line(
        self,
        line_angle_deg: Optional[float],
    ) -> Tuple[int, float, bool]:
        line_angle = float(line_angle_deg or 0.0)

        if line_angle < 0.0:
            self.search_turn = HurdleStatus.Left_Turn_10
        elif line_angle > 0.0:
            self.search_turn = HurdleStatus.Right_Turn_10
        return self.search_turn, line_angle, False


class HurdleStatusPublisher:
    def __init__(
        self,
        node: Node,
        topic_name: str = 'hurdle_result',
        post_crossing_cooldown_sec: float = 2.0,
        monotonic_clock: Optional[Callable[[], float]] = None,
        startup_ignore_sec: float = 3.0,
    ):
        self.node = node
        self.hurdle_decision = HurdleDecision()
        self.post_crossing_cooldown_sec = max(
            0.0,
            float(post_crossing_cooldown_sec),
        )
        self._clock = monotonic_clock or time.monotonic
        self.startup_ignore_sec = max(0.0, float(startup_ignore_sec))
        self._startup_ignore_until: Optional[float] = None
        self._startup_gate_open = self.startup_ignore_sec <= 0.0
        self._crossing_active = False
        self._crossing_started = False
        self._cooldown_until = 0.0
        self._latest_motion_end: Optional[bool] = None
        # 진입 조건을 만족한 검출이 5개 중 3개 이상일 때 허들을 확정합니다.
        # 확정 후에는 거리/검출 변화와 관계없이 Hurdle_Go 종료까지 유지합니다.
        self.hurdle_detection_buffer = deque(maxlen=5)
        self.hurdle_detected = False
        self.entry_eligible = False
        self.entry_reason = 'no_hurdle'
        self.hurdle_pub = self.node.create_publisher(
            HurdleResult,
            topic_name,
            10,
        )
        self.motion_command_sub = self.node.create_subscription(
            MotionCommand,
            'motion_command',
            self._motion_command_callback,
            10,
        )
        self.motion_end_sub = self.node.create_subscription(
            MotionEnd,
            'motion_end',
            self._motion_end_callback,
            10,
        )

    def _log_info(self, message: str) -> None:
        get_logger = getattr(self.node, 'get_logger', None)
        if callable(get_logger):
            get_logger().info(message)

    def _reset_hurdle_detection(self) -> None:
        """Drop entry votes and every state derived from those votes."""
        self.hurdle_detected = False
        self.hurdle_detection_buffer.clear()
        self.entry_eligible = False
        self.entry_reason = 'no_hurdle'
        self.hurdle_decision.reset_detection_cycle()

    def reset_for_mission_start(self) -> None:
        """Restore the complete pre-motion hurdle suppression state."""
        self._startup_ignore_until = None
        self._startup_gate_open = self.startup_ignore_sec <= 0.0
        self._crossing_active = False
        self._crossing_started = False
        self._cooldown_until = 0.0
        self._latest_motion_end = None
        self._reset_hurdle_detection()
        self._log_info(
            "Mission start reset: cleared hurdle votes and latches; "
            "startup suppression is armed."
        )

    def _startup_suppression_active(self) -> bool:
        if self._startup_gate_open:
            return False
        if self._startup_ignore_until is None:
            return True
        if self._clock() < self._startup_ignore_until:
            return True

        # 출발선에서 얻은 raw vote와 확정 latch를 한 번 더 폐기한 뒤,
        # 이 시점 이후에 도착한 새 프레임만 허들 판단에 사용합니다.
        self._reset_hurdle_detection()
        self._startup_gate_open = True
        self._log_info(
            "Startup hurdle suppression ended; detection restarts with "
            "fresh frames."
        )
        return False

    def _motion_command_callback(self, msg: MotionCommand) -> None:
        command = int(msg.command)

        if not self._startup_gate_open:
            if self._startup_ignore_until is None:
                if command == HurdleStatus.Initial_Pose:
                    # 초기자세 중에는 타이머도 시작하지 않고 계속 폐기합니다.
                    self._reset_hurdle_detection()
                    return
                self._startup_ignore_until = (
                    self._clock() + self.startup_ignore_sec
                )
                self._reset_hurdle_detection()
                self._log_info(
                    "Startup hurdle suppression started for "
                    f"{self.startup_ignore_sec:.1f}s after first motion."
                )

            # 금지 구간의 motion command도 허들 정렬 상태에 반영하지 않습니다.
            if self._startup_suppression_active():
                self._reset_hurdle_detection()
                return

        if command == HurdleStatus.Hurdle_Go:
            self._crossing_active = True
            # motion 노드가 command를 먼저 처리했다면 motion_end=false가
            # 이 콜백보다 먼저 도착했을 수 있습니다.
            self._crossing_started = self._latest_motion_end is False
            self._cooldown_until = 0.0
            self.hurdle_decision.reset_detection_cycle()
            self._log_info(
                "Hurdle detection suppressed while Hurdle_Go is running."
            )
            return

        if not self.hurdle_decision.back_to_initial_waiting:
            return

        # 메인 판단 노드가 실제로 27번 모션을 실행한 시점부터 다음 판단 허용
        if command != HurdleStatus.Back_To_Initial:
            return

        self.hurdle_decision.back_to_initial_waiting = False
        self.hurdle_decision.back_to_initial_done = True

    def _motion_end_callback(self, msg: MotionEnd) -> None:
        motion_ended = bool(msg.motion_end)
        self._latest_motion_end = motion_ended

        if not self._crossing_active:
            return

        if not motion_ended:
            self._crossing_started = True
            return

        # motion_command와 motion_end는 서로 다른 노드가 발행하므로 직전
        # 모션의 motion_end=true가 19번 명령보다 늦게 도착할 수 있습니다.
        # 19번 시작을 나타내는 false를 먼저 확인한 뒤의 true만 완료로 봅니다.
        if not self._crossing_started:
            return

        self._crossing_active = False
        self._crossing_started = False
        self._reset_hurdle_detection()
        self._cooldown_until = (
            self._clock() + self.post_crossing_cooldown_sec
        )
        self._log_info(
            "Hurdle_Go completed; hurdle_detected=false and detection "
            "cooldown started for "
            f"{self.post_crossing_cooldown_sec:.1f}s."
        )

    def suppression_reason(self) -> Optional[str]:
        if self._startup_suppression_active():
            return 'startup'
        if self._crossing_active:
            return 'crossing'
        if self._clock() < self._cooldown_until:
            return 'cooldown'
        return None

    def cooldown_remaining_sec(self) -> float:
        if self._crossing_active:
            return 0.0
        return max(0.0, self._cooldown_until - self._clock())

    # publish 함수
    def publish_hurdle_status(
        self,
        hurdle_detected: bool,
        line_point_count: int = 0,
        line_follow_angle_deg: Optional[float] = None,
        line_second_point_distance_px: float = 0.0,
        line_angle_deg: float = 0.0,
        hurdle_line_intersection_valid: bool = False,
        hurdle_intersection_distance_px: Optional[float] = None,
    ) -> Tuple[int, float, bool]:
        suppression_reason = self.suppression_reason()
        features = HurdleFeatures(
            hurdle_detected=bool(hurdle_detected),
            line_point_count=line_point_count,
            line_follow_angle_deg=line_follow_angle_deg,
            line_second_point_distance_px=line_second_point_distance_px,
            line_angle_deg=line_angle_deg,
            hurdle_line_intersection_valid=hurdle_line_intersection_valid,
            hurdle_intersection_distance_px=hurdle_intersection_distance_px,
        )
        if suppression_reason is not None:
            if suppression_reason == 'startup':
                # 출발선 검출은 False 샘플로도 저장하지 않고 완전히 버립니다.
                self._reset_hurdle_detection()
            self.entry_eligible = False
            self.entry_reason = suppression_reason
            status, angle, ready = (
                HurdleStatus.Hurdle_None,
                0.0,
                False,
            )
        else:
            self.entry_reason = self.hurdle_decision.entry_reason(features)
            self.entry_eligible = self.entry_reason in ('no_line', 'within_range')
            if not self.hurdle_detected:
                self.hurdle_detection_buffer.append(self.entry_eligible)
                detected_count = sum(self.hurdle_detection_buffer)
                if (
                    len(self.hurdle_detection_buffer) == 5
                    and detected_count >= 3
                    and self.entry_eligible
                ):
                    self.hurdle_detected = True
                    self._log_info(
                        "Hurdle confirmed by 5-frame majority: "
                        f"true={detected_count}, "
                        f"false={5 - detected_count}; "
                        f"entry={self.entry_reason}; "
                        "hurdle_detected=true until Hurdle_Go completes."
                    )

            features.hurdle_detected = self.hurdle_detected
            status, angle, ready = self.hurdle_decision.decide(features)

        msg = HurdleResult()
        msg.status = int(status)
        msg.angle = float(angle)
        msg.hurdle_ready = bool(ready)
        # Carry the distance from this decision's features with its result.
        # Do not substitute a newer webcam sample when recording the vote.
        cross_distance = (
            self.hurdle_decision.intersection_distance(features)
            if suppression_reason is None else None
        )
        msg.hurdle_intersection_distance_px = (
            cross_distance if cross_distance is not None else -1.0
        )

        self.hurdle_pub.publish(msg)

        return status, angle, ready
