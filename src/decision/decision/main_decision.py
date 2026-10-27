import rclpy
import os
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from collections import deque, Counter
from msgs.msg import LineResult, MotionCommand, MotionEnd, BallResult, HurdleResult
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

#로그 출력 없이 실행하기 위한 장치
QUIET_LOGS = os.environ.get('IRC_QUIET_LOGS', '').lower() in {
    '1', 'true', 'yes', 'on',
}

class Motion:
    Initial_Pose = 0
    Forward = 1
    Left_Half_Forward = 2 
    Right_Half_Forward = 3
    Left_Turn_Half = 4 #사용x
    Right_Turn_Half = 5 #사용x
    Left_Turn = 6  #line tracking 제자리회전
    Right_Turn = 7 #line tracking 제자리회전
    Forward_half = 8 #webcam ball 미세걷기
    Backward_half = 9
    Left_Move = 10 #사이드스텝
    Right_Move = 11
    Pick = 12
    Shoot = 13
    Neck_Up = 14
    Neck_Left = 15
    Neck_Right = 16
    Neck_Center = 17
    Neck_Down = 18
    Hurdle_Go = 19
    Forward_3step = 20 #사용x
    Left_Turn_Curve = 21 #line tracking 곡선구간 회전
    Right_Turn_Curve = 22 #line tracking 곡선구간 회전
    Left_Turn_Mission_10 = 23
    Right_Turn_Mission_10 = 24
    Back_To_Walk = 25
    Hurdle_Forward_20 = 26 #hurdle 전 미세걷기
    Back_To_Initial = 27
    Left_Turn_Mission_5 = 28
    Right_Turn_Mission_5 = 29
    Left_Turn_Afterpick = 30
    Right_Turn_Afterpick = 31
    Shoot_Close = 32
    Shoot_Forward = 33
    Shoot_Mid = 34
    Shoot_62 = 35
    Data_None = 99
    
    # 모션 번호 나열하기


MOTION_NAME = {
    value: name
    for name, value in vars(Motion).items()
    if not name.startswith('_') and isinstance(value, int)
}

class Ball:
    Ball_None = 99
    Ball_Lost = 45
    Pick_Ready = Motion.Pick
    Shoot = Motion.Shoot
    Shoot_Close = Motion.Shoot_Close
    Shoot_Mid = Motion.Shoot_Mid
    Shoot_62 = Motion.Shoot_62

class Line:
    Line_None = 99

class Hurdle:
    Hurdle_Detected = Motion.Hurdle_Forward_20
    Hurdle_Go = 19
    Hurdle_None = 99
    Hurdle_1step = 25



    
class MainDecision(Node):
    
    #시작직전 쌓인 데이터를 비우기
    VISION_RESET_PARTICIPANTS = frozenset({
        'webcam_yolo',
        'realsense_yolo',
        'ball_fusion',
        'hurdle_fusion',
    })

    def __init__(self):
        super().__init__('main_decision')

        #초기값 설정
        self.status = 0
        self.current_mode = "WaitingMode"
        self.motion_end = False
        self.motion_ready = False
        
        # 실제 경기에서는 초기자세와 두 YOLO가 모두 준비되어도
        # 운영자가 Enter를 눌러 /mission/start 서비스를 호출하기 전까지
        # 판단과 모션 명령을 잠근다.
        self.mission_started = False
        self.mission_armed_state = None
        #시작과정에 오류가 있을 때 진단을 위한 초기상태저장값
        self.mission_armed_logged = False
        self.mission_started_at = None
        self.first_motion_after_start_pending = False
        self.mission_start_warning_logged = False
        
        #Enter 입력 전 비전 초기화 확인용 값
        self.vision_reset_pending = False
        self.vision_reset_token = None
        self.vision_reset_acks = set()
        self.mission_vision_reset_complete = False
        
        #Enter 직후 공 데이터 초기화를 위한 값
        self.ball_start_reset_pending = False
        self.ball_start_reset_token = None
        
        #마지막 2개: Enter 직후 첫 보행 자세 복귀 모션 관리
        self.startup_back_to_walk_pending = False
        self.startup_back_to_walk_started = False
        
        # 두 카메라의 YOLO가 첫 추론을 성공하기 전까지
        # 판단과 모션 발행을 모두 막는다.
        self.webcam_yolo_ready = False
        self.realsense_yolo_ready = False
        #3개의 비전 데이터가 모두 준비되었는지 확인
        self.line_data = False
        self.ball_data = False
        self.hurdle_data = False
        #최신 값 저장
        self.latest_line_angle = 0.0
        self.latest_line_follow_point = False
        self.latest_ball_angle = 0.0
        self.latest_ball_in_hand = False
        self.latest_goal_distance_cm = 0.0
        self.latest_hurdle_angle = 0.0

        self.get_logger().info("motion_ready=true 수신 전까지 판단을 대기합니다.")

        #ball
        self.has_ball = False
        self.pick_try_count = 0
        self.pick_done = False
        self.ball_in_hand = False
        #pick이후 회전
        self.ball_count = 0
        self.turn_after_pick = False
        self.backward_after_pick = False
        self.back_to_walk_after_pick = False
        self.turn_count = 0
        # Pick 실패 기록, 이후 line 복귀하면 True로 초기화
        self.pick_failed = False
        #lost
        self.lost_step = 0
        self.lost_found_dir = 0
        self.lost_body_turn_count = 0
        self.lost_initial_pose_done = False
        self.lost_back_to_walk_pending = False
        self.lost_left_line_seen = False
        self.lost_right_line_seen = False
        self.lost_neck_scan_side = 0
        #goal
        self.goal_count = 0
        self.neck_down_pending = False
        self.turn_after_shoot = False
        self.back_to_walk_after_shoot = False
        self.post_shoot_motion = None
        self.turn_shoot = Motion.Right_Turn
        # Shoot 명령 발행부터 Shoot 이후 강제회전 종료까지 공/골대 검출을 모두 중지한다.
        self.post_shoot_detection_suppressed = False
        self.shoot_in_progress = False
        self.shoot_motion_started = False
        # MainDecision은 Back_To_Initial/Shoot_Forward 이후 verified BallResult를
        # 기다렸다가 전달받은 모션 상태만 실행한다.
        self.pre_shoot_result_waiting = False
        self.pre_shoot_verified_result = None
        self.back_to_walk_after_pre_shoot_cancel = False
        self.pre_shoot_return_to_walk_started = False
        #hurdle
        self.hurdle_step = 0
        self.hurdle_count = 0
        self.hurdle_ready = False
        self.hurdle_detected = False
        self.hurdle_go_active = False
        self.hurdle_go_started = False
        self.hurdle_ball_suppressed = False
        self.hurdle_previous_ball_active = True
        MainDecision._reset_hurdle_line_search(self)
        # 첫 실제 모션 후 3초가 끝날 때까지 허들 검출 금지
        self.hurdle_ignore_until = None
        self.hurdle_ignore_active = True

        # 최근 5개의 비전 상태를 저장
        # line과 BallMode는 판단 시점의 최근 3개만 다수결에 사용
        self.line_buffer = deque(maxlen=5)
        self.line_vote_detail_buffer = deque(maxlen=5)
        self.line_frame_sequence = 0
        self.ball_buffer = deque(maxlen=5)
        self.ball_vote_detail_buffer = deque(maxlen=5)
        self.ball_frame_sequence = 0
        self.hurdle_buffer = deque(maxlen=5)
        self.hurdle_ready_buffer = deque(maxlen=5)

        # QoS
        # 두 YOLO의 최신 준비 상태를 유지한다.
        vision_ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # 최신 모션 상태와 미션 준비 상태를 유지한다.
        motion_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # 비전 초기화 요청과 응답을 안정적으로 전달한다.
        vision_reset_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 공·골대 비전의 최신 활성화 상태를 유지한다.
        vision_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # subscribe
        # yolo_vision에서 라인 상태와 각도를 받는다.
        self.line_result_sub = self.create_subscription(
            LineResult, 'line_result', self.LineResultCallback, 10
        )
        # ball_vision_fusion에서 공과 골대의 판단 결과를 받는다.
        self.ball_result_sub = self.create_subscription(
            BallResult, 'ball_result', self.BallResultCallback, 10
        )
        # webcam_hurdle_publisher에서 허들 상태와 각도를 받는다.
        self.hurdle_result_sub = self.create_subscription(
            HurdleResult, 'hurdle_result', self.HurdleResultCallback, 10
        )
        # yolo_vision에서 웹캠 YOLO의 준비 상태를 받는다.
        self.webcam_yolo_ready_sub = self.create_subscription(
            Bool,
            '/vision/webcam_yolo_ready',
            self.WebcamYoloReadyCallback,
            vision_ready_qos,
        )
        # realsense_yolo_detector에서 RealSense YOLO 준비 상태를 받는다.
        self.realsense_yolo_ready_sub = self.create_subscription(
            Bool,
            '/vision/realsense_yolo_ready',
            self.RealSenseYoloReadyCallback,
            vision_ready_qos,
        )
        # motion_main_node에서 모션 준비 상태와 종료 여부를 받는다.
        self.motion_end_sub = self.create_subscription(
            MotionEnd, 'motion_end', self.MotionEndCallback, motion_state_qos
        )
        # 4개 비전 노드에서 초기화 완료 응답을 받는다.
        self.vision_reset_ack_sub = self.create_subscription(
            String,
            '/mission/vision_reset_ack',
            self.VisionResetAckCallback,
            vision_reset_qos,
        )

        # publish
        # mission_start_keyboard로 Enter 입력 준비 상태를 보낸다.
        self.mission_armed_pub = self.create_publisher(
            Bool,
            '/mission/armed',
            motion_state_qos,
        )
        # Enter 입력 준비 상태를 설정
        self._set_mission_armed_state(False)
        
        # webcam·RealSense·공·허들 비전 노드로 초기화 요청을 보낸다.
        self.vision_reset_pub = self.create_publisher(
            String,
            '/mission/vision_reset',
            vision_reset_qos,
        )
        # webcam·RealSense·공 융합 노드로 공 초기화 요청을 보낸다.
        self.ball_start_reset_pub = self.create_publisher(
            String, '/mission/ball_reset', vision_reset_qos,
        )
        # 모션 노드와 비전 상태 판단기로 실행할 모션 번호를 보낸다.
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)
        # RealSense 검출기와 공 융합 노드로 공 검출 ON/OFF를 보낸다.
        self.ball_active_pub = self.create_publisher(
            Bool,
            '/vision/ball_active',
            vision_mode_qos,
        )
        # RealSense 검출기로 골대 검출 ON/OFF를 보낸다.
        self.hoop_active_pub = self.create_publisher(
            Bool,
            '/vision/hoop_active',
            vision_mode_qos,
        )
        # 공 융합 노드로 웹캠 공 결과 사용 여부를 보낸다.
        self.webcam_ball_allowed_pub = self.create_publisher(
            Bool,
            '/vision/webcam_ball_allowed',
            vision_mode_qos,
        )

        # service
        # mission_start_keyboard에서 Enter로 보낸 시작 요청을 받는다.
        self.mission_start_service = self.create_service(
            Trigger,
            '/mission/start',
            self.MissionStartCallback,
        )

        # timer
        # 미션 시작 후 첫 모션 지연을 0.1초마다 확인한다.
        self.mission_start_watchdog = self.create_timer(
            0.1,
            self._check_mission_start_latency,
        )

        #vision의 공 검출 상태 저장
        self.ball_vision_active = None
        self.hoop_vision_active = None
        self.webcam_ball_allowed = None
        
        #공 검출 결과 사용 여부 전송
        self._set_webcam_ball_allowed(
            True,
            reason='startup: wait for RealSense <= 120cm',
        )
        self._set_vision_activity(
            ball_active=True,
            hoop_active=False,
            reason='startup: search for ball',
        )

    def _vision_stack_ready(self):
        """Return true only after both camera YOLO pipelines have inferred."""
        return bool(
            getattr(self, 'webcam_yolo_ready', True)
            and getattr(self, 'realsense_yolo_ready', True)
        )

    def _mission_has_started(self):
        return bool(self.mission_started)

    def _mission_armed(self):
        """Finish preparation before Enter can start Back_To_Walk."""
        return bool(
            getattr(self, 'motion_ready', False)
            and getattr(self, 'motion_end', False)
            and MainDecision._vision_stack_ready(self)
            and getattr(self, 'mission_vision_reset_complete', True)
            and not getattr(self, 'vision_reset_pending', False)
        )

    def _prepare_mission_vision(self):
        """Run the existing vision reset before the Enter waiting screen."""
        MainDecision._reset_vision_decision_cycle(self)
        reset_publisher = getattr(self, 'vision_reset_pub', None)
        if reset_publisher is None:
            self.mission_vision_reset_complete = True
            return
        self.vision_reset_pending = True
        self.vision_reset_acks = set()
        self.vision_reset_token = str(time.monotonic_ns())
        reset_publisher.publish(String(data=self.vision_reset_token))
        self.get_logger().info(
            '[MISSION PREPARE] Resetting vision before Enter is enabled.'
        )

    def _set_mission_armed_state(self, armed):
        """Publish armed changes for the launch-terminal waiting screen."""
        armed = bool(armed)
        previous = getattr(self, 'mission_armed_state', None)
        self.mission_armed_state = armed
        publisher = getattr(self, 'mission_armed_pub', None)
        if publisher is not None and previous != armed:
            publisher.publish(Bool(data=armed))
        return previous != armed

    def _maybe_log_mission_armed(self):
        if MainDecision._mission_has_started(self):
            MainDecision._set_mission_armed_state(self, False)
            return False
        if (
            not getattr(self, 'mission_vision_reset_complete', True)
            and not getattr(self, 'vision_reset_pending', False)
            and self.motion_ready
            and self.motion_end
            and MainDecision._vision_stack_ready(self)
        ):
            MainDecision._prepare_mission_vision(self)
        armed = MainDecision._mission_armed(self)
        MainDecision._set_mission_armed_state(self, armed)
        if not armed:
            self.mission_armed_logged = False
            return False
        if getattr(self, 'mission_armed_logged', False):
            return False

        self.mission_armed_logged = True
        self.get_logger().info('[MISSION ARMED] Press Enter to start')
        return True

    def MissionStartCallback(self, _request, response):
        received_at = time.monotonic()
        self.get_logger().info(
            '[MISSION REQUEST RECEIVED] /mission/start: '
            f'receive_monotonic={received_at:.6f}, '
            f'started={MainDecision._mission_has_started(self)}, '
            f'motion_ready={self.motion_ready}, motion_end={self.motion_end}, '
            f'vision_ready={MainDecision._vision_stack_ready(self)}'
        )
        if MainDecision._mission_has_started(self):
            response.success = True
            response.message = 'Mission already started.'
            return response

        if not MainDecision._mission_armed(self):
            response.success = False
            response.message = (
                'Mission is not armed: wait for initial pose and both YOLOs.'
            )
            self.get_logger().info(
                '[MISSION START REJECTED] Wait for initial pose and both '
                'YOLO pipelines to become ready.'
            )
            return response

        # Enter 대기 중 다시 쌓인 공 검출/27번 요청도 폐기한다.
        # 초기화 응답을 기다리지 않고 25번은 즉시 실행한다.
        ball_reset_publisher = getattr(self, 'ball_start_reset_pub', None)
        if ball_reset_publisher is not None:
            self.ball_start_reset_pending = True
            self.ball_start_reset_token = str(time.monotonic_ns())
            ball_reset_publisher.publish(String(data=self.ball_start_reset_token))
        MainDecision._reset_vision_decision_cycle(self)
        self.latest_line_angle = 0.0
        self.latest_line_follow_point = False
        self.latest_ball_angle = 0.0
        self.latest_ball_in_hand = False
        self.latest_goal_distance_cm = 0.0
        self.latest_hurdle_angle = 0.0
        self.line_status = Line.Line_None
        self.ball_status = Ball.Ball_None
        self.ball_in_hand = False
        self.hurdle_status = Hurdle.Hurdle_None
        self.hurdle_ready = False
        self.hurdle_detected = False
        self.hurdle_ignore_until = None
        self.hurdle_ignore_active = True
        self.mission_started = True
        MainDecision._set_mission_armed_state(self, False)
        self.mission_started_at = time.monotonic()
        self.first_motion_after_start_pending = True
        self.mission_start_warning_logged = False
        self.current_mode = 'WaitingMode'
        self.startup_back_to_walk_pending = True
        self.startup_back_to_walk_started = False
        self.status = Motion.Back_To_Walk
        self.get_logger().info(
            '[MISSION START] Enter accepted: execute Back_To_Walk now; '
            'decide from cached vision at MotionEnd.'
        )
        MainDecision.MotionCommand(self)
        response.success = True
        response.message = 'Mission start accepted; Back_To_Walk commanded.'
        return response

    def VisionResetAckCallback(self, msg):
        if (
            getattr(self, 'ball_start_reset_pending', False)
            and str(msg.data) == (
                f"ball_start|{self.ball_start_reset_token}"
            )
        ):
            self.ball_start_reset_pending = False
            # 공 초기화 때문에 라인/허들 캐시나 다음 보행 판단을 늦추지 않는다.
            self.ball_buffer.clear()
            self.ball_buffer.extend([Ball.Ball_None] * 3)
            details = getattr(self, 'ball_vote_detail_buffer', None)
            if details is not None:
                details.clear()
            return
        if not getattr(self, 'vision_reset_pending', False):
            return

        try:
            participant, token = str(msg.data).split('|', 1)
        except ValueError:
            return
        if token != getattr(self, 'vision_reset_token', None):
            return

        expected = set(MainDecision.VISION_RESET_PARTICIPANTS)
        if participant not in expected:
            return
        self.vision_reset_acks.add(participant)
        missing = expected - self.vision_reset_acks
        if missing:
            return

        # Enter 전 초기화 중 도착한 중간 결과만 폐기한다.
        MainDecision._reset_vision_decision_cycle(self)
        self.vision_reset_pending = False
        self.mission_vision_reset_complete = True
        self.get_logger().info(
            '[MISSION VISION RESET] complete: ready for Enter.'
        )
        MainDecision._maybe_log_mission_armed(self)

    def _check_mission_start_latency(self):
        if (
            not MainDecision._mission_has_started(self)
            or not getattr(self, 'first_motion_after_start_pending', False)
            or getattr(self, 'mission_start_warning_logged', False)
            or getattr(self, 'mission_started_at', None) is None
        ):
            return

        elapsed = time.monotonic() - self.mission_started_at
        if elapsed < 0.5:
            return

        self.mission_start_warning_logged = True
        if getattr(self, 'vision_reset_pending', False):
            missing = sorted(
                set(MainDecision.VISION_RESET_PARTICIPANTS)
                - set(getattr(self, 'vision_reset_acks', set()))
            )
            self.get_logger().info(
                '[MISSION START WAIT] Waiting for vision reset ack: '
                + ','.join(missing)
            )
            return
        self.get_logger().info(
            '[MISSION START WAIT] No first motion after '
            f'{elapsed:.2f}s: line={len(self.line_buffer)}/3, '
            f'ball={len(self.ball_buffer)}/3, '
            f'hurdle={len(self.hurdle_buffer)}/3.'
        )

    def _set_yolo_readiness(self, source, ready):
        attr_name = f'{source}_yolo_ready'
        ready = bool(ready)
        previous_stack_ready = MainDecision._vision_stack_ready(self)
        previous_source_ready = bool(getattr(self, attr_name, False))
        setattr(self, attr_name, ready)
        stack_ready = MainDecision._vision_stack_ready(self)

        if previous_source_ready != ready:
            self.get_logger().info(
                f'[VisionStartup] {source}_yolo='
                f'{"READY" if ready else "NOT_READY"}'
            )

        # 재시작이나 장애로 어느 한쪽 준비 신호가 내려가면, 그 전에
        # 쌓인 결과가 재준비 직후 의사결정에 섞이지 않게 즉시 버린다.
        if not stack_ready:
            if not MainDecision._mission_has_started(self):
                self.mission_vision_reset_complete = False
            MainDecision._reset_vision_decision_cycle(self)
            MainDecision._maybe_log_mission_armed(self)
            if previous_stack_ready or previous_source_ready != ready:
                self.get_logger().info(
                    '[VisionStartup] 보행 잠금: webcam/RealSense YOLO의 '
                    '첫 정상 추론을 기다립니다.'
                )
            return False

        if not previous_stack_ready:
            MainDecision._reset_vision_decision_cycle(self)
            self.get_logger().info(
                '[VisionStartup] webcam/RealSense YOLO READY: '
                '초기자세 완료와 Enter 입력을 기다립니다.'
            )
        MainDecision._maybe_log_mission_armed(self)
        return True

    def _set_webcam_ball_allowed(self, allowed, reason=''):
        """Allow webcam ball output; Fusion still applies the 120cm gate."""
        allowed = bool(allowed)
        if getattr(self, 'webcam_ball_allowed', None) == allowed:
            return False
        publisher = getattr(self, 'webcam_ball_allowed_pub', None)
        if publisher is not None:
            publisher.publish(Bool(data=allowed))
        self.webcam_ball_allowed = allowed
        logger = getattr(self, 'get_logger', None)
        if callable(logger):
            suffix = f' ({reason})' if reason else ''
            logger().info(
                '[WebcamBallGate] '
                f'allowed={"ON" if allowed else "OFF"}{suffix}'
            )
        return True

    def WebcamYoloReadyCallback(self, msg: Bool):
        self._set_yolo_readiness('webcam', msg.data)

    def RealSenseYoloReadyCallback(self, msg: Bool):
        self._set_yolo_readiness('realsense', msg.data)

    def _set_vision_activity(
        self,
        ball_active,
        hoop_active,
        reason='',
    ):
        """Enable exactly the detector needed by the current match phase."""
        ball_active = bool(ball_active)
        hoop_active = bool(hoop_active)
        if ball_active and hoop_active:
            raise ValueError('ball and hoop detection cannot both be active')

        previous_ball = getattr(self, 'ball_vision_active', None)
        previous_hoop = getattr(self, 'hoop_vision_active', None)
        if previous_ball == ball_active and previous_hoop == hoop_active:
            return False

        ball_pub = getattr(self, 'ball_active_pub', None)
        hoop_pub = getattr(self, 'hoop_active_pub', None)

        # 전환 순간에도 두 검출기가 동시에 켜지지 않도록 OFF를 먼저 보낸다.
        if (
            previous_ball is not False
            and not ball_active
            and ball_pub is not None
        ):
            ball_pub.publish(Bool(data=False))
        if (
            previous_hoop is not False
            and not hoop_active
            and hoop_pub is not None
        ):
            hoop_pub.publish(Bool(data=False))
        if previous_ball is not True and ball_active and ball_pub is not None:
            ball_pub.publish(Bool(data=True))
        if previous_hoop is not True and hoop_active and hoop_pub is not None:
            hoop_pub.publish(Bool(data=True))

        self.ball_vision_active = ball_active
        self.hoop_vision_active = hoop_active
        logger = getattr(self, 'get_logger', None)
        if callable(logger):
            suffix = f' ({reason})' if reason else ''
            logger().info(
                '[VisionMode] '
                f'ball={"ON" if ball_active else "OFF"}, '
                f'hoop={"ON" if hoop_active else "OFF"}{suffix}'
            )
        return True

    # 콜백함수에서 모션 종료 여부를 업데이트
    def MotionEndCallback(self, motion_end_msg:MotionEnd):
        #메시지 받기 이전 상태 저장
        was_ready = self.motion_ready
        was_motion_end = self.motion_end
        #최신 상태 갱신
        self.motion_ready = motion_end_msg.motion_ready
        self.motion_end = motion_end_msg.motion_end

        if getattr(self, 'startup_back_to_walk_pending', False):
            if not self.motion_end:
                self.startup_back_to_walk_started = True
            elif self.startup_back_to_walk_started:
                self.startup_back_to_walk_pending = False
                self.startup_back_to_walk_started = False

        if (
            getattr(self, 'back_to_walk_after_pre_shoot_cancel', False)
            and not self.motion_end
        ):
            self.pre_shoot_return_to_walk_started = True

        #허들모드 잠금해제
        if self.hurdle_go_active:
            if not self.motion_end:
                self.hurdle_go_started = True
            elif self.hurdle_go_started:
                self.hurdle_go_active = False
                self.hurdle_go_started = False
                self.hurdle_detected = False
                if getattr(self, 'hurdle_ball_suppressed', False):
                    self.hurdle_ball_suppressed = False
                    MainDecision._set_vision_activity(
                        self,
                        ball_active=self.hurdle_previous_ball_active,
                        hoop_active=self.hoop_vision_active,
                        reason='hurdle crossing completed: restore ball detection',
                    )
                self.get_logger().info(
                    "Hurdle_Go 완료: hurdle_detected=false, "
                    "허들 모드 잠금을 해제하고 라인 트래킹 복귀를 허용합니다."
                )

        # Shoot 모션의 false(실행 중) -> true(완료) 전이만 확인한다.
        # 검출은 Shoot 명령 발행 시점부터 이미 OFF이며, 여기서는
        # 강제회전 시작 전에 OFF 상태를 한 번 더 보장한다.
        if getattr(self, 'shoot_in_progress', False):
            if not self.motion_end:
                self.shoot_motion_started = True
            elif getattr(self, 'shoot_motion_started', False):
                self.shoot_in_progress = False
                self.shoot_motion_started = False
                MainDecision._set_vision_activity(
                    self,
                    ball_active=False,
                    hoop_active=False,
                    reason=(
                        'shoot motion completed: wait for post-shoot turn'
                    ),
                )

        self.get_logger().info(
            f"motion_ready: {self.motion_ready}, motion_end: {self.motion_end}"
        )
        if self.motion_ready and not was_ready:
            if MainDecision._vision_stack_ready(self):
                self.get_logger().info(
                    "초기자세 완료 확인: Enter 입력을 기다립니다."
                )
            else:
                self.get_logger().info(
                    "초기자세 완료 확인: 두 YOLO 준비 전이므로 보행을 대기합니다."
                )

        MainDecision._maybe_log_mission_armed(self)

        # 모션 실행 중 쌓인 최신 비전 결과를 모션 종료 즉시 집계합니다.
        # 다음 line/ball/hurdle 콜백을 기다리지 않도록 모션이 실제로 종료되는 전환 시점에 한 번만 판단
        if self.motion_end and not was_motion_end:
            if getattr(self, 'pre_shoot_result_waiting', False):
                self._reset_vision_decision_cycle()
                if not self._consume_pre_shoot_verified_result():
                    self.get_logger().info(
                        "[PreShoot] 슈팅 준비 모션 MotionEnd: "
                        "BallStatusPublisher의 verified 결과를 기다립니다."
                    )
                return
            lost_third_turn = (
                getattr(self, 'current_mode', None) == "LostMode"
                and getattr(self, 'lost_step', 0) == 4
                and getattr(self, 'lost_body_turn_count', 0) == 3
            )
            hurdle_third_turn = (
                getattr(self, 'current_mode', None) == "HurdleMode"
                and getattr(self, 'hurdle_line_search_step', 0) == 4
                and getattr(self, 'hurdle_line_turn_count', 0) == 3
            )
            if (
                (lost_third_turn or hurdle_third_turn)
                and getattr(self, 'status', None) in (
                    Motion.Left_Turn_Afterpick,
                    Motion.Right_Turn_Afterpick,
                )
            ):
                # 허들 탐색도 Lost와 동일하게 세 번째 회전만 새 라인을 확인한다.
                self.line_data = False
                self.line_buffer.clear()
                detail_buffer = getattr(self, 'line_vote_detail_buffer', None)
                if detail_buffer is not None:
                    detail_buffer.clear()
                label = 'LostForcedTurns' if lost_third_turn else 'HurdleForcedTurns'
                self.get_logger().info(
                    f"[{label}] completed=3: cleared line votes; "
                    "waiting for 3 new line results."
                )
                return
            self._try_decision_from_cached_results()
        
        
    def LineResultCallback(self, line_msg:LineResult):
        if getattr(self, 'vision_reset_pending', False):
            return
        self.latest_line_angle = line_msg.angle
        self.latest_line_follow_point = line_msg.follow_point

        if (
            not self.motion_ready
            or not MainDecision._vision_stack_ready(self)
            or not MainDecision._mission_has_started(self)
        ):
            return

        #최신 데이터 갱신
        line_status = int(line_msg.status)
        self.line_buffer.append(line_status)
        self.line_frame_sequence = getattr(self, 'line_frame_sequence', 0) + 1
        neck_scan_side = getattr(self, 'lost_neck_scan_side', 0)
        if (
            getattr(self, 'current_mode', None) == "LostMode"
            and not self.motion_end
            and line_status != Line.Line_None
        ):
            if (
                neck_scan_side == -1
                and getattr(self, 'status', None) == Motion.Neck_Left
                and not getattr(self, 'lost_left_line_seen', False)
            ):
                self.lost_left_line_seen = True
                self.get_logger().info(
                    "[LostNeckLineSeen] side=left "
                    f"seq={self.line_frame_sequence} status={line_status} "
                    f"point_count={int(getattr(line_msg, 'point_count', 0))}"
                )
            elif (
                neck_scan_side == 1
                and getattr(self, 'status', None) == Motion.Neck_Right
                and not getattr(self, 'lost_right_line_seen', False)
            ):
                self.lost_right_line_seen = True
                self.get_logger().info(
                    "[LostNeckLineSeen] side=right "
                    f"seq={self.line_frame_sequence} status={line_status} "
                    f"point_count={int(getattr(line_msg, 'point_count', 0))}"
                )
        if (
            getattr(self, 'current_mode', None) == "HurdleMode"
            and not self.motion_end
            and line_status != Line.Line_None
        ):
            side = getattr(self, 'hurdle_neck_scan_side', 0)
            if side == -1 and self.status == Motion.Neck_Left:
                self.hurdle_left_line_seen = True
            elif side == 1 and self.status == Motion.Neck_Right:
                self.hurdle_right_line_seen = True
        detail_buffer = getattr(self, 'line_vote_detail_buffer', None)
        if detail_buffer is not None:
            detail_buffer.append({
                'sequence': self.line_frame_sequence,
                'status': line_status,
                'angle': float(line_msg.angle),
                'point_count': int(getattr(line_msg, 'point_count', 0)),
                'decision_type': str(
                    getattr(line_msg, 'decision_type', 'unknown')
                ),
                'decision_angle': float(
                    getattr(line_msg, 'decision_angle', math.nan)
                ),
                'line_distance': float(
                    getattr(line_msg, 'line_distance', math.nan)
                ),
                'curve_a': float(getattr(line_msg, 'curve_a', math.nan)),
            })
        if self.motion_end == True:
            self._try_decision_from_cached_results()
        else:
            self.get_logger().info(f"line: motion not ended yet")
            
    def BallResultCallback(self, ball_msg:BallResult):
        if (
            getattr(self, 'vision_reset_pending', False)
            or getattr(self, 'ball_start_reset_pending', False)
        ):
            return
        self.latest_ball_angle = ball_msg.angle
        self.latest_ball_in_hand = bool(
            getattr(ball_msg, 'ball_in_hand', False)
        )
        self.latest_goal_distance_cm = float(
            getattr(ball_msg, 'goal_distance_cm', 0.0)
        )
        if not MainDecision._mission_has_started(self):
            return
        self.ball_frame_sequence = getattr(
            self, 'ball_frame_sequence', 0
        ) + 1
        detail = MainDecision._ball_result_detail(
            ball_msg,
            self.ball_frame_sequence,
        )

        if getattr(self, 'pre_shoot_result_waiting', False):
            if bool(getattr(ball_msg, 'pre_shoot_verified', False)):
                status = int(ball_msg.status)
                goal_out_of_range = (
                    status == Ball.Ball_None
                    and self.latest_ball_in_hand
                    and math.isfinite(self.latest_goal_distance_cm)
                    and self.latest_goal_distance_cm > 120.0
                )
                if (
                    status not in (Ball.Ball_None, Motion.Back_To_Initial)
                    or goal_out_of_range
                ):
                    self.pre_shoot_verified_result = (
                        status,
                        float(ball_msg.angle),
                        bool(getattr(ball_msg, 'ball_in_hand', False)),
                        float(getattr(ball_msg, 'goal_distance_cm', 0.0)),
                        detail,
                    )
                    self._consume_pre_shoot_verified_result()
            return

        if (
            not self.motion_ready
            or not MainDecision._vision_stack_ready(self)
            or not MainDecision._mission_has_started(self)
        ):
            return

        ball_status = ball_msg.status
        if getattr(self, 'hurdle_ball_suppressed', False):
            # Keep samples flowing, but reject in-flight ball detections.
            ball_status = Ball.Ball_None
            detail['status'] = Ball.Ball_None
        self.ball_buffer.append(ball_status)
        detail_buffer = getattr(self, 'ball_vote_detail_buffer', None)
        if detail_buffer is not None:
            detail_buffer.append(detail)
        
        if self.motion_end == True:
            self._try_decision_from_cached_results()
        else:
            self.get_logger().info(f"ball: motion not ended yet")

    def _consume_pre_shoot_verified_result(self):
        result = getattr(self, 'pre_shoot_verified_result', None)
        if (
            not getattr(self, 'pre_shoot_result_waiting', False)
            or result is None
            or not self.motion_ready
            or not self.motion_end
            or not MainDecision._mission_has_started(self)
        ):
            return False

        status, angle, ball_in_hand, goal_distance_cm = result[:4]
        detail = result[4] if len(result) >= 5 else None
        self.pre_shoot_result_waiting = False
        self.pre_shoot_verified_result = None
        self._reset_vision_decision_cycle()
        detail_buffer = getattr(self, 'ball_vote_detail_buffer', None)
        if detail_buffer is not None and detail is not None:
            detail_buffer.append(detail)
        self.latest_ball_angle = angle
        self.latest_ball_in_hand = ball_in_hand
        self.latest_goal_distance_cm = goal_distance_cm
        self.ball_status = status
        self.ball_angle = angle
        self.ball_in_hand = ball_in_hand
        self.current_mode = "BallMode"
        self.get_logger().info(
            "[PreShoot] BallStatusPublisher verified result: "
            f"status={MOTION_NAME.get(status, 'Unknown')}, "
            f"distance={goal_distance_cm:.1f}cm, angle={angle:+.1f}deg"
        )
        if status == Ball.Ball_None:
            self.back_to_walk_after_pre_shoot_cancel = True
            self.pre_shoot_return_to_walk_started = False
            self.status = Motion.Back_To_Walk
            self.get_logger().info(
                "[PreShoot] 골대 120cm 초과 재확인: "
                "슛 준비 취소, Back_To_Walk 완료 후 라인 복귀."
            )
            self.MotionCommand()
            return True
        self.BallMode()
        return True

    @staticmethod
    def _ball_result_detail(ball_msg, sequence):
        def finite_value(name, fallback=math.nan):
            try:
                value = float(getattr(ball_msg, name, fallback))
            except (TypeError, ValueError):
                return math.nan
            return value if math.isfinite(value) else math.nan

        decision_angle = finite_value('angle', 0.0)
        return {
            'sequence': int(sequence),
            'status': int(ball_msg.status),
            'decision_angle': decision_angle,
            'measured_angle': finite_value(
                'detected_angle', decision_angle
            ),
            'x_distance_px': finite_value('x_distance_px'),
            'y_distance_px': finite_value('y_distance_px'),
            'ball_x_px': finite_value('ball_x_px'),
            'ball_y_px': finite_value('ball_y_px'),
            'goal_x_px': finite_value('goal_x_px'),
            'goal_y_px': finite_value('goal_y_px'),
            'goal_distance_cm': finite_value('goal_distance_cm'),
            'ball_in_hand': bool(
                getattr(ball_msg, 'ball_in_hand', False)
            ),
        }

    @staticmethod
    def _log_ball_action_evidence(node, action_status):
        if QUIET_LOGS:
            return
        details = list(
            getattr(node, 'ball_vote_detail_buffer', [])
        )[-3:]
        label = (
            'PickDecisionEvidence'
            if action_status == Ball.Pick_Ready
            else 'ShootDecisionEvidence'
        )
        action_name = MOTION_NAME.get(action_status, 'Unknown')

        if not details:
            node.get_logger().info(
                f'[{label}] action={action_name}, frames=0, evidence=N/A'
            )
            return

        def value_text(value, unit):
            return (
                f'{value:.2f}{unit}'
                if math.isfinite(value)
                else 'N/A'
            )

        frame_logs = []
        for detail in details:
            status_name = MOTION_NAME.get(
                detail['status'], 'Unknown'
            )
            if action_status == Ball.Pick_Ready:
                measurements = (
                    f"ball_raw_x={value_text(detail['ball_x_px'], 'px')} "
                    f"ball_raw_y={value_text(detail['ball_y_px'], 'px')} "
                    f"x_offset={value_text(detail['x_distance_px'], 'px')} "
                    f"y_offset={value_text(detail['y_distance_px'], 'px')}"
                )
            else:
                measurements = (
                    f"goal_raw_x={value_text(detail['goal_x_px'], 'px')} "
                    f"goal_raw_y={value_text(detail['goal_y_px'], 'px')} "
                    f"goal_distance="
                    f"{value_text(detail['goal_distance_cm'], 'cm')}"
                )
            frame_logs.append(
                f"seq={detail['sequence']} status={status_name} "
                f"{measurements} measured_angle="
                f"{value_text(detail['measured_angle'], 'deg')} "
                f"decision_angle="
                f"{value_text(detail['decision_angle'], 'deg')}"
            )

        node.get_logger().info(
            f'[{label}] action={action_name}, frames={len(details)} | '
            + ' | '.join(frame_logs)
        )

    def HurdleResultCallback(self, hurdle_msg:HurdleResult):
        if getattr(self, 'vision_reset_pending', False):
            return
        ignore_until = getattr(self, 'hurdle_ignore_until', None)
        ignore_hurdle = (
            ignore_until is None or time.monotonic() < ignore_until
        )

        if ignore_hurdle:
            # 금지 시간에 들어온 양성 결과는 래치하거나 기억하지 않습니다.
            # Hurdle_None은 다른 비전 판단이 계속 진행될 수 있게 합니다.
            if not getattr(self, 'hurdle_ignore_active', False):
                self.hurdle_buffer.clear()
                self.hurdle_ready_buffer.clear()
            self.hurdle_ignore_active = True
            self.hurdle_detected = False
            self.hurdle_ready = False
            self.hurdle_go_active = False
            self.hurdle_go_started = False
            self.hurdle_status = Hurdle.Hurdle_None
            self.hurdle_angle = 0.0
        elif getattr(self, 'hurdle_ignore_active', False):
            # gate가 열리는 순간 금지 구간의 강제 False 샘플까지 버리고,
            # 이후에 도착한 프레임만으로 허들을 판단합니다.
            self.hurdle_buffer.clear()
            self.hurdle_ready_buffer.clear()
            self.hurdle_data = False
            self.hurdle_ignore_active = False

        hurdle_status = (
            Hurdle.Hurdle_None if ignore_hurdle else hurdle_msg.status
        )
        hurdle_ready = False if ignore_hurdle else hurdle_msg.hurdle_ready
        self.latest_hurdle_angle = 0.0 if ignore_hurdle else hurdle_msg.angle

        if (
            not self.motion_ready
            or not MainDecision._vision_stack_ready(self)
            or not MainDecision._mission_has_started(self)
        ):
            return

        #허들이 검출되고, 허들이 2번 이내 실행되었다면 hurdle mode 유지
        confirmed_hurdle_detected = bool(
            hurdle_ready or hurdle_status != Hurdle.Hurdle_None
        )
        if (
            not self.hurdle_detected
            and self.hurdle_count < 2
            and confirmed_hurdle_detected
        ):
            self.hurdle_detected = True
            self.get_logger().info(
                "허들 비전 다수결 확정 수신: hurdle_detected=true, "
                "Hurdle_Go 완료까지 허들 모드를 유지합니다."
            )

        self.hurdle_buffer.append(hurdle_status)
        self.hurdle_ready_buffer.append(bool(hurdle_ready))
        if self.motion_end == True:
            self._try_decision_from_cached_results()
        else:
            self.get_logger().info(f"hurdle: motion not ended yet")

    #모션 종료 후 저장된 비전값으로 다음 행동 결정
    def _try_decision_from_cached_results(self):
        # Pre-Shoot Back_To_Initial/Shoot_Forward 뒤에는 일반 투표 대신
        # BallStatusPublisher가 표시한 verified 결과 하나만 사용한다.
        if getattr(self, 'pre_shoot_result_waiting', False):
            return False
        if getattr(self, 'startup_back_to_walk_pending', False):
            return False
        if (
            getattr(self, 'vision_reset_pending', False)
            or
            not self.motion_ready
            or not self.motion_end
            or not MainDecision._vision_stack_ready(self)
            or not MainDecision._mission_has_started(self)
        ):
            return False

        # 이미 현재 종료 사이클의 판단을 시작했다면 중복 명령을 막습니다.
        if self.line_data and self.ball_data and self.hurdle_data:
            return False

        returning_from_pre_shoot = getattr(
            self, 'back_to_walk_after_pre_shoot_cancel', False
        )
        if (
            returning_from_pre_shoot
            and not getattr(self, 'pre_shoot_return_to_walk_started', False)
        ):
            return False

        if (
            len(self.line_buffer) < 3
            or (
                not returning_from_pre_shoot
                and (
                    len(self.ball_buffer) < 3
                    or len(self.hurdle_buffer) < 3
                )
            )
        ):
            self.get_logger().info(
                "저장된 비전 데이터 부족: "
                f"line={len(self.line_buffer)}, "
                f"ball={len(self.ball_buffer)}, "
                f"hurdle={len(self.hurdle_buffer)}"
            )
            return False

        # 라인은 모션 중 저장된 값 가운데 최근 3개만
        # 다수결에 사용합니다. 세 상태가 모두 다르면
        # 현재 자세를 가장 잘 반영하는 최신 상태를 선택합니다.
        line_votes = list(self.line_buffer)[-3:]
        voted_status, vote_count = Counter(
            line_votes
        ).most_common(1)[0]
        self.line_status = (
            line_votes[-1]
            if vote_count == 1
            else voted_status
        )

        line_vote_details = list(
            getattr(self, 'line_vote_detail_buffer', [])
        )[-3:]
        if not QUIET_LOGS and len(line_vote_details) == 3:
            frame_logs = []
            for detail in line_vote_details:
                angle = detail['decision_angle']
                distance = detail['line_distance']
                curve_a = detail['curve_a']
                angle_text = (
                    f"{angle:+.1f}deg" if math.isfinite(angle) else "N/A"
                )
                distance_text = (
                    f"{distance:+.1f}px"
                    if math.isfinite(distance)
                    else "N/A"
                )
                curve_text = (
                    f"{curve_a:+.2e}"
                    if math.isfinite(curve_a)
                    else "N/A"
                )
                frame_logs.append(
                    f"seq={detail['sequence']} "
                    f"status={detail['status']} "
                    f"type={detail['decision_type']} "
                    f"pc={detail['point_count']} "
                    f"angle={angle_text} "
                    f"distance={distance_text} "
                    f"curve_a={curve_text}"
                )
            self.get_logger().info(
                f"[LineVoteFrames] selected={self.line_status} | "
                + " | ".join(frame_logs)
            )

        if returning_from_pre_shoot:
            self.back_to_walk_after_pre_shoot_cancel = False
            self.pre_shoot_return_to_walk_started = False
            self.angle = self.latest_line_angle
            self.line_follow_point = self.latest_line_follow_point
            self._reset_vision_decision_cycle()
            self.get_logger().info(
                "[LineReturn] pre-shoot cancel Back_To_Walk completed: "
                f"line_status={self.line_status}"
            )
            if self.line_status != Line.Line_None:
                self.LineTracking()
            else:
                # 보행 자세에서 새 Lost 탐색을 시작한다.
                self.lost_step = 0
                self.lost_found_dir = 0
                self.lost_body_turn_count = 0
                self.lost_initial_pose_done = False
                self.lost_back_to_walk_pending = False
                self.LostMode()
            return True

        if self.current_mode == "BallMode":
            ball_votes = list(self.ball_buffer)[-3:]
            ball_vote_counts = Counter(ball_votes)

            # Pick은 정지 상태의 새 샘플 중 2개 이상이
            # Pick_Ready일 때만 확정합니다.
            if ball_vote_counts[Ball.Pick_Ready] >= 2:
                self.ball_status = Ball.Pick_Ready
            else:
                voted_status, vote_count = (
                    ball_vote_counts.most_common(1)[0]
                )

                # 1:1:1 가운데 Pick_Ready가 있으면 Pick 경계에서
                # 흔들린 값일 수 있으므로 명령을 보류합니다. 모션 중
                # 샘플을 비우고 정지 상태의 새 ball_result 3개를
                # 다시 받습니다. Pick_Ready가 없는 1:1:1은 기존처럼
                # 현재 자세를 가장 잘 반영하는 최신값을 선택합니다.
                if vote_count == 1:
                    if Ball.Pick_Ready in ball_votes:
                        self.ball_data = False
                        self.ball_buffer.clear()
                        self.get_logger().info(
                            "[PickVoteDeferred] BallMode 최근 3개가 "
                            f"Pick_Ready를 포함한 1:1:1입니다: "
                            f"{ball_votes}. 최신값을 실행하지 않고 "
                            "ball_buffer를 비운 뒤 정지 상태의 "
                            "새 샘플 3개를 기다립니다."
                        )
                        return False

                    self.ball_status = ball_votes[-1]
                else:
                    self.ball_status = voted_status
        else:
            # BallMode가 아니면 기존 다수결 방식을 유지합니다.
            ball_votes = list(self.ball_buffer)
            self.ball_status = Counter(
                self.ball_buffer
            ).most_common(1)[0][0]
        # ready도 최근 최대 5개 허들 프레임을 다수결 판단 - 3개이상일때만
        ready_count = sum(self.hurdle_ready_buffer)
        self.hurdle_ready = ready_count >= 3

        hurdle_status_candidates = list(self.hurdle_buffer)
        hurdle_search_active = getattr(self, 'hurdle_line_search_step', 0) != 0
        # 허들이 확정되면 hurdle none 상태는 제외 후 판단
        if self.hurdle_detected and not hurdle_search_active:
            hurdle_status_candidates = [
                status
                for status in hurdle_status_candidates
                if status != Hurdle.Hurdle_None
            ]
            if not hurdle_status_candidates:
                self.get_logger().info(
                    "허들모드에 유효한 허들 상태가 없어 새 프레임을 기다립니다."
                )
                return False
        # ready가 false인 동안에는 Forward 20을 상태 투표에서 제외
        if not self.hurdle_ready and not hurdle_search_active:
            hurdle_status_candidates = [
                status
                for status in hurdle_status_candidates
                if status != Motion.Hurdle_Forward_20
            ]
            if not hurdle_status_candidates:
                self.get_logger().info(
                    "hurdle_ready=false이지만 허들 상태가 모두 "
                    "Hurdle_Forward_20이어서 새 프레임을 기다립니다."
                )
                return False

        self.hurdle_status = Counter(
            hurdle_status_candidates
        ).most_common(1)[0][0]

        # 연속값과 나머지 상태 플래그는 콜백에서 저장한 최신값을 사용합니다.
        self.angle = self.latest_line_angle
        self.line_follow_point = self.latest_line_follow_point
        self.ball_angle = self.latest_ball_angle
        self.ball_in_hand = self.latest_ball_in_hand
        self.hurdle_angle = self.latest_hurdle_angle

        self.line_data = True
        self.ball_data = True
        self.hurdle_data = True

        self.get_logger().info(
            "[CachedVision] "
            f"line={self.line_status}, "
            f"ball_votes={ball_votes}, "
            f"ball={self.ball_status}, "
            f"ball_in_hand={self.ball_in_hand}, "
            f"hurdle={self.hurdle_status}, "
            f"hurdle_ready={self.hurdle_ready}"
        )
        self.Decision()
        return True


###### 판단 로직 시작 #######
    def Decision(self):
        if (
            not self.motion_ready
            or not MainDecision._vision_stack_ready(self)
            or not MainDecision._mission_has_started(self)
        ):
            return

        if not (self.line_data == True and self.ball_data == True and self.hurdle_data == True):   
            self.get_logger().info("아직 모든 데이터가 도착하지 않았습니다. 판단 대기중...")
            return
        
        #모든 데이터가 준비된 경우에만 의사결정 로직 실행
        self.get_logger().info("3가지 데이터 모두 도착 완료! 판단을 시작합니다.")

        #우선순위 1 : hurdle mode
        # 허들이 한 번 검출되면 Hurdle_Go가 끝날 때까지 다른 모드로 전환하지 않습니다.
        if self.hurdle_detected:
            self.lost_step = 0
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0
            self.lost_initial_pose_done = False
            self.lost_back_to_walk_pending = False
            self.lost_left_line_seen = False
            self.lost_right_line_seen = False
            self.lost_neck_scan_side = 0

            self.HurdleMode()

        # BallMode 내부에서 Pick 확인, Pick 이후 회전까지 처리
        #우선순위 2 : ball mode
        #Ball mode 활성화 조건
        elif (
            self.pick_done == True
            or self.turn_after_pick == True
            or getattr(self, 'back_to_walk_after_pick', False)
            or self.turn_after_shoot == True
            or getattr(self, 'back_to_walk_after_shoot', False)
            or self._ball_status_is_detected(self.ball_status)
        ):
            self.BallMode()

        #lostmode 진행중이면 계속 lostmode
        elif (
            self.lost_step != 0
            or getattr(self, 'lost_initial_pose_done', False)
            or getattr(self, 'lost_back_to_walk_pending', False)
        ):
            self.LostMode()

        #우선순위 3 : lost mode    
        elif self.line_status == Line.Line_None:    
            self.LostMode()

        #우선순위 4 : line tracking mode
        else:
            self.LineTracking()
                
      
    def CheckBall(self):
        self.pick_done = False
        if self.ball_in_hand == True:
            self.pick_failed = False
            self.has_ball = True
            MainDecision._set_webcam_ball_allowed(
                self,
                False,
                reason='ball possession confirmed',
            )
            MainDecision._set_vision_activity(
                self,
                ball_active=False,
                hoop_active=True,
                reason='ball possession confirmed',
            )
            self.get_logger().info("pick success: ball is in hand")
        else:
            self.has_ball = False
            MainDecision._begin_post_pick_failure_ball_suppression(self)
            self.get_logger().info("pick failed: ball is not in hand")

        return self.has_ball

    def TurnAfterPick(self):
        #회전 시작 첫 호출 시에만 방향을 결정
        if self.turn_count == 0:
            if self.ball_count == 0:
                self.turn_pick = Motion.Right_Turn_Afterpick
            elif self.ball_count == 1:
                self.turn_pick = Motion.Left_Turn_Afterpick
            else:
                self.turn_after_pick = False
                self.backward_after_pick = False
                if getattr(
                    self,
                    'pick_failed',
                    False,
                ):
                    MainDecision._finish_post_pick_failure_recovery(
                        self,
                        'post-pick turn skipped',
                    )
                self.LineTracking()
                return

            self.ball_count += 1

        # 최소 한 번 회전한 뒤, 라인이 보이면 회전 종료
        if self.turn_count > 0 and self.line_status != Line.Line_None:
            self.turn_after_pick = False
            self.backward_after_pick = False
            self.turn_count = 0
            self.pick_try_count = 0
            self.back_to_walk_after_pick = True
            self.status = Motion.Back_To_Walk
            self.MotionCommand()
            return
        
        # 라인이 안 보이면 최대 10번까지만 회전
        if self.turn_count >= 10:
            self.turn_after_pick = False
            self.backward_after_pick = False
            self.back_to_walk_after_pick = False
            self.turn_count = 0
            self.pick_try_count = 0
            if getattr(
                self,
                'pick_failed',
                False,
            ):
                MainDecision._finish_post_pick_failure_recovery(
                    self,
                    'post-pick turn limit reached',
                )
            self.LostMode()
            return

        self.status = self.turn_pick
        self.turn_count += 1
        self.MotionCommand()

    def TurnAfterShoot(self):
        shoot_13_return = (
            getattr(self, 'post_shoot_motion', None) == Motion.Shoot
        )
        #회전 시작 첫 호출 시에만 방향을 결정
        if self.turn_count == 0:
            if self.goal_count == 0:
                self.turn_shoot = Motion.Right_Turn_Afterpick
            elif self.goal_count == 1 or shoot_13_return:
                # 13번은 세 번째 이후에도 마지막 골대의 왼쪽 방향으로 탐색한다.
                self.turn_shoot = Motion.Left_Turn_Afterpick
            else:
                self._finish_turn_after_shoot('post-shoot turn skipped')
                self.LineTracking()
                return

            self.goal_count += 1

        # 13번만 짧은 30/31번 회전을 최소 5회 수행한다.
        # 나머지 Shoot은 최소 4회 회전한다.
        # 그 뒤부터 라인이 보이면 회전을 끝내고 보행 자세로 복귀한다.
        min_turn_count = 5 if shoot_13_return else 4
        if (
            self.turn_count >= min_turn_count
            and self.line_status != Line.Line_None
        ):
            self.turn_after_shoot = False
            self.turn_count = 0
            self.back_to_walk_after_shoot = True
            self._finish_post_shoot_detection_suppression(
                'post-shoot turn completed'
            )
            self.status = Motion.Back_To_Walk
            self.MotionCommand()
            return
        
        # 13번은 라인이 보일 때까지 계속 회전한다. 다른 슛은 기존 10회 제한.
        if not shoot_13_return and self.turn_count >= 10:
            self._finish_turn_after_shoot('post-shoot turn limit reached')
            self.LostMode()
            return

        self.status = self.turn_shoot
        self.turn_count += 1
        self.MotionCommand()

    def _finish_turn_after_shoot(self, reason):
        """강제회전을 끝내고 다음 공의 RealSense 검출을 시작한다."""
        self.turn_after_shoot = False
        self.back_to_walk_after_shoot = False
        self.post_shoot_motion = None
        self.turn_count = 0

        # 골대 또는 Shoot 직후 결과가 이후 판단에 섞이지 않도록 기존
        # 공 집계 상태를 비우고 다음 공은 RealSense부터 탐색한다.
        self.ball_data = False
        self.ball_buffer.clear()
        self._finish_post_shoot_detection_suppression(reason)
        logger = getattr(self, 'get_logger', None)
        if callable(logger):
            logger().info(
                f"[PostShoot] {reason}: RealSense 공 ON, 골대 OFF, "
                "웹캠 공은 120cm까지 OFF"
            )

    def _finish_post_shoot_detection_suppression(self, reason):
        """Shoot 후 회전 종료 시 다음 공의 RealSense 검출을 시작한다."""
        if not getattr(self, 'post_shoot_detection_suppressed', False):
            return False

        self.post_shoot_detection_suppressed = False
        self.ball_data = False
        self.ball_buffer.clear()
        MainDecision._set_vision_activity(
            self,
            ball_active=True,
            hoop_active=False,
            reason=reason,
        )
        MainDecision._set_webcam_ball_allowed(
            self,
            True,
            reason=reason,
        )
        return True

    #Ball mission            
    def BallMode(self):
        self.current_mode = "BallMode"

        # TurnAfterPick 종료 후 실행한 Back_To_Walk가 완료되면,
        # 해당 모션 종료 판단에 사용한 직전 라인 결과로 즉시 복귀한다.
        # 여기서 버퍼를 먼저 비우면 새 3프레임이 올 때까지 로봇이 멈춘다.
        if getattr(self, 'back_to_walk_after_pick', False):
            self.back_to_walk_after_pick = False
            if getattr(
                self,
                'pick_failed',
                False,
            ):
                MainDecision._finish_post_pick_failure_recovery(
                    self,
                    'Back_To_Walk completed',
                )
            else:
                self.current_mode = "LineTrackingMode"
                self.get_logger().info(
                    "[LineReturn] post-pick Back_To_Walk completed: "
                    f"cached line_status={self.line_status} 즉시 실행"
                )
                self.LineTracking()
            return

        # TurnAfterShoot 종료 후 Back_To_Walk가 완료되면 모션 종료 판단에
        # 사용한 직전 라인 결과를 새 프레임 대기 없이 즉시 실행한다.
        if getattr(self, 'back_to_walk_after_shoot', False):
            self.back_to_walk_after_shoot = False
            self._finish_turn_after_shoot(
                'post-shoot Back_To_Walk completed'
            )
            self.current_mode = "LineTrackingMode"
            self._finish_post_shoot_detection_suppression(
                'post-shoot LineTrackingMode entered'
            )
            self.get_logger().info(
                "[LineReturn] post-shoot Back_To_Walk completed: "
                f"cached line_status={self.line_status} 즉시 실행"
            )
            self.LineTracking()
            return

        #Pick 이후 공 확인, 성공했을 때만 Neck Up 실행
        if self.pick_done == True:
            pick_succeeded = self.CheckBall()
            self.turn_after_pick = True
            self.backward_after_pick = True

            if pick_succeeded:
                self.status = Motion.Neck_Up
                self.MotionCommand()
                return

            # Pick 실패 시 Neck Up을 건너뛰고 아래의 후진 단계로 진행

        # Pick 실패 확인 직후 또는 Pick 성공 시 Neck Up 완료 후,
        # 통합 회전 모션 전에 Backward_half를 한 번만 실행
        if self.turn_after_pick == True:
            if self.backward_after_pick:
                self.backward_after_pick = False
                self.status = Motion.Backward_half
                self.MotionCommand()
                return

            self.TurnAfterPick()
            return

        #Shoot 완료 후 Neck Down을 한 번만 실행
        if self.neck_down_pending == True:
            self.neck_down_pending = False
            self.status = Motion.Neck_Down
            self.MotionCommand()
            return

        #Neck Down 완료 후 Shoot 회전루프
        if self.turn_after_shoot == True:
            self.TurnAfterShoot()
            return
        
        ##### 공이 있음, shoot Mode #####
        #goal이 보이고 공을 가지고 있으면 shoot 시도
        if self.has_ball == True:
            #shoot 준비완료
            if self.ball_status in (
                Ball.Shoot, Ball.Shoot_Close, Ball.Shoot_Mid, Ball.Shoot_62,
            ):
                MainDecision._log_ball_action_evidence(
                    self,
                    self.ball_status,
                )
                self.status = self.ball_status
                #shoot 이후 처리
                self.post_shoot_motion = self.status
                self.has_ball = False
                self.neck_down_pending = True
                self.turn_after_shoot = True
                self.back_to_walk_after_shoot = False
                self.turn_count = 0
                self.shoot_in_progress = True
                self.shoot_motion_started = False
                self.post_shoot_detection_suppressed = True
                MainDecision._set_webcam_ball_allowed(
                    self,
                    False,
                    reason='shoot command issued',
                )
                MainDecision._set_vision_activity(
                    self,
                    ball_active=False,
                    hoop_active=False,
                    reason='shoot command issued: suppress until turn ends',
                )
                self.MotionCommand()
                return

            self.status = self.ball_status
            self.MotionCommand()
            return
        
        ##### 공이 없으면 Pick Mode #####
        #공이 없는데 ShootReady이면 무시
        if self.ball_status in (
            Ball.Shoot, Ball.Shoot_Close, Ball.Shoot_Mid, Ball.Shoot_62,
        ):
            self.LineTracking()
            return
        
        #Pick은 한번만 시도 -> 나중에 횟수 변경하기
        if self.pick_try_count >= 1:
            self.LineTracking()
            return
        
        #Pick 준비 완료되면 동작 실행
        if self.ball_status == Ball.Pick_Ready:
            MainDecision._log_ball_action_evidence(
                self,
                self.ball_status,
            )
            self.pick_try_count += 1
            self.backward_after_pick = False
            self.back_to_walk_after_pick = False
            self.status = Motion.Pick
            self.pick_done = True
            self.MotionCommand()
            return
        
        #그 외에는 비전이 준 명령 실행
        else:
            self.status = self.ball_status

        self.MotionCommand()
        
    #Hurdle mission            
    def HurdleMode(self):
        self.current_mode = "HurdleMode"
        if not getattr(self, 'hurdle_ball_suppressed', False):
            self.hurdle_previous_ball_active = getattr(
                self, 'ball_vision_active', True
            )
            self.hurdle_ball_suppressed = True
            MainDecision._set_vision_activity(
                self,
                ball_active=False,
                hoop_active=getattr(self, 'hoop_vision_active', False),
                reason='hurdle mode: suppress ball detection until crossing ends',
            )

        #step 0: Ready 전 접근명령
        if self.hurdle_step == 0:
            if getattr(self, 'hurdle_line_search_step', 0) != 0:
                if MainDecision._advance_hurdle_line_search(self):
                    return
            if not self.hurdle_ready:
                if self.hurdle_status == Motion.Neck_Left:
                    if self.line_status != Line.Line_None:
                        # 이미 정면 라인이 돌아왔다면 늦게 도착한 탐색 요청은
                        # 실행하지 않고 기존 허들 정렬 결과를 기다린다.
                        self._reset_vision_decision_cycle()
                        return
                    MainDecision._advance_hurdle_line_search(self)
                    return
                if self.hurdle_status == Hurdle.Hurdle_None:
                    self.get_logger().info(
                        "HurdleMode에서 status=99는 모션으로 발행하지 않습니다."
                    )
                    self._reset_vision_decision_cycle()
                    return
                self.status = self.hurdle_status
                self.MotionCommand()
                return

            #Ready 후 20번 종종걸음
            self.hurdle_step = 1
            self.status = Motion.Hurdle_Forward_20
            self.MotionCommand()
            return
        
        #step 1: 허들 넘기 실행
        elif self.hurdle_step == 1:
            self.hurdle_step = 0
            self.hurdle_count += 1

            self.status = Motion.Hurdle_Go
            self.hurdle_go_active = True
            self.hurdle_go_started = False
            self.MotionCommand()
            return

    
    def _reset_hurdle_line_search(self):
        self.hurdle_line_search_step = 0
        self.hurdle_line_found_dir = 0
        self.hurdle_line_turn_count = 0
        self.hurdle_left_line_seen = False
        self.hurdle_right_line_seen = False
        self.hurdle_neck_scan_side = 0

    def _advance_hurdle_line_search(self):
        """Keep neck search and body turns inside HurdleMode."""
        step = getattr(self, 'hurdle_line_search_step', 0)
        if step == 0:
            MainDecision._reset_hurdle_line_search(self)
            self.hurdle_line_search_step = 1
            self.hurdle_neck_scan_side = -1
            self.status = Motion.Neck_Left
        elif step == 1:
            if self.hurdle_left_line_seen or self.line_status != Line.Line_None:
                self.hurdle_line_found_dir = -1
                self.hurdle_line_search_step = 3
                self.hurdle_neck_scan_side = 0
                self.status = Motion.Neck_Center
            else:
                self.hurdle_right_line_seen = False
                self.hurdle_line_search_step = 2
                self.hurdle_neck_scan_side = 1
                self.status = Motion.Neck_Right
        elif step == 2:
            self.hurdle_neck_scan_side = 0
            self.status = Motion.Neck_Center
            if self.hurdle_right_line_seen or self.line_status != Line.Line_None:
                self.hurdle_line_found_dir = 1
                self.hurdle_line_search_step = 3
            else:
                self.hurdle_line_search_step = 5
        elif step in (3, 4):
            if (
                step == 4
                and self.hurdle_line_turn_count >= 3
                and self.line_status != Line.Line_None
            ):
                MainDecision._reset_hurdle_line_search(self)
                return False  # 25번 없이 바로 기존 허들 정렬 판단으로 복귀.
            self.hurdle_line_search_step = 4
            self.hurdle_line_turn_count += 1
            self.status = (
                Motion.Left_Turn_Afterpick
                if self.hurdle_line_found_dir == -1
                else Motion.Right_Turn_Afterpick
            )
        elif step == 5:
            MainDecision._reset_hurdle_line_search(self)
            self.status = Motion.Backward_half
        else:
            return False

        self.get_logger().info(
            f"[HurdleLineSearch] step={self.hurdle_line_search_step}, "
            f"direction={self.hurdle_line_found_dir}, "
            f"turns={self.hurdle_line_turn_count}, command={self.status}"
        )
        self.MotionCommand()
        return True

    #Lost             
    def LostMode(self):
        self.current_mode = "LostMode"

        # LostMode에 처음 진입하면 기존 탐색을 시작하기 전에 기본자세로
        # 복귀한다. 이 플래그는 LineTrackingMode 복귀 때 초기화한다.
        if not getattr(self, 'lost_initial_pose_done', False):
            self.lost_left_line_seen = False
            self.lost_right_line_seen = False
            self.lost_neck_scan_side = 0
            self.lost_initial_pose_done = True
            self.status = Motion.Back_To_Initial
            self.MotionCommand()
            return

        # 라인을 발견해 실행한 Back_To_Walk가 끝난 뒤의 최신 판단으로
        # 라인이 유지될 때만 LineTrackingMode 복귀를 완료한다.
        if getattr(self, 'lost_back_to_walk_pending', False):
            self.lost_back_to_walk_pending = False
            if self.line_status != Line.Line_None:
                self.LineTracking()
                return

            # Back_To_Walk 도중 라인이 사라졌다면 이전 몸통 탐색 방향을
            # 이어가지 않는다. Back_To_Walk는 기본자세로 끝나는 모션이
            # 아니므로 Back_To_Initial을 반드시 실행한 뒤 새 Lost 탐색을
            # 목 왼쪽부터 다시 시작한다.
            self.lost_step = 0
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0
            self.lost_initial_pose_done = True
            self.lost_left_line_seen = False
            self.lost_right_line_seen = False
            self.lost_neck_scan_side = 0
            self.status = Motion.Back_To_Initial
            self.MotionCommand()
            return

        #step 0 
        if self.lost_step == 0:
            if self.line_status != Line.Line_None:
                MainDecision._return_from_lost_to_line_tracking(self)
                return
            # 목 왼쪽 회전
            self.lost_left_line_seen = False
            self.lost_right_line_seen = False
            self.lost_neck_scan_side = -1
            self.lost_step = 1
            self.status = Motion.Neck_Left
            self.MotionCommand()
            return
        
        #step 1 : 왼쪽에서 라인 확인 
        if self.lost_step == 1:
            #라인 발견하면 step 3 이동, 목 원점 복귀, 방향 저장
            if (
                getattr(self, 'lost_left_line_seen', False)
                or self.line_status != Line.Line_None
            ):
                self.lost_found_dir = -1
                self.lost_step = 3
                self.lost_neck_scan_side = 0
                self.status = Motion.Neck_Center
                self.MotionCommand()
                return
            #목 오른쪽 회전
            self.lost_right_line_seen = False
            self.lost_neck_scan_side = 1
            self.lost_step = 2
            self.status = Motion.Neck_Right
            self.MotionCommand()
            return

        #step 2 : 오른쪽에서 라인 확인
        if self.lost_step == 2:
            #라인 발견하면 step 3 이동, 목 원점 복귀, 방향 저장
            if (
                getattr(self, 'lost_right_line_seen', False)
                or self.line_status != Line.Line_None
            ):
                self.lost_found_dir = 1
                self.lost_step = 3
                self.lost_neck_scan_side = 0
                self.status = Motion.Neck_Center
                self.MotionCommand()
                return

            # 좌우 모두 라인이 없으면 목 중앙 복귀 후 한 번 후진한다.
            self.lost_step = 5
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0
            self.lost_left_line_seen = False
            self.lost_right_line_seen = False
            self.lost_neck_scan_side = 0

            self.status = Motion.Neck_Center
            self.MotionCommand()
            return

        #step 3 : 몸통 회전 명령
        if self.lost_step == 3:
            # Start the forced turns toward the line found during the neck scan.
            
            #왼쪽 회전 기억
            if self.lost_found_dir == -1:
                self.lost_step = 4
                self.lost_body_turn_count = 1
                self.status = Motion.Left_Turn_Afterpick
                self.MotionCommand()
                return
            
            #오른쪽 회전 기억
            elif self.lost_found_dir == 1:
                self.lost_step = 4
                self.lost_body_turn_count = 1
                self.status = Motion.Right_Turn_Afterpick
                self.MotionCommand()
                return

            else:
                self.lost_step = 0
                self.lost_found_dir = 0
                self.lost_body_turn_count = 0
                self.LostMode()
                return

        #step 4 : 몸통 회전 후 라인 보이는지 판단
        if self.lost_step == 4:
            if (
                self.lost_body_turn_count >= 3
                and self.line_status != Line.Line_None
            ):
                MainDecision._return_from_lost_to_line_tracking(self)
                return
            
            if self.lost_body_turn_count < 8:
                self.lost_body_turn_count += 1

                if self.lost_found_dir == -1:
                    self.status = Motion.Left_Turn_Afterpick

                elif self.lost_found_dir == 1:
                    self.status = Motion.Right_Turn_Afterpick
                else:
                    self.lost_step = 0
                    self.lost_found_dir = 0
                    self.lost_body_turn_count = 0
                    self.LostMode()
                    return

                self.MotionCommand()
                return

            # 회전 한도에 도달하면 후진하지 않고 좌우 목 탐색을 다시 한다.
            self.lost_step = 0
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0
            MainDecision.LostMode(self)
            return

        #step 5 : 좌우 탐색 실패 후 목 중앙 복귀가 끝나면 한 번 후진
        if self.lost_step == 5:
            # Backward_half는 기본자세로 끝나므로 추가 자세 복귀 없이
            # 후진 완료 후 step 0에서 라인을 확인하고 새 목 탐색을 한다.
            self.lost_step = 0
            self.status = Motion.Backward_half
            self.MotionCommand()
            return

    def _return_from_lost_to_line_tracking(self):
        """Restore the walking pose before leaving LostMode."""
        self.lost_neck_scan_side = 0
        self.lost_back_to_walk_pending = True
        self.status = Motion.Back_To_Walk
        self.MotionCommand()


    #Line tracking 
    def LineTracking(self):  
        # BallMode 등의 복귀 경로는 Decision의 Line_None 분기를 거치지
        # 않고 이 함수를 직접 호출하므로 최신 라인 상태를 다시 확인한다.
        if self.line_status == Line.Line_None:
            self.get_logger().info(
                "[LineReturn] line_status=99: LostMode로 전환합니다."
            )
            self.LostMode()
            return

        self.current_mode = "LineTrackingMode"
        self._finish_post_shoot_detection_suppression(
            'post-shoot LineTrackingMode entered'
        )

        # 라인을 찾았으므로 Lost 탐색 상태 초기화
        self.lost_step = 0
        self.lost_found_dir = 0
        self.lost_body_turn_count = 0
        self.lost_initial_pose_done = False
        self.lost_back_to_walk_pending = False
        self.lost_left_line_seen = False
        self.lost_right_line_seen = False
        self.lost_neck_scan_side = 0

        # Publisher에서 확정한 최신 라인 모션을 그대로 실행한다.
        self.status = self.line_status
        self.MotionCommand()

    # 공 없음(99)과 공 놓침(45)이 아닌 공 동작 상태인지 확인
    @staticmethod
    def _ball_status_is_detected(status):
        return status not in (Ball.Ball_None, Ball.Ball_Lost)

    # 다음 판단을 위해 비전 데이터 준비 상태와 저장 버퍼를 초기화
    def _reset_vision_decision_cycle(self):
        self.line_data = False
        self.ball_data = False
        self.hurdle_data = False
        self.line_buffer.clear()
        detail_buffer = getattr(self, 'line_vote_detail_buffer', None)
        if detail_buffer is not None:
            detail_buffer.clear()
        self.ball_buffer.clear()
        if getattr(self, 'ball_start_reset_pending', False):
            self.ball_buffer.extend([Ball.Ball_None] * 3)
        ball_detail_buffer = getattr(
            self, 'ball_vote_detail_buffer', None
        )
        if ball_detail_buffer is not None:
            ball_detail_buffer.clear()
        self.hurdle_buffer.clear()
        self.hurdle_ready_buffer.clear()

    def _begin_post_pick_failure_ball_suppression(self):
        """Keep RealSense on but suppress webcam during failed-pick recovery."""
        self.pick_failed = True
        self.ball_data = False
        self.ball_buffer.clear()
        MainDecision._set_webcam_ball_allowed(
            self,
            False,
            reason='pick failed: suppress webcam during recovery',
        )
        MainDecision._set_vision_activity(
            self,
            ball_active=True,
            hoop_active=False,
            reason='pick failed: keep RealSense ball active',
        )
        self.get_logger().info(
            "[PostPickFailure] RealSense 공은 ON으로 유지하고 웹캠 공만 "
            "OFF로 잠갔습니다."
        )

    def _finish_post_pick_failure_recovery(self, reason):
        """Restore ball processing immediately after recovery ends."""
        if not getattr(
            self,
            'pick_failed',
            False,
        ):
            return False

        # OFF 구간에 발행된 status=99와 과거 웹캠 투표가 새 공 판단에
        # 섞이지 않도록 검출기를 켜기 직전에 모든 판단 버퍼를 비운다.
        MainDecision._reset_vision_decision_cycle(self)
        self.pick_failed = False
        self.current_mode = "LineTrackingMode"
        MainDecision._set_vision_activity(
            self,
            ball_active=True,
            hoop_active=False,
            reason='post-pick failure recovery completed',
        )
        MainDecision._set_webcam_ball_allowed(
            self,
            True,
            reason='post-pick failure recovery completed',
        )
        self.get_logger().info(
            f"[PostPickFailure] {reason}: 공 투표를 초기화하고 "
            "즉시 새 프레임부터 공 검출과 5프레임 투표를 다시 시작합니다."
        )
        return True

    def MotionCommand(self):
        if not MainDecision._mission_has_started(self):
            self.get_logger().info(
                "mission_started=false: Enter 입력 전에는 모션 명령을 보내지 않습니다."
            )
            return
        if getattr(self, 'vision_reset_pending', False):
            self.get_logger().info(
                'vision_reset_pending=true: 새 프레임 수집 전이라 '
                '모션 명령을 보내지 않습니다.'
            )
            return
        if not self.motion_ready:
            self.get_logger().info("motion_ready=false: 모션 명령을 보내지 않습니다.")
            return
        if not MainDecision._vision_stack_ready(self):
            self.get_logger().info(
                "vision_startup=false: webcam/RealSense YOLO 준비 전이라 "
                "모션 명령을 보내지 않습니다."
            )
            return

        motion_msg = MotionCommand()
        
        if self.status == 0:
            motion_msg.command = Motion.Initial_Pose
            
        elif self.status == 1:
            motion_msg.command = Motion.Forward
        
        elif self.status == 2:
            motion_msg.command = Motion.Left_Half_Forward
        
        elif self.status == 3:
            motion_msg.command = Motion.Right_Half_Forward
        
        elif self.status == 4:
            motion_msg.command = Motion.Left_Turn_Half
        
        elif self.status == 5:
            motion_msg.command = Motion.Right_Turn_Half  
        
        elif self.status == 6:
            motion_msg.command = Motion.Left_Turn
        
        elif self.status == 7:
            motion_msg.command = Motion.Right_Turn
        
        elif self.status == 8:
            motion_msg.command = Motion.Forward_half
        
        elif self.status == 9:
            motion_msg.command = Motion.Backward_half
        
        elif self.status == 10:
            motion_msg.command = Motion.Left_Move
        
        elif self.status == 11:
            motion_msg.command = Motion.Right_Move
        
        elif self.status == 12:
            motion_msg.command = Motion.Pick
        
        elif self.status == 13:
            motion_msg.command = Motion.Shoot

        elif self.status == 14:
            motion_msg.command = Motion.Neck_Up

        elif self.status == 15:
            motion_msg.command = Motion.Neck_Left

        elif self.status == 16:
            motion_msg.command = Motion.Neck_Right

        elif self.status == 17:
            motion_msg.command = Motion.Neck_Center

        elif self.status == 18:
            motion_msg.command = Motion.Neck_Down
            
        elif self.status == 19:
            motion_msg.command = Motion.Hurdle_Go
            
        elif self.status == 20:
            motion_msg.command = Motion.Forward_3step
            
        elif self.status == 21:
            motion_msg.command = Motion.Left_Turn_Curve
        
        elif self.status == 22:
            motion_msg.command = Motion.Right_Turn_Curve
            
        elif self.status == 23:
            motion_msg.command = Motion.Left_Turn_Mission_10
        
        elif self.status == 24:
            motion_msg.command = Motion.Right_Turn_Mission_10

        elif self.status == 25:
            motion_msg.command = Motion.Back_To_Walk

        elif self.status == 26:
            motion_msg.command = Motion.Hurdle_Forward_20

        elif self.status == 27:
            motion_msg.command = Motion.Back_To_Initial

        elif self.status == 28:
            motion_msg.command = Motion.Left_Turn_Mission_5

        elif self.status == 29:
            motion_msg.command = Motion.Right_Turn_Mission_5

        elif self.status == 30:
            motion_msg.command = Motion.Left_Turn_Afterpick

        elif self.status == 31:
            motion_msg.command = Motion.Right_Turn_Afterpick

        elif self.status == 32:
            motion_msg.command = Motion.Shoot_Close

        elif self.status == 33:
            motion_msg.command = Motion.Shoot_Forward

        elif self.status == 34:
            motion_msg.command = Motion.Shoot_Mid

        elif self.status == 35:
            motion_msg.command = Motion.Shoot_62

        else:
            # 미지원 상태가 메시지 기본값 0(Initial_Pose)으로 발행되지
            # 않게 한다. 새 판단에 쓸 비전 버퍼와 motion_end는 유지한다.
            self.get_logger().info(
                f"[MotionCommandRejected] unsupported status={self.status}: "
                "모션 명령을 보내지 않습니다."
            )
            return

        if (
            motion_msg.command in (
                Motion.Back_To_Initial,
                Motion.Shoot_Forward,
            )
            and getattr(self, 'has_ball', False)
            and getattr(self, 'current_mode', None) == "BallMode"
        ):
            self.pre_shoot_result_waiting = True
            self.pre_shoot_verified_result = None
            self.get_logger().info(
                f"[PreShoot] {MOTION_NAME[motion_msg.command]} 실행: "
                "MotionEnd 이후 BallStatusPublisher 결과를 기다립니다."
            )
        
        if (
            motion_msg.command != Motion.Initial_Pose
            and getattr(self, 'hurdle_ignore_until', None) is None
        ):
            self.hurdle_ignore_until = time.monotonic() + 3.0
            self.hurdle_ignore_active = True
            self.hurdle_detected = False
            self.hurdle_ready = False
            self.hurdle_go_active = False
            self.hurdle_go_started = False
            self.hurdle_status = Hurdle.Hurdle_None
            self.hurdle_angle = 0.0
            self.latest_hurdle_angle = 0.0
            self.hurdle_buffer.clear()
            self.hurdle_ready_buffer.clear()

        self.motion_pub.publish(motion_msg)
        motion_name = MOTION_NAME.get(motion_msg.command, 'Unknown')
        self.get_logger().info(
            f"[MotionCommand] command={motion_msg.command}, "
            f"motion={motion_name}, mode={self.current_mode}"
        )
        if getattr(self, 'first_motion_after_start_pending', False):
            started_at = getattr(self, 'mission_started_at', None)
            latency = (
                time.monotonic() - started_at
                if started_at is not None
                else 0.0
            )
            self.first_motion_after_start_pending = False
            self.get_logger().info(
                f'[MISSION STARTED] First motion latency={latency:.3f}s, '
                f'command={motion_msg.command} ({motion_name}).'
            )
        
        self._reset_vision_decision_cycle()
        self.motion_end = False
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MainDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
