import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from collections import deque, Counter
from msgs.msg import LineResult, MotionCommand, MotionEnd, BallResult, HurdleResult

# 커스텀메시지 가져오기

class Motion:
    Initial_Pose = 0
    Forward_4step = 1
    Left_Half_Forward = 2  #기본3스텝
    Right_Half_Forward = 3
    Left_Forward = 4 #no
    Right_Forward = 5 #no
    Left_Turn = 6  #line tracking
    Right_Turn = 7 #line tracking
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
    Forward_3step = 20 #ball mode
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

    Data_None = 99
    
    # 모션 번호 나열하기


MOTION_NAME = {
    value: name
    for name, value in vars(Motion).items()
    if not name.startswith('_') and isinstance(value, int)
}

class Ball:
    Ball_None = 99
    Ball_Forward = Motion.Forward_3step
    Ball_Forward_1step = Motion.Forward_half
    Ball_Lost = 45
    Ball_Right = Motion.Right_Move
    Ball_Left = Motion.Left_Move
    Pick_Ready = Motion.Pick
    Shoot = Motion.Shoot
    Ball_In_Hand = 50



class Line:
    Line_None = 99

class Hurdle:
    Hurdle_Detected = Motion.Hurdle_Forward_20
    Hurdle_Go = 19
    Hurdle_None = 99
    Hurdle_1step = 25



    
class MainDecision(Node):
    def __init__(self):
        super().__init__('main_decision')

        #test_mode 파라미터 선언 및 초기화
        self.declare_parameter('test_mode', False)
        self.declare_parameter('ball_lost_timeout_sec', 0.8)
        test_mode_param = self.get_parameter('test_mode').value
        if isinstance(test_mode_param, str):
            self.test_mode = test_mode_param.lower() in ('true', '1', 'yes', 'on')
        else:
            self.test_mode = bool(test_mode_param)
        self.ball_lost_timeout_sec = max(
            0.0,
            float(self.get_parameter('ball_lost_timeout_sec').value),
        )

        #초기값 설정
        self.status = 0
        self.current_mode = "WaitingMode"
        #test mode true/false에 따라 초기값 조정
        self.motion_end = self.test_mode
        self.motion_ready = self.test_mode
        self.line_data = False
        self.ball_data = False
        self.hurdle_data = False
        self.latest_line_angle = 0.0
        self.latest_line_follow_point = False
        self.latest_ball_angle = 0.0
        self.latest_ball_in_hand = False
        self.latest_hurdle_angle = 0.0
        self.latest_hurdle_ready = False

        if self.test_mode:
            self.get_logger().info(
                "test_mode enabled: motion_ready and motion_end will stay true"
            )
        else:
            self.get_logger().info("motion_ready=true 수신 전까지 판단을 대기합니다.")

        #ball
        self.has_ball = False
        self.pick_try_count = 0
        self.pick_done = False
        self.ball_in_hand = False
        # 공 접근을 시작한 뒤의 짧은 미검출은 라인 모드 전환으로 처리하지 않는다.
        # 이 latch는 Pick 명령 전 접근 구간에서만 사용한다.
        self.ball_tracking_active = False
        self.ball_last_seen_time = None
        self.ball_loss_grace_waiting = False
        #pick이후 회전
        self.ball_count = 0
        self.turn_after_pick = False
        self.backward_before_turn_after_pick_pending = False
        self.back_to_walk_after_pick_pending = False
        self.turn_count = 0
        #lost
        self.lost_count = 0
        self.lost_step = 0
        self.lost_found_dir = 0
        self.lost_body_turn_count = 0
        #goal
        self.goal_count = 0
        self.neck_down_pending = False
        self.turn_after_shoot = False
        self.turn_shoot = Motion.Right_Turn

        #hurdle
        self.hurdle_step = 0
        self.hurdle_done = False
        self.hurdle_count = 0
        self.hurdle_ready = False
        self.hurdle_detected = False
        self.hurdle_go_active = False
        self.hurdle_go_started = False

        # 최근 5개의 비전 상태를 저장합니다.
        self.line_buffer = deque(maxlen=5)
        self.ball_buffer = deque(maxlen=5)
        self.hurdle_buffer = deque(maxlen=5)
        self.hurdle_ready_buffer = deque(maxlen=5)
        
        # subscribe
        self.line_result_sub = self.create_subscription(LineResult, 'line_result', self.LineResultCallback, 10)
        self.ball_result_sub = self.create_subscription(BallResult, 'ball_result', self.BallResultCallback, 10)
        self.hurdle_result_sub = self.create_subscription(HurdleResult, 'hurdle_result', self.HurdleResultCallback, 10)
        #motion_ready 명령을 못 받는 상황 방지
        motion_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.motion_end_sub = self.create_subscription(
            MotionEnd, 'motion_end', self.MotionEndCallback, motion_state_qos
        )
        #publish
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)
        # 비전 콜백이 잠시 끊겨도 0.8초 유예 만료를 확인할 수 있게 한다.
        self.ball_loss_grace_timer = self.create_timer(
            0.05,
            self._check_pre_pick_ball_loss_timeout,
        )
        
    # 콜백함수에서 모션 종료 여부를 업데이트
    def MotionEndCallback(self, motion_end_msg:MotionEnd):
        if self.test_mode:
            self.motion_ready = True
            self.motion_end = True
            return
        #메시지 받기 이전 상태 저장
        was_ready = self.motion_ready
        was_motion_end = self.motion_end
        #최신 상태 갱신
        self.motion_ready = motion_end_msg.motion_ready
        self.motion_end = motion_end_msg.motion_end

        if self.hurdle_go_active:
            if not self.motion_end:
                self.hurdle_go_started = True
            elif self.hurdle_go_started:
                self.hurdle_go_active = False
                self.hurdle_go_started = False
                self.hurdle_detected = False
                self.get_logger().info(
                    "Hurdle_Go 완료: hurdle_detected=false, "
                    "허들 모드 잠금을 해제하고 라인 트래킹 복귀를 허용합니다."
                )

        self.get_logger().info(
            f"motion_ready: {self.motion_ready}, motion_end: {self.motion_end}"
        )
        if self.motion_ready and not was_ready:
            self.get_logger().info("초기자세 완료 확인: 판단을 시작합니다.")

        # 모션 실행 중 쌓인 최신 비전 결과를 모션 종료 즉시 집계합니다.
        # 다음 line/ball/hurdle 콜백을 기다리지 않도록 false -> true
        # 전환에서만 판단을 시도합니다.
        if self.motion_end and not was_motion_end:
            self._try_decision_from_cached_results()
        
        
    def LineResultCallback(self, line_msg:LineResult):
        self.latest_line_angle = line_msg.angle
        self.latest_line_follow_point = line_msg.follow_point

        if not self.motion_ready:
            return

        #최신 데이터 갱신
        self.line_buffer.append(line_msg.status)
        if self.motion_end == True:
            self._try_decision_from_cached_results()
        else:
            self.get_logger().info(f"line: motion not ended yet")
            
    def BallResultCallback(self, ball_msg:BallResult):
        self.latest_ball_angle = ball_msg.angle
        self.latest_ball_in_hand = bool(
            getattr(ball_msg, 'ball_in_hand', False)
        )

        if not self.motion_ready:
            return

        # 유예 시간은 raw 프레임의 마지막 실제 검출 시각을 기준으로 잰다.
        # latch 활성화 자체는 다수결 결과로 BallMode에 진입할 때만 한다.
        if (
            self._ball_status_is_detected(ball_msg.status)
            and self._pre_pick_ball_grace_enabled()
        ):
            self.ball_last_seen_time = self._now_seconds()
            if self.ball_loss_grace_waiting:
                self.ball_loss_grace_waiting = False
                self.get_logger().info(
                    f"{self.ball_lost_timeout_sec:.1f}초 유예 중 공 재검출: "
                    "공 모드를 계속 유지합니다."
                )

        self.ball_buffer.append(ball_msg.status)
        
        if self.motion_end == True:
            self._try_decision_from_cached_results()
        else:
            self.get_logger().info(f"ball: motion not ended yet")
            
    def HurdleResultCallback(self, hurdle_msg:HurdleResult):
        self.latest_hurdle_angle = hurdle_msg.angle
        self.latest_hurdle_ready = hurdle_msg.hurdle_ready

        if not self.motion_ready:
            return

        confirmed_hurdle_detected = bool(
            hurdle_msg.hurdle_ready
            or hurdle_msg.status != Hurdle.Hurdle_None
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

        self.hurdle_buffer.append(hurdle_msg.status)
        self.hurdle_ready_buffer.append(bool(hurdle_msg.hurdle_ready))
        if self.motion_end == True:
            self._try_decision_from_cached_results()
        else:
            self.get_logger().info(f"hurdle: motion not ended yet")

    def _try_decision_from_cached_results(self):
        if not self.motion_ready or not self.motion_end:
            return False

        # 이미 현재 종료 사이클의 판단을 시작했다면 중복 명령을 막습니다.
        if self.line_data and self.ball_data and self.hurdle_data:
            return False

        if (
            len(self.line_buffer) < 3
            or len(self.ball_buffer) < 3
            or len(self.hurdle_buffer) < 3
        ):
            self.get_logger().info(
                "저장된 비전 데이터 부족: "
                f"line={len(self.line_buffer)}, "
                f"ball={len(self.ball_buffer)}, "
                f"hurdle={len(self.hurdle_buffer)}"
            )
            return False

        # status는 모션 중 저장된 최근 샘플의 다수결을 사용합니다.
        self.line_status = Counter(
            self.line_buffer
        ).most_common(1)[0][0]

        if self.current_mode == "BallMode":
            ball_votes = list(self.ball_buffer)[-3:]
            voted_status, vote_count = Counter(
                ball_votes
            ).most_common(1)[0]

            # 최근 3개 상태가 모두 다르면 가장 최근 상태를 선택합니다.
            self.ball_status = (
                ball_votes[-1]
                if vote_count == 1
                else voted_status
            )
        else:
            # BallMode가 아니면 기존 다수결 방식을 유지합니다.
            ball_votes = list(self.ball_buffer)
            self.ball_status = Counter(
                self.ball_buffer
            ).most_common(1)[0][0]
        # ready도 최근 최대 5개 허들 프레임을 다수결합니다.
        # 버퍼에 3~5개가 있어도 고정 기준인 true 3개 이상일 때만 ready입니다.
        ready_count = sum(self.hurdle_ready_buffer)
        self.hurdle_ready = ready_count >= 3

        # ready가 false인 동안에는 Forward 20을 상태 투표에서 제외합니다.
        hurdle_status_candidates = list(self.hurdle_buffer)
        # 허들 확정 전에 쌓인 None은 확정 후의 동작 선택에 사용하지 않습니다.
        if self.hurdle_detected:
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

        if not self.hurdle_ready:
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
        if not self.motion_ready:
            return

        if not (self.line_data == True and self.ball_data == True and self.hurdle_data == True):   
            self.get_logger().info("아직 모든 데이터가 도착하지 않았습니다. 판단 대기중...")
            return
        
        #모든 데이터가 준비된 경우에만 의사결정 로직 실행
        self.get_logger().info("3가지 데이터 모두 도착 완료! 판단을 시작합니다.")

        # 허들이 한 번 검출되면 Hurdle_Go가 끝날 때까지 다른 모드로 전환하지 않습니다.
        if self.hurdle_detected:
            self.lost_count = 0
            self.lost_step = 0
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0

            self.HurdleMode()

        # Pick 전 공 접근 중에는 0.8초 미만의 일시 미검출만으로
        # 라인트래킹에 복귀하지 않는다. 모션이 끝난 상태에서 새 비전 샘플을
        # 다시 모아 공 재검출 또는 timeout을 확인한다.
        elif self._hold_pre_pick_ball_mode_for_grace_period():
            return

        # BallMode 내부에서 Pick 확인, Pick 이후 회전까지 처리
        #우선순위 1 : ball mode
        elif (
            self.pick_done == True
            or self.turn_after_pick == True
            or getattr(self, 'back_to_walk_after_pick_pending', False)
            or self.turn_after_shoot == True
            or self._ball_status_is_detected(self.ball_status)
        ):
            if (
                self._ball_status_is_detected(self.ball_status)
                and self._pre_pick_ball_grace_enabled()
            ):
                self._activate_pre_pick_ball_tracking()
            self.ball_loss_grace_waiting = False
            self.BallMode()

        #lostmode 진행중이면 계속 lostmode
        elif self.lost_step != 0:
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
            self.has_ball = True
            self.get_logger().info("pick success: ball is in hand")
        else:
            self.has_ball = False
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
                self.backward_before_turn_after_pick_pending = False
                self.LineTracking()
                return

            self.ball_count += 1

        # 최소 한 번 회전한 뒤, 라인이 보이면 회전 종료
        if self.turn_count > 0 and self.line_status != Line.Line_None:
            self.turn_after_pick = False
            self.backward_before_turn_after_pick_pending = False
            self.turn_count = 0
            self.pick_try_count = 0
            self.back_to_walk_after_pick_pending = True
            self.status = Motion.Back_To_Walk
            self.MotionCommand()
            return
        
        # 라인이 안 보이면 최대 10번까지만 회전
        if self.turn_count >= 10:
            self.turn_after_pick = False
            self.backward_before_turn_after_pick_pending = False
            self.back_to_walk_after_pick_pending = False
            self.turn_count = 0
            self.pick_try_count = 0
            self.LostMode()
            return

        self.status = self.turn_pick
        self.turn_count += 1
        self.MotionCommand()

    def TurnAfterShoot(self):
        #회전 시작 첫 호출 시에만 방향을 결정
        if self.turn_count == 0:
            if self.goal_count == 0:
                self.turn_shoot = Motion.Right_Turn_Afterpick
            elif self.goal_count == 1:
                self.turn_shoot = Motion.Left_Turn_Afterpick
            else:
                self.turn_after_shoot = False
                self.LineTracking()
                return

            self.goal_count += 1

        # 최소 한 번 회전한 뒤, 라인이 보이면 회전 종료
        if self.turn_count > 0 and self.line_status != Line.Line_None:
            self.turn_after_shoot = False
            self.turn_count = 0
            self.LineTracking()
            return
        
        # 라인이 안 보이면 최대 5번까지만 회전
        if self.turn_count >= 5:
            self.turn_after_shoot = False
            self.turn_count = 0
            self.LostMode()
            return

        self.status = self.turn_shoot
        self.turn_count += 1
        self.MotionCommand()

    #Ball mission            
    def BallMode(self):
        self.current_mode = "BallMode"

        # TurnAfterPick 종료 후 실행한 Back_To_Walk가 완료되면,
        # 해당 모션 중 쌓인 비전값을 버리고 새 라인 데이터로 복귀 판단한다.
        if getattr(self, 'back_to_walk_after_pick_pending', False):
            self.back_to_walk_after_pick_pending = False
            self.current_mode = "LineTrackingMode"
            self._reset_vision_decision_cycle()
            return

        #Pick 이후 공 확인, 성공했을 때만 Neck Up 실행
        if self.pick_done == True:
            pick_succeeded = self.CheckBall()
            self.turn_after_pick = True
            self.backward_before_turn_after_pick_pending = True

            if pick_succeeded:
                self.status = Motion.Neck_Up
                self.MotionCommand()
                return

            # Pick 실패 시 Neck Up을 건너뛰고 아래의 후진 단계로 진행

        # Pick 실패 확인 직후 또는 Pick 성공 시 Neck Up 완료 후,
        # 통합 회전 모션 전에 Backward_half를 한 번만 실행
        if self.turn_after_pick == True:
            if self.backward_before_turn_after_pick_pending:
                self.backward_before_turn_after_pick_pending = False
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
            #shoot 직전 공확인
            self.CheckBall()

            #공 없으면 무시하고 라인트래킹
            if self.has_ball == False:
                self.LineTracking()
                return
            
            #shoot 준비완료
            if self.ball_status == Ball.Shoot:
                self.status = Motion.Shoot
                #shoot 이후 처리
                self.has_ball = False
                self.neck_down_pending = True
                self.turn_after_shoot = True
                self.turn_count = 0
                self.MotionCommand()
                return

            else:
                self.status = self.ball_status
                self.MotionCommand()
            return
        
        ##### 공이 없으면 Pick Mode #####
        #공이 없는데 ShootReady이면 무시
        if self.ball_status == Ball.Shoot:
            self.LineTracking()
            return
        
        #Pick은 한번만 시도 -> 나중에 횟수 변경하기
        if self.pick_try_count >= 1:
            self.LineTracking()
            return
        
        #Pick 준비 완료되면 동작 실행
        if self.ball_status == Ball.Pick_Ready:
            self._reset_pre_pick_ball_tracking()
            self.pick_try_count += 1
            self.backward_before_turn_after_pick_pending = False
            self.back_to_walk_after_pick_pending = False
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

        #step 0: Ready 전 접근명령
        if self.hurdle_step == 0:
            if not self.hurdle_ready:
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

    
    #Lost             
    def LostMode(self):
        self.current_mode = "LostMode"

        #step 0 
        if self.lost_step == 0:
            if self.line_status != Line.Line_None:
                self.LineTracking()
                return
            # 목 왼쪽 회전
            self.lost_step = 1
            self.status = Motion.Neck_Left
            self.MotionCommand()
            return
        
        #step 1 : 왼쪽에서 라인 확인 
        if self.lost_step == 1:
            #라인 발견하면 step 3 이동, 목 원점 복귀, 방향 저장
            if self.line_status != Line.Line_None:
                self.lost_found_dir = -1
                self.lost_step = 3
                self.status = Motion.Neck_Center
                self.MotionCommand()
                return
            #목 오른쪽 회전
            self.lost_step = 2
            self.status = Motion.Neck_Right
            self.MotionCommand()
            return

        #step 2 : 오른쪽에서 라인 확인
        if self.lost_step == 2:
            #라인 발견하면 step 3 이동, 목 원점 복귀, 방향 저장
            if self.line_status != Line.Line_None:
                self.lost_found_dir = 1
                self.lost_step = 3
                self.status = Motion.Neck_Center
                self.MotionCommand()
                return

            # 목 중앙 복귀 후 처음부터 탐색
            self.lost_count = 0
            self.lost_step = 0
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0

            self.status = Motion.Neck_Center
            self.MotionCommand()
            return

        #step 3 : 몸통 회전 명령
        if self.lost_step == 3:
            #라인 발견하면 lost mode 종료, line tracking으로 이동
            if self.line_status != Line.Line_None:
                self.LineTracking()
                return
            
            #왼쪽 회전 기억
            if self.lost_found_dir == -1:
                self.lost_step = 4
                self.lost_body_turn_count = 1
                self.status = Motion.Left_Turn
                self.MotionCommand()
                return
            
            #오른쪽 회전 기억
            elif self.lost_found_dir == 1:
                self.lost_step = 4
                self.lost_body_turn_count = 1
                self.status = Motion.Right_Turn
                self.MotionCommand()
                return

            else:
                self.lost_count = 0
                self.lost_step = 0
                self.lost_found_dir = 0
                self.lost_body_turn_count = 0
                self.LostMode()
                return

        #step 4 : 몸통 회전 후 라인 보이는지 판단
        if self.lost_step == 4:
            if self.line_status != Line.Line_None:
                self.LineTracking()
                return
            
            if self.lost_body_turn_count < 5:
                self.lost_body_turn_count += 1

                if self.lost_found_dir == -1:
                    self.status = Motion.Left_Turn

                elif self.lost_found_dir == 1:
                    self.status = Motion.Right_Turn
                else:
                    self.lost_count = 0
                    self.lost_step = 0
                    self.lost_found_dir = 0
                    self.lost_body_turn_count = 0
                    self.LostMode()
                    return

                self.MotionCommand()
                return

            self.lost_step = 0
            self.lost_found_dir = 0
            self.lost_body_turn_count = 0
            self.status = Motion.Forward_3step
            self.MotionCommand()
            return


    #Line tracking 
    def LineTracking(self):  
        self.current_mode = "LineTrackingMode"

        #라인을 찾고 lost count 초기화
        self.lost_count = 0 
        self.lost_step = 0
        self.lost_found_dir = 0
        self.lost_body_turn_count = 0

        #vision에서 받은 명령 그대로 실행
        self.status = self.line_status
        self.MotionCommand()

    def _now_seconds(self):
        return time.monotonic()

    @staticmethod
    def _ball_status_is_detected(status):
        return status not in (Ball.Ball_None, Ball.Ball_Lost)

    def _pre_pick_ball_grace_enabled(self):
        return bool(
            not getattr(self, 'has_ball', False)
            and not getattr(self, 'pick_done', False)
            and not getattr(self, 'turn_after_pick', False)
            and not getattr(self, 'back_to_walk_after_pick_pending', False)
            and not getattr(self, 'turn_after_shoot', False)
            and getattr(self, 'pick_try_count', 0) == 0
        )

    def _activate_pre_pick_ball_tracking(self):
        if not self.ball_tracking_active:
            self.get_logger().info(
                f"BallMode 진입: Pick 전 {self.ball_lost_timeout_sec:.1f}초 "
                "미검출 유예를 활성화합니다."
            )
        self.ball_tracking_active = True
        self.ball_loss_grace_waiting = False
        if self.ball_last_seen_time is None:
            self.ball_last_seen_time = self._now_seconds()

    def _reset_pre_pick_ball_tracking(self):
        self.ball_tracking_active = False
        self.ball_last_seen_time = None
        self.ball_loss_grace_waiting = False

    def _reset_vision_decision_cycle(self):
        self.line_data = False
        self.ball_data = False
        self.hurdle_data = False
        self.line_buffer.clear()
        self.ball_buffer.clear()
        self.hurdle_buffer.clear()
        self.hurdle_ready_buffer.clear()

    def _hold_pre_pick_ball_mode_for_grace_period(self):
        if self._ball_status_is_detected(self.ball_status):
            self.ball_loss_grace_waiting = False
            return False

        if not self._pre_pick_ball_grace_enabled():
            self._reset_pre_pick_ball_tracking()
            return False

        if not self.ball_tracking_active or self.ball_last_seen_time is None:
            return False

        elapsed = max(0.0, self._now_seconds() - self.ball_last_seen_time)
        timeout = getattr(self, 'ball_lost_timeout_sec', 0.8)
        if elapsed >= timeout:
            self.get_logger().info(
                f"공 미검출 {elapsed:.2f}초: 공 모드를 해제하고 "
                "라인 판단을 허용합니다."
            )
            self._reset_pre_pick_ball_tracking()
            return False

        self.current_mode = "BallMode"
        if not self.ball_loss_grace_waiting:
            self.get_logger().info(
                f"공 미검출 {elapsed:.2f}초/{timeout:.2f}초: "
                "Pick 전 공 모드를 유지하고 재검출을 기다립니다."
            )
        self.ball_loss_grace_waiting = True
        self._reset_vision_decision_cycle()
        return True

    def _check_pre_pick_ball_loss_timeout(self):
        """비전 콜백 유무와 관계없이 유예 만료를 정확히 처리한다."""
        if not self.ball_tracking_active:
            return

        if not self._pre_pick_ball_grace_enabled():
            self._reset_pre_pick_ball_tracking()
            return

        if self.ball_last_seen_time is None:
            return

        timeout = getattr(self, 'ball_lost_timeout_sec', 0.8)
        elapsed = max(0.0, self._now_seconds() - self.ball_last_seen_time)
        if elapsed < timeout:
            return

        was_waiting = self.ball_loss_grace_waiting
        self._reset_pre_pick_ball_tracking()
        self.get_logger().info(
            f"공 미검출 {elapsed:.2f}초: 공 모드를 해제합니다."
        )

        # 유예 중 모션이 정지해 있었다면, 이미 모인 최신 비전값으로 즉시
        # line/lost/hurdle 모드를 다시 판단한다.
        if (
            was_waiting
            and self.motion_ready
            and self.motion_end
        ):
            self.line_data = False
            self.ball_data = False
            self.hurdle_data = False
            self._try_decision_from_cached_results()
                
    def MotionCommand(self):
        if not self.motion_ready:
            self.get_logger().info("motion_ready=false: 모션 명령을 보내지 않습니다.")
            return

        motion_msg = MotionCommand()
        
        if self.status == 0:
            motion_msg.command = Motion.Initial_Pose
            
        elif self.status == 1:
            motion_msg.command = Motion.Forward_4step
        
        elif self.status == 2:
            motion_msg.command = Motion.Left_Half_Forward
        
        elif self.status == 3:
            motion_msg.command = Motion.Right_Half_Forward
        
        elif self.status == 4:
            motion_msg.command = Motion.Left_Forward
        
        elif self.status == 5:
            motion_msg.command = Motion.Right_Forward   
        
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
        
        self.motion_pub.publish(motion_msg)
        motion_name = MOTION_NAME.get(motion_msg.command, 'Unknown')
        self.get_logger().info(
            f"[MotionCommand] command={motion_msg.command}, "
            f"motion={motion_name}, mode={self.current_mode}"
        )
        
        self._reset_vision_decision_cycle()
        #test mode일 때는 true 로 유지
        self.motion_end = True if self.test_mode else False
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MainDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
