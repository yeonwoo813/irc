import rclpy
from rclpy.node import Node
from collections import deque, Counter
from msgs.msg import LineResult, MotionCommand, MotionEnd, BallResult, HurdleResult

# 커스텀메시지 가져오기

class Motion:
    Initial_Pose = 0
    Forward = 1
    Left_Half_Forward = 2
    Right_Half_Forward = 3
    Left_Forward = 4
    Right_Forward = 5
    Left_Turn = 6
    Right_Turn = 7
    Forward_half = 8
    Backward_half = 9
    Pick = 10
    Recatch = 11
    
    
    # 모션 번호 나열하기
    
    
class MainDecision(Node):
    def __init__(self):
        super().__init__('main_decision')

        #초기값 설정
        self.status = 0
        self.motion_end = False
        self.line_data = False
        self.ball_data = False
        self.hurdle_data = False
        
        #최근 5개의 데이터를 저장하는 버퍼
        self.line_buffer = deque(maxlen=5)
        self.ball_buffer = deque(maxlen=5)
        self.hurdle_buffer = deque(maxlen=5)
        
        # subscribe
        self.line_result_sub = self.create_subscription(LineResult, 'line_result', self.LineResultCallback, 10)
        self.ball_result_sub = self.create_subscription(BallResult, 'ball_result', self.BallResultCallback, 10)
        self.hurdle_result_sub = self.create_subscription(HurdleResult, 'hurdle_result', self.HurdleResultCallback, 10)
        self.motion_end_sub = self.create_subscription(MotionEnd, 'motion_end', self.MotionEndCallback, 10)
        #publish
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)
        
    # 콜백함수에서 모션 종료 여부를 업데이트
    def MotionEndCallback(self, motion_end_msg:MotionEnd):
        self.motion_end = motion_end_msg.motion_end
        self.get_logger().info(f"motion_end: {motion_end_msg.motion_end}")
        
        
    def LineResultCallback(self, line_msg:LineResult):
        #최신 데이터 갱신
        self.line_buffer.append(line_msg.status)
        if self.motion_end == True:
            if len (self.line_buffer) >= 3:
                # Counter를 사용해 가장 빈도수가 높은 값 추출
                counts = Counter(self.line_buffer)
                most_common_status = counts.most_common(1)[0][0]
                #다수결 따라 라인 status 결정
                self.line_status = most_common_status
                self.angle = line_msg.angle
                #라인 데이터 ready
                self.line_data = True
                # line result 상태, 각도를 로그로 출력
                self.get_logger().info(f"[LineResult] status: {line_msg.status}, angle: {line_msg.angle}")
                self.Decision()
            else:
                self.get_logger().info(f"not enough data in line buffer")
        else:
            self.get_logger().info(f"line: motion not ended yet")
            
    def BallResultCallback(self, ball_msg:BallResult):
        self.ball_buffer.append(ball_msg.status)
        if self.motion_end == True:
            if len (self.ball_buffer) >= 3:
                counts = Counter(self.ball_buffer)
                most_common_status = counts.most_common(1)[0][0]
                #다수결 따라 ball status 결정
                self.ball_status = most_common_status
                self.ball_angle = ball_msg.angle
                #ball 데이터 ready
                self.ball_data = True
                self.get_logger().info(f"[BallResult] status: {ball_msg.status}, angle: {ball_msg.angle}")
                self.Decision()
            else:
                self.get_logger().info(f"not enough data in ball buffer")
        else:
            self.get_logger().info(f"ball: motion not ended yet")
            
    def HurdleResultCallback(self, hurdle_msg:HurdleResult):
        self.hurdle_buffer.append(hurdle_msg.status)
        if self.motion_end == True:
            if len (self.hurdle_buffer) >= 3:
                counts = Counter(self.hurdle_buffer)
                most_common_status = counts.most_common(1)[0][0]
                #다수결 따라 hurdle status 결정
                self.hurdle_status = most_common_status
                self.hurdle_angle = hurdle_msg.angle
                #hurdle 데이터 ready
                self.hurdle_data = True
                self.get_logger().info(f"[HurdleResult] status: {hurdle_msg.status}, angle: {hurdle_msg.angle}")
                self.Decision()
            else:
                self.get_logger().info(f"not enough data in hurdle buffer")
        else:
            self.get_logger().info(f"hurdle: motion not ended yet")

    def Decision(self):
        if(self.line_data == True and self.ball_data == True and self.hurdle_data == True):   
           
           #모든 데이터가 준비된 경우에만 의사결정 로직 실행
            self.get_logger().info("3가지 데이터 모두 도착 완료! 판단을 시작합니다.")

            #출발 시 무작정 걷는 로직 나중에 만들기
            #우선순위 1 : ball mode
            if self.ball_status == 10:
                self.BallMode()
            #우선순위 2 : hurdle mode
            elif self.hurdle_status == 11:
                self.HurdleMode()
            #우선순위 3 : lost mode    
            elif self.line_status == 8:    
                self.LostMode()
            #우선순위 4 : line tracking mode
            else:
                self.LineTracking()
                
        else: 
            self.get_logger().info("아직 모든 데이터가 도착하지 않았습니다. 판단 대기중...")
        
    #Ball mission            
    def BallMode(self):
        
    #Hurdle mission            
    def HurdleMode(self):
        
    #Lost             
    def LostMode(self):    
        
    #Line tracking 
    def LineTracking(self):    
        self.status = self.line_status
        self.MotionCommand()
                
    def MotionCommand(self):
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
            motion_msg.command = Motion.Pick
        
        elif self.status == 11:
            motion_msg.command = Motion.Recatch
        
        self.motion_pub.publish(motion_msg)
        self.get_logger().info(f"motion command: {motion_msg.command}")
        
        self.line_data = False
        self.ball_data = False
        self.hurdle_data = False
        self.motion_end = False
        self.line_buffer.clear()
        self.ball_buffer.clear()
        self.hurdle_buffer.clear()
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MainDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()