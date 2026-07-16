#include "dynamixel.hpp"
#include <iostream>  

// Dxl 클래스의 생성자에서는 다이나믹셀과의 통신을 설정하고 초기화 작업을 수행합니다.
Dxl::Dxl(bool use_virtual)
{
    virtual_mode = use_virtual;

    // 1. 벡터 크기 초기화 
    th_.resize(NUMBER_OF_DYNAMIXELS);
    th_last_.resize(NUMBER_OF_DYNAMIXELS);
    th_dot_est_.resize(NUMBER_OF_DYNAMIXELS);
    ref_th_.resize(NUMBER_OF_DYNAMIXELS);
    ref_th_value_.resize(NUMBER_OF_DYNAMIXELS);

    // 2. 초기값 0으로 세팅 (쓰레기값 방지)
    th_.setZero();
    th_last_.setZero();
    th_dot_est_.setZero();
    ref_th_.setZero();
    
    // 다이나믹셀 모터 내부의 영점 2048을 모터 좌표계의 영점으로 맞추기 위한 수동 오프셋 배열입니다.
    // 모터의 영점이 실제로 2048이 아닐 수 있기 때문에 이 배열을 통해 각 모터의 영점 오프셋을 수동으로 조정할 수 있습니다.
    for(int i=0; i<NUMBER_OF_DYNAMIXELS; i++) zero_manual_offset[i] = 0.0;

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    // Dynamixel SDK의 PortHandler 객체를 생성하여 포트 통신을 관리합니다.
    // DEVICE_NAME은 "/dev/ttyUSB0"로 설정되어 있습니다.
    // getPortHandler 함수는 포트 이름을 입력으로 받아 해당 포트를 관리하는 PortHandler 객체를 반환합니다.

    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    // Dynamixel SDK의 PacketHandler 객체를 생성하여 데이터 패킷 송수신을 처리합니다.
    // PROTOCOL_VERSION은 2.0으로 설정되어 있습니다.
    // getPacketHandler 함수는 프로토콜 버전을 입력으로 받아 해당 버전에 맞는 PacketHandler 객체를 반환합니다.

    // 가상 모드이면 여기서 종료 (하드웨어 접근 안 함)
    if (virtual_mode) {
        std::cout << "[Info] Dxl Class initialized in VIRTUAL MODE" << std::endl;
        return;
    }

// ********************포트 열기 및 통신 속도 설정**********************

    if (!portHandler->openPort())
        std::cerr << "[Error] Failed to open the port!" << std::endl;
    else 
        std::cout << "[Info] Succeeded to open the port!" << std::endl;
        // openPort() 함수는 포트를 열려고 시도하며, 성공하면 true를 반환하고 실패하면 false를 반환합니다.
        // 포트 열기에 실패하면 에러 메시지를 출력하고, 성공하면 성공 메시지를 출력합니다.
        // 포트가 성공적으로 열렸는지 확인하는 것은 매우 중요합니다. 포트가 열리지 않으면 다이나믹셀과 통신할 수 없기 때문입니다.

    if (!portHandler->setBaudRate(BAUDRATE))
        std::cerr << "[Error] Failed to set the baudrate!" << std::endl;
    else 
        std::cout << "[Info] Succeeded to set the baudrate!" << std::endl;
        // setBaudRate() 함수는 통신 속도를 설정하려고 시도하며, 성공하면 true를 반환하고 실패하면 false를 반환합니다.
        // 통신 속도 설정에 실패하면 에러 메시지를 출력하고, 성공하면 성공 메시지를 출력합니다.
        // 통신 속도가 올바르게 설정되어야 다이나믹셀과 안정적으로 통신할 수 있습니다.

    // 🚀 [보완] 포트 버퍼 비우기 (이전 실행의 찌꺼기 제거)
    portHandler->clearPort();

// ********************위치 제어 모드 및 토크 설정 (안전 로직 포함)**********************

    uint8_t dxl_error = 0;
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++) 
    // 모든 다이나믹셀 모터에 대해 반복합니다. 로봇의 전체 모터 수입니다.
    {
        int32_t cur_pos = 0;
        int retry = 0;
        // 성공할 때까지 읽기 시도 (초기 위치 동기화)
        while(packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition, (uint32_t*)&cur_pos, &dxl_error) != COMM_SUCCESS) {
            if(++retry > 5) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // 토크를 켜기 전에 현재 위치를 목표 위치 레지스터에 먼저 써서 튀는 현상을 방지합니다.
        packetHandler->write4ByteTxRx(portHandler, dxl_id[i], DxlReg_GoalPosition, cur_pos, &dxl_error);

        // 제어 모드를 위치 제어 모드로 설정합니다.
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Position_Control_Mode, &dxl_error);

        // 🚀 [보완] Profile Velocity와 Profile Acceleration을 0으로 설정 (내부 가감속 비활성화)
        // 5차 다항식 궤적을 정확히 따르기 위해 모터 자체의 프로파일 기능을 끕니다.
        packetHandler->write4ByteTxRx(portHandler, dxl_id[i], 112, 0, &dxl_error); // Profile Velocity
        packetHandler->write4ByteTxRx(portHandler, dxl_id[i], 108, 0, &dxl_error); // Profile Acceleration

        // 토크 On (모터에 힘 주기)
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 1, &dxl_error);

        // LED On (디버깅용)
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 1, &dxl_error);
    }

// ********************PID 게인 초기화 (P: 850)**********************

    VectorXd PID_Gain(3); // PID 게인을 저장하는 벡터입니다. (P, I, D).
    PID_Gain << 850, 0, 0; // P 게인은 850으로 설정하고, I와 D 게인은 0으로 설정합니다.
    SetPIDGain(PID_Gain); // PID 게인을 설정하는 함수입니다.
}

// Dxl 클래스의 소멸자에서는 프로그램 종료 시 다이나믹셀의 토크를 끄고 포트를 닫는 작업을 수행합니다.
Dxl::~Dxl()
{
    if (virtual_mode) return;
    uint8_t dxl_error = 0;

    // ********************토크 Off (모터에 힘 풀기)**********************
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 0, &dxl_error);
    }

    // ********************LED Off**********************
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 0, &dxl_error);
    }

    portHandler->closePort(); 
    // openPort() 함수를 통해 열었던 포트를 closePort() 함수를 통해 닫습니다. 리소스 관리를 위해 매우 중요합니다. 
}

// ************************************ GETTERS 각도 읽기(rad으로 변환)*****************************************

// **********************현재 각도 읽기*************************
void Dxl::syncReadTheta() 
// 이 함수는 모든 다이나믹셀 모터의 현재 각도를 읽어서 th_ 벡터에 저장합니다.
{
    if (virtual_mode) {
        th_ = ref_th_;
        return;
    }

    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    // GroupSyncRead 클래스를 사용하여 여러 다이나믹셀 모터의 데이터를 동시에 읽어올 수 있도록 도와줍니다.
    // DxlReg_PresentPosition은 현재 위치를 읽어오는 레지스터 주소입니다.

    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    // addParam() 함수를 통해 여러 모터의 데이터를 동시에 읽어올 수 있습니다.

    if (groupSyncRead.txRxPacket() != COMM_SUCCESS) {
        groupSyncRead.clearParam();
        return;
    }

    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) {
        if (groupSyncRead.isAvailable(dxl_id[i], DxlReg_PresentPosition, 4)) {
            position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
            th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
        }
    }
    // 마지막으로 읽어온 데이터를 convertValue2Radian() 함수를 통해 라디안 단위로 변환하여 th_ 벡터에 저장합니다.
    groupSyncRead.clearParam();
}

VectorXd Dxl::GetThetaAct() // 현재 각도 반환
{
    syncReadTheta(); // 현재 각도를 읽어서 th_ 벡터에 저장하는 함수입니다.
    return th_; 
}

// **********************현재 각속도 읽기**********************************
void Dxl::syncReadThetaDot()
{
    if (virtual_mode) return;
    dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, DxlReg_PresentVelocity, 4);
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) groupSyncReadThDot.addParam(dxl_id[i]);
    if(groupSyncReadThDot.txRxPacket() == COMM_SUCCESS) {
        for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) velocity[i] = groupSyncReadThDot.getData(dxl_id[i], DxlReg_PresentVelocity, 4);
    }
    groupSyncReadThDot.clearParam();
}

// ***********************각속도 변환(rad/s)*************************
VectorXd Dxl::GetThetaDot()
{
    VectorXd vel_(NUMBER_OF_DYNAMIXELS);
    if (virtual_mode) { vel_.setZero(); return vel_; }
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) {
        if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; 
        else vel_[i] = velocity[i] * 0.003816667;
    }
    return vel_;
}

//Getter() : About dynamixel packet data
void Dxl::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

//Getter() : 추정계산 (이전 세타값 - 현재 세타값 / 시간) [rad/s]
void Dxl::CalculateEstimatedThetaDot(int dt_us) { th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001); th_last_ = th_; }
VectorXd Dxl::GetThetaDotEstimated() { return th_dot_est_; }
int16_t Dxl::GetPresentMode() { return this->Mode; }

// **************************** SETTERS ******************************** //

//setter() : 각도 setter() [rad]
void Dxl::syncWriteTheta() 
// 이 함수는 ref_th_ 벡터에 저장된 목표 각도를 다이나믹셀 모터에 전달하여 실제로 모터를 구동하는 역할을 합니다.
{
  if (virtual_mode) return;

  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, DxlReg_GoalPosition, 4);
  // GroupSyncWrite 클래스를 사용하여 여러 모터의 데이터를 동시에 써줄 수 있도록 도와줍니다.

  uint8_t parameter[4] = {0}; 
  ref_th_value_ = ref_th_ * RAD_TO_VALUE; 
  // 라디안 단위의 목표 각도를 다이나믹셀의 raw 값으로 변환합니다.

  for (uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++){
    getParam(ref_th_value_[i], parameter);
    gSyncWriteTh.addParam(dxl_id[i], parameter);
  }
  gSyncWriteTh.txPacket(); // 명령 전송
  gSyncWriteTh.clearParam();
}

// SetThetaRef() 함수는 입력으로 받은 theta 벡터를 ref_th_ 벡터에 저장하는 함수입니다.
// PI를 더해주는 이유는 다이나믹셀의 영점이 2048(중간값)으로 설정되어 있기 때문입니다.
void Dxl::SetThetaRef(VectorXd theta) 
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS;i++) ref_th_[i] = theta[i]+PI; 
}

// Setter() : PID gain setter()
void Dxl::SetPIDGain(VectorXd PID_Gain)
{    
    if (virtual_mode) return;
    uint8_t dxl_error = 0;
    uint16_t P_gain = static_cast<uint16_t>(PID_Gain(0));
    uint16_t I_gain = static_cast<uint16_t>(PID_Gain(1));
    uint16_t D_gain = static_cast<uint16_t>(PID_Gain(2));
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++) {
        packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionPGain, P_gain, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionIGain, I_gain, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error);
    }
}

//Setter() : 현재 모드 설정 (위치 제어 모드로 고정)
int16_t Dxl::SetPresentMode(int16_t Mode) { this->Mode = Position_Control_Mode; return Position_Control_Mode; }

// **************************** Function ******************************** //

//Value2Radian (Raw data -> Radian)
float Dxl::convertValue2Radian(int32_t value) { return value / RAD_TO_VALUE; }

//각도(rad), 각속도(rad/s) 읽기 제어 루프
void Dxl::Loop(bool RxTh, bool RxThDot) { if(RxTh) syncReadTheta(); if(RxThDot) syncReadThetaDot(); }

//dxl 초기 세팅 (오프셋 0으로 초기화)
void Dxl::initActuatorValues() { for (int i=0; i<NUMBER_OF_DYNAMIXELS; i++) zero_manual_offset[i] = 0; }

VectorXd Dxl::read_rad() 
// 현재 각도를 직접 읽어서 라디안 단위로 반환하는 함수입니다.
{
    if (virtual_mode) return ref_th_;
    VectorXd rdl_(NUMBER_OF_DYNAMIXELS);
    int32_t present_position = 0;
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++) {
        packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition,(uint32_t*)&present_position);
        rdl_[i] = (present_position - 2048) * (2.0 * M_PI / 4096.0);
    }
    return rdl_;
}

void Dxl::MoveToTargetQuintic(const VectorXd& theta_goal, int steps, int delay_ms) 
{
    VectorXd theta_now = read_rad();
    for (int s = 1; s <= steps; ++s) {
        double t = static_cast<double>(s) / steps;
        double rate = t * t * t * (10.0 - 15.0 * t + 6.0 * t * t); 
        VectorXd theta_interp = theta_now + (theta_goal - theta_now) * rate;
        SetThetaRef(theta_interp); syncWriteTheta();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    SetThetaRef(theta_goal); syncWriteTheta();
}
