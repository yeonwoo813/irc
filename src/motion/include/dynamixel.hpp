// 다이나믹셀 제어 클래스 정의 (위치 제어 전용 다이어트 버전)

#ifndef DYNAMIXEL_HPP 
#define DYNAMIXEL_HPP 

#include <iostream> 
#include <vector> 
#include <cmath> 
#include <thread> 
#include <chrono> 
#include <Eigen/Dense> 
#include "dynamixel_sdk/dynamixel_sdk.h" 

using namespace Eigen; 

// **************************** 다이나믹셀 제어 상수 **************************** //
#define DEVICE_NAME         "/dev/ttyUSB0" // 포트 이름
#define BAUDRATE            2000000        // 통신 속도
#define PROTOCOL_VERSION    2.0            // 프로토콜 버전
#define NUMBER_OF_DYNAMIXELS 22         // G.O.A.T 전체 관절 수

// 제어 모드 상수
#define Position_Control_Mode 3 // 위치 제어 모드

// 다이나믹셀 레지스터 주소 (X-시리즈 기준) 
#define DxlReg_OperatingMode  11 
#define DxlReg_TorqueEnable   64 
#define DxlReg_LED            65 
#define DxlReg_PositionDGain  80 
#define DxlReg_PositionIGain  82 
#define DxlReg_PositionPGain  84 
#define DxlReg_GoalPosition   116 
#define DxlReg_PresentVelocity 128 
#define DxlReg_PresentPosition 132 

// 변환 계수
#define RAD_TO_VALUE          (4096.0 / (2.0 * M_PI)) 

class Dxl // 다이나믹셀 제어 클래스
{
private:
    // SDK 관련 핸들러
    dynamixel::PortHandler *portHandler; 
    dynamixel::PacketHandler *packetHandler; 

    // 하드웨어 정보
    uint8_t dxl_id[NUMBER_OF_DYNAMIXELS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };
    int16_t Mode = Position_Control_Mode; 

    // 데이터 저장용 변수
    int32_t position[NUMBER_OF_DYNAMIXELS];
    int32_t velocity[NUMBER_OF_DYNAMIXELS];
    double zero_manual_offset[NUMBER_OF_DYNAMIXELS];

    const double PI = 3.141592653589793; 
    bool virtual_mode = false; 

public:
    Dxl(bool use_virtual = false); 
    ~Dxl(); 

    VectorXd th_;           
    VectorXd th_last_;      
    VectorXd th_dot_est_;   
    
    VectorXd ref_th_;       
    VectorXd ref_th_value_; 
    
    // **************************** GETTERS ******************************** //
    void syncReadTheta();           
    VectorXd GetThetaAct();         
    void syncReadThetaDot();        
    VectorXd GetThetaDot();         
    void CalculateEstimatedThetaDot(int dt_us); 
    VectorXd GetThetaDotEstimated(); 
    int16_t GetPresentMode();       
    VectorXd read_rad();            

    // **************************** SETTERS ******************************** //
    void syncWriteTheta();          
    void SetThetaRef(VectorXd theta); 
    void SetPIDGain(VectorXd PID_Gain);   
    int16_t SetPresentMode(int16_t Mode); 

    // **************************** FUNCTIONS ****************************** //
    void getParam(int32_t data, uint8_t *param); 
    float convertValue2Radian(int32_t value); 
    void Loop(bool RxTh, bool RxThDot); 
    void initActuatorValues(); 
    void MoveToTargetSmoothCos(const VectorXd& theta_goal, int steps, int delay_ms); 
    void MoveToTargetQuintic(const VectorXd& theta_goal, int steps, int delay_ms); 
};

#endif // DYNAMIXEL_HPP
