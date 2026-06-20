#include "rclcpp/rclcpp.hpp"                
#include "dynamixel.hpp"   

#include "dynamixel_controller.hpp"          
#include "BRP_Kinematics.hpp"

#include <sstream>
#include <unistd.h> 
#include <atomic> 
#include <chrono>
#include <memory>
#include <cstdio> 

using namespace std::chrono_literals;

class MainNode : public rclcpp::Node
{
public:
  // node이름 설정, 모션실행중 여부 값 포기화
  MainNode() : Node("main_node"), motion_in_progress(false) 

  //ttyUSB0의 latency_timer 설정, 지연시간 최소화
  system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

  //객체초기화
  dxl_ = std::make_shared<Dxl>();   // Dynamixel 모터와 통신하는 객체
 //trajectory_ = std::make_shared<Trajectory>(); //궤적생성 객체 - 동역학해석하면 필요
 //ik_ = std::make_shared<IK_Function>();
 //pick_ = std::make_shared<Pick>();

  dxl_ctrl_ = std::make_shared<Dxl_Controller>(dxl_.get());  //Dynamixel 제어 객체
  callback_ = std::make_shared<Callback>(trajectory_.get(), ik_.get(), dxl_.get(), pick_.get());

  //모터값 0으로 초기화
  VectorXd theta_zero = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  dxl_->MoveToTargetSmoothCos(theta_zero, 150, 10);
  LogDofSnapshot("ZERO_POSE");

  //초기자세 실행
  callback_->Default_Pose();

}