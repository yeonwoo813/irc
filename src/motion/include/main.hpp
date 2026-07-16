#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <memory>

#include "msgs/msg/motion_command.hpp"
#include "msgs/msg/motion_end.hpp"
#include "callback.hpp"
#include "dynamixel.hpp"

class MainNode : public rclcpp::Node 
{
private:
    // 모션 선택, 궤적 생성, 목표 관절각 계산
    std::shared_ptr<Callback> callback_;

    // 다이나믹셀 통신
    std::shared_ptr<Dxl> dxl_port_;        
    
    // ROS 2 통신망 객체들
    // command 구독
    rclcpp::Subscription<msgs::msg::MotionCommand>::SharedPtr 
        motion_command_sub_;

    // motion_end와 motion_ready를 하나의 메시지로 publish
    rclcpp::Publisher<msgs::msg::MotionEnd>::SharedPtr
        motion_end_pub_;

    // timer
    rclcpp::TimerBase::SharedPtr 
        timer_;

    //상태변수
    bool initial_pose_done_ = false;
    int current_motion_id_ = -1;

    //내부 함수들
    void StartInitialPose();
    /////////msg에서 command 번호 정수값을 꺼내옴
    void MotionCallback(
        const msgs::msg::MotionCommand::SharedPtr msg);
    //5ms마다 호출되는 제어루프
    void ControlLoop();
    // motion_end와 motion_ready 상태를 함께 publish
    void PublishMotionState(bool motion_end, bool motion_ready);
    //관절각 벡터의 크기와 NaN/Inf 검사
    bool IsValidTheta(
        const Eigen::VectorXd& theta) const;


public:
    //토픽 구독, 타이머 생성하는 main node 생성자
    MainNode();
};
