cmake_minimum_required(VERSION 3.8)
project(motion)

# C++ 17 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

# 헤더 경로 설정
include_directories(include)

# 🚀 실행 파일 이름을 'main'으로 변경
add_executable(main
  src/main.cpp
  src/callback.cpp
  src/sdk.cpp
  src/dynamixel.cpp
  src/dynamixel_controller.cpp
)

# 인클루드 디렉토리 타겟 설정
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

# 의존성 연결
ament_target_dependencies(main
  rclcpp
  std_msgs
  msgs
  dynamixel_sdk
)

# Eigen3 타겟 링크
target_link_libraries(main Eigen3::Eigen)

# 설치 설정 (ros2 run motion main 으로 실행 가능하게)
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

# ROS 2 패키지 등록
ament_package()
