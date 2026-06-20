cmake_minimum_required(VERSION 3.8)
project(motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 이 줄을 추가하여 ROS2 패키지 기능을 사용. 
# 메시지 타입과 서비스가 필요하면 추가
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

#빌드 타겟 설정
add_executable(main_motion src/main_motion.cpp)

#타겟에 대한 포함 디렉토리
target_include_directories(main_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main_motion PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS main_motion
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
