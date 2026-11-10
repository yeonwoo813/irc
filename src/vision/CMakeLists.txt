cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(msgs REQUIRED)

install(PROGRAMS
  scripts/yolo_detector.py
  scripts/realsense_yolo_detector.py
  scripts/ready_gated_process.py
  scripts/ball_vision_fusion.py
  scripts/realsense_rgb_stabilizer.py
  scripts/hurdle_vision_fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/frame_health.py
  scripts/line_status_publisher.py
  scripts/ball_status_publisher.py
  scripts/hurdle_status_publisher.py
  scripts/motion_completion_gate.py
  scripts/hurdle_detection_block.py
  DESTINATION lib/${PROJECT_NAME}
)

# decision uses the vision-owned controller at MotionEnd without another
# topic/service round trip. Keep the existing script import working as well.
ament_python_install_module(scripts/line_status_publisher.py)
# Reuse the Python path registered above. This ament module helper rejects a
# second top-level module installation into the same destination.
install(FILES scripts/hurdle_detection_block.py
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
