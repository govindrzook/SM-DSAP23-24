cmake_minimum_required(VERSION 3.8)
project(digital_architecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(imu src/IMU.cpp)
ament_target_dependencies(imu rclcpp std_msgs)

add_executable(logger src/Logger.cpp)
ament_target_dependencies(logger rclcpp std_msgs)

add_executable(central_controller src/CentralController.cpp)
ament_target_dependencies(central_controller rclcpp std_msgs)

add_executable(front_right_servo_controller src/FrontRightServoController.cpp)
ament_target_dependencies(front_right_servo_controller rclcpp std_msgs)

add_executable(front_left_servo_controller src/FrontLeftServoController.cpp)
ament_target_dependencies(front_left_servo_controller rclcpp std_msgs)

add_executable(back_left_servo_controller src/FrontLeftServoController.cpp)
ament_target_dependencies(back_left_servo_controller rclcpp std_msgs)

add_executable(back_right_servo_controller src/BackRightServoController.cpp)
ament_target_dependencies(back_right_servo_controller rclcpp std_msgs)

add_executable(front_right_solo src/FrontRightSOLO.cpp src/hardware_libraries/SOLOUno.cpp)
ament_target_dependencies(front_right_solo rclcpp std_msgs)

add_executable(front_left_solo src/FrontLeftSOLO.cpp src/hardware_libraries/SOLOUno.cpp) 
ament_target_dependencies(front_left_solo rclcpp std_msgs)

add_executable(back_right_solo src/BackRightSOLO.cpp src/hardware_libraries/SOLOUno.cpp) 
ament_target_dependencies(back_right_solo rclcpp std_msgs)

add_executable(back_left_solo src/BackLeftSOLO.cpp src/hardware_libraries/SOLOUno.cpp) 
ament_target_dependencies(back_left_solo rclcpp std_msgs)

target_link_libraries(front_right_solo
  ${catkin_LIBRARIES} serial
)

target_link_libraries(front_left_solo
  ${catkin_LIBRARIES} serial
)

target_link_libraries(back_right_solo
  ${catkin_LIBRARIES} serial
)

target_link_libraries(back_left_solo
  ${catkin_LIBRARIES} serial
)

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
install(TARGETS
  imu
  central_controller
  front_right_servo_controller
  front_left_servo_controller
  back_right_servo_controller
  back_left_servo_controller
  front_right_solo
  front_left_solo
  back_right_solo
  back_left_solo
  logger
  DESTINATION lib/${PROJECT_NAME})
ament_package()