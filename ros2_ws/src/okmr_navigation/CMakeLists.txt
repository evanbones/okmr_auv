cmake_minimum_required(VERSION 3.8)
project(okmr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(okmr_msgs REQUIRED)

add_executable(dead_reckoning
  src/dead_reckoning.cpp)

add_executable(navigator
  src/navigator.cpp)

add_executable(motion_planner
  src/motion_planner.cpp)

add_executable(motor_cortex
  src/motor_cortex.cpp)

ament_target_dependencies(motor_cortex rclcpp std_msgs okmr_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(dead_reckoning rclcpp std_msgs okmr_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(navigator rclcpp std_msgs sensor_msgs okmr_msgs geometry_msgs tf2_geometry_msgs tf2)
ament_target_dependencies(motion_planner rclcpp std_msgs sensor_msgs okmr_msgs geometry_msgs tf2_geometry_msgs tf2)

target_include_directories(motion_planner PUBLIC include)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
    dead_reckoning
    navigator
    motion_planner
    motor_cortex
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
    ${PROJECT_NAME}/navigator_action_server.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME navigator_action_server
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


ament_package()
