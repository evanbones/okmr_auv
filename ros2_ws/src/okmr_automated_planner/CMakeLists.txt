cmake_minimum_required(VERSION 3.8)
project(okmr_automated_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY state_machine_configs/
  DESTINATION share/${PROJECT_NAME}/state_machine_configs
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
    ${PROJECT_NAME}/automated_planner.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME automated_planner
)

if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_movement_action_integration.py)
  add_ros_isolated_launch_test(test/test_relative_move_integration.py)
  add_ros_isolated_launch_test(test/test_freeze_integration.py)
endif()

ament_package()
