cmake_minimum_required(VERSION 3.8)
project(okmr_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(okmr_msgs REQUIRED)

include_directories(include)

add_library(thrust_to_pwm
  src/thrust_to_pwm.cpp
)
ament_target_dependencies(thrust_to_pwm)

add_executable(voltage_check_node
  src/voltage_check.cpp
)
ament_target_dependencies(voltage_check_node
  rclcpp
  okmr_msgs
)

target_link_libraries(voltage_check_node thrust_to_pwm)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install C++ executables
install(TARGETS
  voltage_check_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  thrust_to_pwm
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY tables/
  DESTINATION share/${PROJECT_NAME}/tables
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp okmr_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
