cmake_minimum_required(VERSION 3.8)
project(okmr_stonefish)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(stonefish_ros2 REQUIRED)
#find_package(rclcpp REQUIRED)
#probably dont need c++ code here, everything can be in python

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
    ${PROJECT_NAME}/sim_adaptor.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME sim_adaptor
)

ament_package()
