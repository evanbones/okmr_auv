cmake_minimum_required(VERSION 3.8)
project(okmr_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(okmr_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

add_executable(sim_test src/sim_test.cpp)
ament_target_dependencies(sim_test rclcpp sensor_msgs)

add_executable(depth_to_pointcloud src/depth_to_pointcloud.cpp)
ament_target_dependencies(depth_to_pointcloud 
  rclcpp 
  sensor_msgs 
  okmr_msgs 
  cv_bridge 
  OpenCV 
  pcl_conversions
  message_filters
)
target_link_libraries(depth_to_pointcloud ${PCL_LIBRARIES})
target_include_directories(depth_to_pointcloud PRIVATE ${PCL_INCLUDE_DIRS})

add_executable(cloud_visualizer src/cloud_visualizer.cpp)
ament_target_dependencies(cloud_visualizer 
  rclcpp 
  sensor_msgs 
  pcl_conversions
)
target_link_libraries(cloud_visualizer ${PCL_LIBRARIES})
target_include_directories(cloud_visualizer PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS
  sim_test
  depth_to_pointcloud
  cloud_visualizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
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
