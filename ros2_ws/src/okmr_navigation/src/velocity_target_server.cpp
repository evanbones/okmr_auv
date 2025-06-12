#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "okmr_msgs/msg/movement_command.hpp"
#include "okmr_msgs/srv/distance_from_goal.hpp"
#include "okmr_msgs/msg/sensor_reading.hpp"
#include "okmr_msgs/msg/goal_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "okmr_msgs/srv/status.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class VelocityTargetServer: public rclcpp::Node
{
    //TODO: fill in this file to act similar to relative pose target server
    //but instead of sending GoalPose, we send GoalSpeed
    //GoalSpeed also needs to have a duration attached to it,
    //so that the Velocity target server can automatically set the speed to 0 once done
    //this prevents out of control movements on communication loss
    //and removes the need for higher level controls to 
    //keep track of remaining duration at high frequncies
    
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityTargetServer>());
  rclcpp::shutdown();
  return 0;
}
