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
    //but instead of sending GoalPose, we send GoalVelocity
    //GoalVelocity also needs to have a duration attached to it,
    //so that the Velocity target server can automatically set the speed to 0 once done
    //this prevents out of control movements on communication loss
    //and removes the need for higher level controls to 
    //keep track of remaining duration at high frequncies
    //
    //ALSO
    //need to create very similar servers for Acceleration, Thrust, and Throttle.
    //practcally idential except simpler, no need to integrate, only need to track duration and automatically set setpoints to 0 when done
    //need to create corresponding messages called GoalAccel, GoalWrench, and GoalThrottle
    //accel and thrust are in 6dof, linear and angular, 
    //but Throttle has to use message type MotorThrottle as a field (MotorThrottle and duration are 2 main fields)
    //
    //Also add a subscribtion to ControlMode message at /control_mode in all servers (including retroactive addition to relative_pose_target_server.cpp
    //inside control mode sub callback, we need to implement an enabled variable which gets set if the control mode matches the server
    //when we receive a control mode that disables the current server, set all setpoints to 0, 
    //except within the pose server, where we set the goal pose to be equal to the current pose on sub callback
    //so when pose disabled, goal_pose = current pose
    //
    
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityTargetServer>());
  rclcpp::shutdown();
  return 0;
}
