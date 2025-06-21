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
#include "okmr_msgs/srv/status.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RelativePoseTargetServer: public rclcpp::Node
{

    public:
       RelativePoseTargetServer() : Node("relative_pose_target_server"){ 

            goal_pose_subscription = this->create_subscription<okmr_msgs::msg::GoalPose>
                ("/current_goal_pose", 10, 
                 std::bind(&RelativePoseTargetServer::goal_pose_callback, this, _1));

            current_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>
                ("/pose", 10, 
                 std::bind(&RelativePoseTargetServer::current_pose_callback, this, _1));
            
            distance_from_goal_service = this->create_service<okmr_msgs::srv::DistanceFromGoal>
                ("distance_from_pose_goal", 
                 std::bind(&RelativePoseTargetServer::distance_from_goal_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
            //placeholders are used in place of request and response arguments

            //adds all publishers to a map
            pidPublisherMap.insert(std::pair{"yaw_angle", 
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/yaw_angle/target", 10)});

            pidPublisherMap.insert(std::pair{"pitch_angle", 
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/pitch_angle/target", 10)});

            pidPublisherMap.insert(std::pair{"roll_angle", 
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/roll_angle/target", 10)});

            pidPublisherMap.insert(std::pair{"zero_reference",
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/zero_reference/target", 10)});

            pidPublisherMap.insert(std::pair{"x_translation", 
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/x_translation/actual", 10)});

            pidPublisherMap.insert(std::pair{"y_translation", 
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/y_translation/actual", 10)});

            pidPublisherMap.insert(std::pair{"z_translation", 
                    this->create_publisher<okmr_msgs::msg::SensorReading>
                    ("/PID/z_translation/actual", 10)});

            timer = this->create_wall_timer(
                10ms, std::bind(&RelativePoseTargetServer::updateSetPoints, this));
            //TODO make this period a parameter

        }
    private:
        void goal_pose_callback(okmr_msgs::msg::GoalPose msg){
            current_goal_pose_msg=msg;
            hold_mode=false;
            current_goal_eulers = euler_from_quaternion(msg.pose.orientation);
        }

        void current_pose_callback(geometry_msgs::msg::PoseStamped msg){
            current_pose_msg=msg;
            //TODO, can be turned into a lambda
        }

        void distance_from_goal_callback(
                const std::shared_ptr<okmr_msgs::srv::DistanceFromGoal::Request> request,
                std::shared_ptr<okmr_msgs::srv::DistanceFromGoal::Response>      response)
        {
            (void)request;

            response -> translation_differences = 
                calculate_relative_translation(
                        current_pose_msg.pose, 
                        current_goal_pose_msg.pose);

            if(current_goal_pose_msg.copy_orientation){
                response -> orientation_differences = 
                    calculate_euler_differences(
                            current_pose_msg.pose, 
                            current_goal_pose_msg.pose);
            }
            else{
                response -> orientation_differences = geometry_msgs::msg::Vector3();
                //if not copying the orientation, we are always meeting the orientation goal
            }
        }

        geometry_msgs::msg::Vector3 euler_from_quaternion(geometry_msgs::msg::Quaternion q) {

            tf2::Quaternion quat;
            tf2::fromMsg(q, quat);  // Convert from geometry_msgs to tf2::Quaternion

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); 
            //convert to a rotation matrix, then rpy from matrix
            //quaternion -> matrix -> rpy

            geometry_msgs::msg::Vector3 eulers;
            eulers.x = roll;
            eulers.y = pitch;
            eulers.z = yaw;
            return eulers;
        }

        geometry_msgs::msg::Vector3 calculate_euler_differences(
                const geometry_msgs::msg::Pose& current_pose,
                const geometry_msgs::msg::Pose& goal_pose)
        {
            auto current_eulers = euler_from_quaternion(current_pose.orientation);
            auto goal_eulers = euler_from_quaternion(goal_pose.orientation);
            
            auto result = geometry_msgs::msg::Vector3();

            result.x = goal_eulers.x - current_eulers.x;
            result.y = goal_eulers.y - current_eulers.y;
            result.z = goal_eulers.z - current_eulers.z;

            return result;
        }

        geometry_msgs::msg::Vector3 calculate_relative_translation(
                const geometry_msgs::msg::Pose& current_pose,
                const geometry_msgs::msg::Pose& goal_pose)
        {
            // Convert poses to tf2::Transform
            tf2::Transform tf_current, tf_goal;
            tf2::fromMsg(current_pose, tf_current);
            tf2::fromMsg(goal_pose, tf_goal);
        
            // Calculate the inverse of the current pose
            tf2::Transform tf_current_inv = tf_current.inverse();
        
            // Transform the goal pose from the current frame to the robot's frame
            tf2::Transform tf_goal_relative = tf_current_inv * tf_goal;
        
            // Calculate the relative translation
            tf2::Vector3 translation = tf_goal_relative.getOrigin();
        
            // Convert the result back to geometry_msgs::msg::Vector3
            geometry_msgs::msg::Vector3 relative_translation;
            relative_translation.x = translation.x();
            relative_translation.y = translation.y();
            relative_translation.z = translation.z();
        
            return relative_translation;
        }
        
        void updateSetPoints(){
            //This timer callback gets called on a regular interval 
            //defined by update Frequency parameter. 
            //It calculates relative difference between current pose and goal pose, 
            //then publishes neccessary PID setpoints to move towards the goal pose
            //
            //difference is called "relative" because the distances are relative to the current pose
            //rough example: 
            // if we are facing the goal head on, we are 1 meter away
            // but if we are facing 180 degrees from the goal, 
            // we are -1 meters away, since its behind us

            //used to store the goal values to be published at end of callback
            float yaw=0,
                  pitch=0,
                  roll=0;
            
            geometry_msgs::msg::Vector3 relative_translation = 
                calculate_relative_translation(current_pose_msg.pose, current_goal_pose_msg.pose);

            float trig_dist = sqrt(
                    relative_translation.x*relative_translation.x + 
                    relative_translation.y*relative_translation.y + 
                    relative_translation.z*relative_translation.z);
            
            //using xy trig distance to ignore depth / z distance
            //TODO: discuss if we should use xyz or just xy to determine radius of hold zone
            float xy_trig_dist = sqrt(
                    relative_translation.x*relative_translation.x + 
                    relative_translation.y*relative_translation.y);

            if(xy_trig_dist<1){//TODO make holding radius a parameter
                //This if statement results in 2 Different Behaviors, 
                //depending on if copy_orientation is true inside the currentGoalPose
                //
                //When we are inside the holding radius, the goal orientation will be set to:
                //
                //copy_orientation true: the orientation of the goal pose
                //
                //copy_orientation false: the yaw at the time of entering the holding radius 
                //
                //NOTE: roll & pitch are 0 in hold_mode by default, 
                //they can only be set by using copy_orientation
                //
                if(!hold_mode){
                    //this is only called at the moment of entering the holding radius
                    hold_mode = true;
                    hold_yaw = euler_from_quaternion(current_pose_msg.pose.orientation).z;
                }
                if(current_goal_pose_msg.copy_orientation){
                    hold_yaw = current_goal_eulers.z;
                    pitch = current_goal_eulers.y; 
                    roll = current_goal_eulers.x;
                }
                yaw = hold_yaw;
            }
            else{
                //if not inside the holding radius, straight line to the holding radius
                //only triggered during large movement ( distance > holding radius )
                yaw = atan2(
                        current_goal_pose_msg.pose.position.y-current_pose_msg.pose.position.y,
                        current_goal_pose_msg.pose.position.x-current_pose_msg.pose.position.x)
                        *(180/3.1415926);

                //if(trig_dist>1.5)
                //   hold_mode=false;
                // this used to be useful in simulation during early 2024, 
                // but is not needed in real life, since our pid gains are pretty good
                // only useful if the auv enters the holding radius but is unable to stop in time 
                // maybe it will be useful in the future?
                // essentially this is a hack if pid gain values are very very bad
            }

            okmr_msgs::msg::SensorReading 
                pitchMsg = okmr_msgs::msg::SensorReading(),
                yawMsg = okmr_msgs::msg::SensorReading(),
                rollMsg = okmr_msgs::msg::SensorReading(),
                xMsg = okmr_msgs::msg::SensorReading(),
                yMsg = okmr_msgs::msg::SensorReading(),
                zMsg = okmr_msgs::msg::SensorReading(),
                zeroMsg = okmr_msgs::msg::SensorReading();

            pitchMsg.data=pitch;
            yawMsg.data=yaw;
            rollMsg.data=roll;

            //translation is negated for pid controller consistency
            //
            //ex. if we want to move forward towards the goal (move in positive x)
            //we need the error (target (0) - actual (-1)) = 1
            //since a relative_translation.x of 1 would mean the goal is 1 meter ahead of the current pose
            //we negate so that after this subtraction, the error is positive
            xMsg.data=-relative_translation.x;
            yMsg.data=-relative_translation.y;
            zMsg.data=-relative_translation.z;

            pitchMsg.header.stamp=this->now();
            yawMsg.header.stamp=this->now();
            rollMsg.header.stamp=this->now();
            xMsg.header.stamp=this->now();
            yMsg.header.stamp=this->now();
            zMsg.header.stamp=this->now();
            zeroMsg.header.stamp=this->now();

            pidPublisherMap["pitch_angle"]->publish(pitchMsg);
            pidPublisherMap["yaw_angle"]->publish(yawMsg);
            pidPublisherMap["roll_angle"]->publish(rollMsg);

            pidPublisherMap["x_translation"]->publish(xMsg);
            pidPublisherMap["y_translation"]->publish(yMsg);
            pidPublisherMap["z_translation"]->publish(zMsg);

            pidPublisherMap["zero_reference"]->publish(zeroMsg);
            //zero reference is published for position PID controllers
            //since we want the xyz_translation from the goal pose to be zero
            //zero translation from goal pose = at the goal pose

            //these publishers define the pid setpoints for 
            //translation and angles (xyzrpy), not rate (uvwpqr)
            //
            //speed setpoint generation is handled by either 
            //a) cascading pid control (intended when using this node)
            //or
            //b) speed setpoint server (indended for spinning and test commands)
        }
        
        //Member Variables
        
        float hold_yaw=0;
        bool hold_mode=false;

        //cached poses, goal and actual
        okmr_msgs::msg::GoalPose current_goal_pose_msg; 
        geometry_msgs::msg::PoseStamped current_pose_msg;
        geometry_msgs::msg::Vector3 current_goal_eulers; 
        //used to cache the result of quaternion_to_euler on goal pose callback
        
        //ROS2 system interaction components
        rclcpp::Subscription<okmr_msgs::msg::GoalPose>::SharedPtr  goal_pose_subscription;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscription;
        
        rclcpp::Service<okmr_msgs::srv::DistanceFromGoal>::SharedPtr distance_from_goal_service;
        
        rclcpp::TimerBase::SharedPtr timer;
        
        std::map<std::string, rclcpp::Publisher<okmr_msgs::msg::SensorReading>::SharedPtr> pidPublisherMap;
        
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativePoseTargetServer>());
    rclcpp::shutdown();
    return 0;
}
