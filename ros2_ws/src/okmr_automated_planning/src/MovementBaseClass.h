#ifndef MOVEMENTBASECLASS_H
#define MOVEMENTBASECLASS_H

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <okmr_msgs/srv/status.hpp>
#include <okmr_msgs/msg/movement_command.hpp>



class MovementBaseClass : public BT::SyncActionNode 

{
public:
  MovementBaseClass(const std::string& name, const BT::NodeConfiguration& config);

static BT::PortsList providedPorts() {
        return {};
    }




  virtual void setCommandData(okmr_msgs::msg::MovementCommand &msg) = 0;

  virtual BT::NodeStatus tick() override;

  virtual ~MovementBaseClass() = default;




private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<okmr_msgs::msg::MovementCommand>::SharedPtr publisher_;
    rclcpp::Client<okmr_msgs::srv::Status>::SharedPtr client_;

    bool message_sent_;
    int32_t command_; 

};
#endif 

// MOVEMENTBASECLASS_H
