#ifndef MOVE1M_H
#define MOVE1M_H

#include "MovementBaseClass.h"  
#include <cascade_msgs/msg/movement_command.hpp>  
#include <geometry_msgs/msg/pose_stamped.hpp>  

class Move1m : public MovementBaseClass {
public:
    Move1m(const std::string& name, const BT::NodeConfiguration& config);

//static BT::PortsList providedPorts() {
 //       return {};
 //   }


void setCommandData(cascade_msgs::msg::MovementCommand &msg) override;

    BT::NodeStatus tick() override;

  virtual ~Move1m() = default;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<cascade_msgs::msg::MovementCommand>::SharedPtr publisher_;
    rclcpp::Client<cascade_msgs::srv::Status>::SharedPtr client_;

    bool message_sent_;
    int32_t command_; 


};

#endif //MOVE1M_H
