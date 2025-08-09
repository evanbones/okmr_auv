#ifndef OKMR_CONTROLS_POSE_CONTROL_LAYER_HPP
#define OKMR_CONTROLS_POSE_CONTROL_LAYER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <okmr_msgs/msg/relative_pose.hpp>

#include "okmr_controls/control_layer_base.hpp"

namespace okmr_controls {

class PoseControlLayer : public ControlLayerBase {
   public:
    PoseControlLayer ();

   protected:
    void update () override;

   private:
    rclcpp::Subscription<okmr_msgs::msg::RelativePose>::SharedPtr pose_target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_target_pub_;

    okmr_msgs::msg::RelativePose current_pose_;
    okmr_msgs::msg::RelativePose pose_target_;

    void pose_target_callback (const okmr_msgs::msg::RelativePose::SharedPtr msg);
};

}  // namespace okmr_controls

#endif  // OKMR_CONTROLS_POSE_CONTROL_LAYER_HPP