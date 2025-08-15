#ifndef OKMR_CONTROLS_VELOCITY_CONTROL_LAYER_HPP
#define OKMR_CONTROLS_VELOCITY_CONTROL_LAYER_HPP

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "okmr_controls/control_layer_base.hpp"

namespace okmr_controls {

class VelocityControlLayer : public ControlLayerBase {
   public:
    VelocityControlLayer ();

   protected:
    void update () override;

   private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_target_pub_;

    geometry_msgs::msg::Twist current_velocity_;
    geometry_msgs::msg::Twist velocity_target_;

    void velocity_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void velocity_target_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

}  // namespace okmr_controls

#endif  // OKMR_CONTROLS_VELOCITY_CONTROL_LAYER_HPP