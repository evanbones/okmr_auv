/*
This is the main ROS2 node that handles checking the voltage value
It subscribes to the voltage topic so the Thrust to PWM converter can switch tables
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "thrust_to_pwm.h"

class voltage_check : public rclcpp::Node {
public:
    voltage_check() : Node("voltage_check") {
        voltage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/voltage", 10,
            std::bind(&voltage_check::voltage_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Voltage Check started.");
    }

private:
    void voltage_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double voltage = msg->data;
        try {
            updateThrustTableIfVoltageChanged(voltage);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading table: %s", e.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr voltage_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<voltage_check>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
