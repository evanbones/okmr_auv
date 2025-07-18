/*
This is the main ROS2 node that handles checking the voltage value
It subscribes to the voltage topic so the Thrust to PWM converter can switch tables
It will round to the nearest voltage interval which may cause minor inaccuracies in thrust
*/

#include "rclcpp/rclcpp.hpp"
#include "okmr_msgs/msg/motor_throttle.hpp"
#include "thrust_to_pwm.h"
#include <vector>
#include <cmath>
#include <limits>

class VoltageCheck : public rclcpp::Node {
public:
    VoltageCheck() : Node("voltage_check") {
        voltage_sub_ = this->create_subscription<okmr_msgs::msg::MotorThrottle>(
            "/voltage", 10,
            std::bind(&VoltageCheck::voltage_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Voltage Check started.");
    }

private:
    int roundToNearestAllowedVoltage(double voltage) {
        const std::vector<int> allowed = {10, 12, 14, 16, 18, 20};
        int nearest = allowed[0];
        double min_diff = std::abs(voltage - nearest);

        for (int v : allowed) {
            double diff = std::abs(voltage - v);
            if (diff < min_diff) {
                nearest = v;
                min_diff = diff;
            }
        }

        return nearest;
    }

    void voltage_callback(const okmr_msgs::msg::MotorThrottle::SharedPtr msg) {
        double voltage = msg->data;
        int rounded_voltage = roundToNearestAllowedVoltage(voltage);
        try {
            updateThrustTableIfVoltageChanged(static_cast<double>(rounded_voltage));
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading table: %s", e.what());
        }
    }

    rclcpp::Subscription<okmr_msgs::msg::MotorThrottle>::SharedPtr voltage_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoltageCheck>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
