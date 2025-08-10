#include <rclcpp/rclcpp.hpp>
#include <okmr_msgs/msg/motor_thrust.hpp>
#include <okmr_msgs/msg/motor_throttle.hpp>
#include <okmr_msgs/msg/battery_voltage.hpp>
#include <lazycsv.hpp>
#include <map>
#include <cmath>
#include <string>
#include <stdexcept>

class ThrustToPwmNode : public rclcpp::Node
{
public:
    ThrustToPwmNode() : Node("thrust_to_pwm")
    {
        // Declare parameter for thrust curve CSV file
        this->declare_parameter<std::string>("thrust_curve_file", "");
        
        // Get the parameter
        std::string csv_file_path = this->get_parameter("thrust_curve_file").as_string();
        if (csv_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "thrust_curve_file parameter not set");
            return;
        }
        
        // Load CSV table
        load_thrust_curve(csv_file_path);
        
        // Create subscribers
        motor_thrust_sub_ = this->create_subscription<okmr_msgs::msg::MotorThrust>(
            "/motor_thrust", 10,
            std::bind(&ThrustToPwmNode::motor_thrust_callback, this, std::placeholders::_1));
            
        voltage_sub_ = this->create_subscription<okmr_msgs::msg::BatteryVoltage>(
            "/voltage", rclcpp::QoS(10).transient_local(),
            std::bind(&ThrustToPwmNode::voltage_callback, this, std::placeholders::_1));
        
        // Create publisher
        motor_throttle_pub_ = this->create_publisher<okmr_msgs::msg::MotorThrottle>(
            "/motor_throttle", 10);
            
        RCLCPP_INFO(this->get_logger(), "ThrustToPwm node initialized");
    }

private:
    // Data structure: voltage -> (thrust -> throttle)
    std::map<float, std::map<float, float>> thrust_curve_map_;
    float current_voltage_ = 12.0f; // Default voltage
    
    rclcpp::Subscription<okmr_msgs::msg::MotorThrust>::SharedPtr motor_thrust_sub_;
    rclcpp::Subscription<okmr_msgs::msg::BatteryVoltage>::SharedPtr voltage_sub_;
    rclcpp::Publisher<okmr_msgs::msg::MotorThrottle>::SharedPtr motor_throttle_pub_;

    void load_thrust_curve(const std::string& file_path)
    {
        try {
            lazycsv::parser<> parser(file_path);
            
            for (const auto row : parser) {
                const auto [voltage_str, throttle_str, thrust_str] = row.cells(0, 1, 2);
                
                float voltage = std::stof(std::string(voltage_str.trimmed()));
                float throttle = std::stof(std::string(throttle_str.trimmed()));
                float thrust = std::stof(std::string(thrust_str.trimmed()));
                
                // Round thrust to 2 decimals for consistent lookup
                thrust = std::round(thrust * 100.0f) / 100.0f;
                
                thrust_curve_map_[voltage][thrust] = throttle;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loaded thrust curve with %zu voltage levels", 
                       thrust_curve_map_.size());
                       
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load thrust curve: %s", e.what());
            throw;
        }
    }
    
    void voltage_callback(const okmr_msgs::msg::BatteryVoltage::SharedPtr msg)
    {
        if (!msg->cell_voltages.empty()) {
            // Calculate total voltage from cell voltages
            float total_voltage = 0.0f;
            for (float cell_voltage : msg->cell_voltages) {
                total_voltage += cell_voltage;
            }
            current_voltage_ = total_voltage;
            RCLCPP_DEBUG(this->get_logger(), "Updated voltage: %.2f V", current_voltage_);
        }
    }
    
    void motor_thrust_callback(const okmr_msgs::msg::MotorThrust::SharedPtr msg)
    {
        auto throttle_msg = std::make_shared<okmr_msgs::msg::MotorThrottle>();
        throttle_msg->header = msg->header;
        
        // Process each thrust value
        for (size_t i = 0; i < msg->thrust.size(); ++i) {
            float thrust = msg->thrust[i];
            float throttle = interpolate_throttle(current_voltage_, thrust);
            throttle_msg->throttle[i] = throttle;
        }
        
        motor_throttle_pub_->publish(*throttle_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published motor throttle commands");
    }
    
    float interpolate_throttle_at_voltage(float voltage, float requested_thrust)
    {
        // Round thrust value to 2 decimals
        float thrust = std::round(requested_thrust * 100.0f) / 100.0f;
        
        auto voltage_it = thrust_curve_map_.find(voltage);
        if (voltage_it == thrust_curve_map_.end()) {
            RCLCPP_WARN(this->get_logger(), "Voltage %.2f not found in thrust curve", voltage);
            return 1500.0f; // Default PWM value
        }
        
        const auto& thrust_map = voltage_it->second;
        
        // Check if thrust value already exists in the map
        auto thrust_it = thrust_map.find(thrust);
        if (thrust_it != thrust_map.end()) {
            return thrust_it->second;
        }
        
        // Linear search to find bracketing thrust values
        std::vector<std::pair<float, float>> thrust_throttle_pairs;
        for (const auto& pair : thrust_map) {
            thrust_throttle_pairs.push_back(pair);
        }
        
        // Sort by thrust value
        std::sort(thrust_throttle_pairs.begin(), thrust_throttle_pairs.end());
        
        // Find interpolation bounds
        for (size_t i = 0; i < thrust_throttle_pairs.size() - 1; ++i) {
            float t1 = thrust_throttle_pairs[i].first;
            float t2 = thrust_throttle_pairs[i + 1].first;
            
            if (thrust >= t1 && thrust <= t2) {
                float p1 = thrust_throttle_pairs[i].second;
                float p2 = thrust_throttle_pairs[i + 1].second;
                float alpha = (thrust - t1) / (t2 - t1);
                return p1 + alpha * (p2 - p1);
            }
        }
        
        // Handle edge cases: thrust outside of range
        if (thrust < thrust_throttle_pairs.front().first) {
            return thrust_throttle_pairs.front().second;
        } else {
            return thrust_throttle_pairs.back().second;
        }
    }
    
    float interpolate_throttle(float voltage, float requested_thrust)
    {
        // Find the two nearest voltages for interpolation
        auto voltage_it = thrust_curve_map_.lower_bound(voltage);
        
        // Handle edge cases: voltage outside of range
        if (voltage_it == thrust_curve_map_.end()) {
            // Voltage is higher than all available data, use highest voltage
            voltage_it = std::prev(thrust_curve_map_.end());
            return interpolate_throttle_at_voltage(voltage_it->first, requested_thrust);
        } else if (voltage_it == thrust_curve_map_.begin()) {
            // Voltage is lower than all available data, use lowest voltage
            return interpolate_throttle_at_voltage(voltage_it->first, requested_thrust);
        } else if (voltage_it->first == voltage) {
            // Exact voltage match
            return interpolate_throttle_at_voltage(voltage, requested_thrust);
        }
        
        // Get the two bracketing voltages
        auto higher_voltage_it = voltage_it;
        auto lower_voltage_it = std::prev(voltage_it);
        
        float v1 = lower_voltage_it->first;
        float v2 = higher_voltage_it->first;
        
        // Calculate throttle at both voltages
        float throttle1 = interpolate_throttle_at_voltage(v1, requested_thrust);
        float throttle2 = interpolate_throttle_at_voltage(v2, requested_thrust);
        
        // Interpolate between the two throttle values based on voltage
        float voltage_alpha = (voltage - v1) / (v2 - v1);
        float interpolated_throttle = throttle1 + voltage_alpha * (throttle2 - throttle1);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Voltage interpolation: %.2fV between %.2fV (%.1f) and %.2fV (%.1f) = %.1f",
                    voltage, v1, throttle1, v2, throttle2, interpolated_throttle);
        
        return interpolated_throttle;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ThrustToPwmNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("thrust_to_pwm"), "Node failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}