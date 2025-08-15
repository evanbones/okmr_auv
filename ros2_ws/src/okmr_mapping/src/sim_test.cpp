#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>

class DepthImageAnalyzer : public rclcpp::Node {
   public:
    DepthImageAnalyzer () : Node ("depth_image_analyzer") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image> (
            "/labeled_image", 10,
            std::bind (&DepthImageAnalyzer::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO (this->get_logger (), "Subscribed to /camera2/camera2/image_depth");
    }

   private:
    void image_callback (const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO (this->get_logger (), "Received depth image: %dx%d, encoding: %s", msg->width,
                     msg->height, msg->encoding.c_str ());

        std::set<float> unique_depths;

        if (msg->encoding == "32FC1") {
            const float* data = reinterpret_cast<const float*> (msg->data.data ());
            size_t num_pixels = msg->width * msg->height;

            for (size_t i = 0; i < num_pixels; ++i) {
                float depth = data[i];
                if (!std::isnan (depth) && !std::isinf (depth)) {
                    unique_depths.insert (depth);
                }
            }
        } else if (msg->encoding == "16UC1") {
            const uint16_t* data = reinterpret_cast<const uint16_t*> (msg->data.data ());
            size_t num_pixels = msg->width * msg->height;

            for (size_t i = 0; i < num_pixels; ++i) {
                float depth = data[i] / 1000.0f;  // Convert mm to meters
                unique_depths.insert (depth);
            }
        } else {
            RCLCPP_WARN (this->get_logger (), "Unsupported encoding: %s", msg->encoding.c_str ());
            return;
        }

        RCLCPP_INFO (this->get_logger (), "Found %zu unique depth values:", unique_depths.size ());

        int count = 0;
        for (const auto& depth : unique_depths) {
            std::cout << std::fixed << std::setprecision (3) << depth << " ";
            count++;
            if (count % 10 == 0) {
                std::cout << std::endl;
            }
        }
        if (count % 10 != 0) {
            std::cout << std::endl;
        }

        if (!unique_depths.empty ()) {
            RCLCPP_INFO (this->get_logger (), "Min depth: %.3f, Max depth: %.3f",
                         *unique_depths.begin (), *unique_depths.rbegin ());
        }

        std::cout << "----------------------------------------" << std::endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main (int argc, char* argv[]) {
    rclcpp::init (argc, argv);
    rclcpp::spin (std::make_shared<DepthImageAnalyzer> ());
    rclcpp::shutdown ();
    return 0;
}
