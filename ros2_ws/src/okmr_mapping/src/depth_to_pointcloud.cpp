#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/core.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include "okmr_msgs/msg/semantic_depth.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <vector>

class DepthProjectionNode : public rclcpp::Node {
public: 
    DepthProjectionNode();
    
private:
    void semantic_depth_callback(const okmr_msgs::msg::SemanticDepth::SharedPtr msg);
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr projectDepthImage(const okmr_msgs::msg::SemanticDepth& img, bool rainbow_mode);
    
    rclcpp::Subscription<okmr_msgs::msg::SemanticDepth>::SharedPtr semantic_depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    
    float max_dist_;
    float min_dist_;
    double cx_, cy_, fx_inv_, fy_inv_;
    bool camera_info_received_;
};

DepthProjectionNode::DepthProjectionNode() : Node("depth_to_pointcloud_node"), camera_info_received_(false) {
    this->declare_parameter("max_dist", 6.0);
    this->declare_parameter("min_dist", 0.07);
    this->declare_parameter("use_semantic_subscriber", true);
    
    max_dist_ = this->get_parameter("max_dist").as_double();
    min_dist_ = this->get_parameter("min_dist").as_double();
    
    cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10, std::bind(&DepthProjectionNode::camera_info_callback, this, std::placeholders::_1));
    
    bool use_semantic = this->get_parameter("use_semantic_subscriber").as_bool();
    if (use_semantic) {
        semantic_depth_sub_ = this->create_subscription<okmr_msgs::msg::SemanticDepth>(
            "/semantic_depth", 10, std::bind(&DepthProjectionNode::semantic_depth_callback, this, std::placeholders::_1));
    } else {
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth", 10, std::bind(&DepthProjectionNode::depth_callback, this, std::placeholders::_1));
    }
}

void DepthProjectionNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    cx_ = msg->k[2];
    cy_ = msg->k[5]; 
    fx_inv_ = 1.0 / msg->k[0];
    fy_inv_ = 1.0 / msg->k[4];
    camera_info_received_ = true;
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Camera info received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                     1.0/fx_inv_, 1.0/fy_inv_, cx_, cy_);
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr DepthProjectionNode::projectDepthImage(const okmr_msgs::msg::SemanticDepth& img, bool rainbow_mode = false) {
    cv::Mat depth_img = cv_bridge::toCvCopy(img.depth)->image;
    cv::Mat rgb_img = cv_bridge::toCvCopy(img.rgb)->image;
    cv::Mat mask_img = cv_bridge::toCvCopy(img.mask)->image;

    const int w = depth_img.cols; 
    const int h = depth_img.rows;
    
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    cloud->header.frame_id = "base_link";
    cloud->header.stamp = pcl_conversions::toPCL(this->now());
    cloud->is_dense = true;
    cloud->points.reserve(w * h);


    for (int v = 0; v < h; v++) {
        for (int u = 0; u < w; u++) {
            float depth = depth_img.at<unsigned short>(v, u);
            std::uint32_t mask_value = static_cast<std::uint32_t>(mask_img.at<int>(v, u));
            std::uint8_t r, g, b;
            if (!rainbow_mode) {
                r = rgb_img.at<cv::Vec3b>(v, u)[2]; 
                g = rgb_img.at<cv::Vec3b>(v, u)[1];
                b = rgb_img.at<cv::Vec3b>(v, u)[0]; 
            } else {
                // Rainbow colorizing based on depth (close = red, far = blue)
                float depth_meters = depth * 0.001f;
                float depth_normalized = (depth_meters - min_dist_) / (max_dist_ - min_dist_);
                depth_normalized = std::max(0.0f, std::min(1.0f, depth_normalized));
                
                // HSV: Hue from 0 (red) to 240 (blue), full saturation and value
                float hue = (1.0f - depth_normalized) * 240.0f; // Invert so close is red
                cv::Mat hsv_pixel(1, 1, CV_32FC3, cv::Scalar(hue, 125, 255));
                cv::Mat rgb_pixel;
                cv::cvtColor(hsv_pixel, rgb_pixel, cv::COLOR_HSV2BGR);
                
                b = static_cast<std::uint8_t>(rgb_pixel.at<cv::Vec3f>(0, 0)[0]);
                g = static_cast<std::uint8_t>(rgb_pixel.at<cv::Vec3f>(0, 0)[1]); 
                r = static_cast<std::uint8_t>(rgb_pixel.at<cv::Vec3f>(0, 0)[2]);
            }

            float x = depth * 0.001f;
    
            //hardcoded transform. X is forward, Y is left / right, Z is up down
            if (x > min_dist_ && x < max_dist_ && mask_value != 0) {
                float y = (static_cast<float>(u) - cx_) * x * fx_inv_; 
                float z = (static_cast<float>(v) - cy_) * x * fy_inv_; 
                
                pcl::PointXYZRGBL point(x, y, z, r, g, b, mask_value);
                cloud->points.push_back(point);
            }
        }  
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    return cloud;
}

//add convert from float to uint method
//takes depth image in F32C1 format and converts to U16C1

void DepthProjectionNode::semantic_depth_callback(const okmr_msgs::msg::SemanticDepth::SharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Camera info not received yet, skipping image processing");
        return;
    }
    
    if (msg->rgb.width != msg->depth.width || msg->rgb.height != msg->depth.height ||
        msg->mask.width != msg->depth.width || msg->mask.height != msg->depth.height) {
        RCLCPP_ERROR(this->get_logger(), "Image dimensions do not match: RGB(%dx%d), Depth(%dx%d), Mask(%dx%d)",
                     msg->rgb.width, msg->rgb.height, msg->depth.width, msg->depth.height,
                     msg->mask.width, msg->mask.height);
        return;
    }
    
    auto cloud = projectDepthImage(*msg);
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_publisher_->publish(cloud_msg);
}

void DepthProjectionNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Camera info not received yet, skipping image processing");
        return;
    }
    
    okmr_msgs::msg::SemanticDepth semantic_msg;

    if (msg->encoding == "32FC1") {
        cv::Mat depth_float = cv_bridge::toCvCopy(msg, "32FC1")->image;
        cv::Mat depth_uint16;
        depth_float.convertTo(depth_uint16, CV_16UC1, 1000.0);
        semantic_msg.depth = *cv_bridge::CvImage(msg->header, "16UC1", depth_uint16).toImageMsg();
    } else {
        semantic_msg.depth = *msg;
    }
    
    cv::Mat white_rgb = cv::Mat::ones(msg->height, msg->width, CV_8UC3);
    white_rgb.setTo(cv::Scalar(255, 255, 255));
    semantic_msg.rgb = *cv_bridge::CvImage(msg->header, "bgr8", white_rgb).toImageMsg();
    
    cv::Mat mask_ones = cv::Mat::ones(msg->height, msg->width, CV_32SC1);
    semantic_msg.mask = *cv_bridge::CvImage(msg->header, "32SC1", mask_ones).toImageMsg();
    
    auto cloud = projectDepthImage(semantic_msg, true);
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_publisher_->publish(cloud_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthProjectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
