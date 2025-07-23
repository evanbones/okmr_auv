#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <sstream>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class DepthProjectionNode : public rclcpp::Node {
   public:
    DepthProjectionNode ();

   private:
    void depth_callback (const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_and_mask_callback (const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                  const sensor_msgs::msg::Image::ConstSharedPtr& mask_msg);
    void depth_and_rgb_callback (const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                 const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg);
    void depth_mask_and_rgb_callback (const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                      const sensor_msgs::msg::Image::ConstSharedPtr& mask_msg,
                                      const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg);
    void camera_info_callback (const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    sensor_msgs::msg::Image::SharedPtr convertDepthFormat (sensor_msgs::msg::Image::SharedPtr msg);

    bool validateImageDimensions (const sensor_msgs::msg::Image* depth_msg,
                                  const sensor_msgs::msg::Image* rgb_msg = nullptr,
                                  const sensor_msgs::msg::Image* mask_msg = nullptr);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr projectDepthImage (
        const sensor_msgs::msg::Image* depth_msg, const sensor_msgs::msg::Image* rgb_msg = nullptr,
        const sensor_msgs::msg::Image* mask_msg = nullptr);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> mask_filter_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::Image>
        SyncPolicy2;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        SyncPolicy3;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy2>> sync2_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy3>> sync3_;

    float max_dist_;
    float min_dist_;
    double cx_, cy_, fx_inv_, fy_inv_;
    bool camera_info_received_;
    bool use_rgb_;
    bool use_mask_;
};

DepthProjectionNode::DepthProjectionNode ()
    : Node ("depth_to_pointcloud_node"), camera_info_received_ (false) {
    this->declare_parameter ("max_dist", 6.0);
    this->declare_parameter ("min_dist", 0.07);
    this->declare_parameter ("use_rgb", false);
    this->declare_parameter ("use_mask", true);

    max_dist_ = this->get_parameter ("max_dist").as_double ();
    min_dist_ = this->get_parameter ("min_dist").as_double ();
    use_rgb_ = this->get_parameter ("use_rgb").as_bool ();
    use_mask_ = this->get_parameter ("use_mask").as_bool ();

    cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> ("/pointcloud", 10);
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo> (
        "/camera_info", 10,
        std::bind (&DepthProjectionNode::camera_info_callback, this, std::placeholders::_1));

    if (!use_rgb_ && !use_mask_) {
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image> (
            "/depth", 10,
            std::bind (&DepthProjectionNode::depth_callback, this, std::placeholders::_1));
    } else {
        depth_filter_ =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>> (this, "/depth");

        if (use_rgb_ && use_mask_) {
            rgb_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>> (
                this, "/rgb");
            mask_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>> (
                this, "/mask");
            sync3_ = std::make_shared<message_filters::Synchronizer<SyncPolicy3>> (
                SyncPolicy3 (10), *depth_filter_, *mask_filter_, *rgb_filter_);
            sync3_->registerCallback (std::bind (&DepthProjectionNode::depth_mask_and_rgb_callback,
                                                 this, std::placeholders::_1, std::placeholders::_2,
                                                 std::placeholders::_3));
        } else if (use_rgb_) {
            rgb_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>> (
                this, "/rgb");
            sync2_ = std::make_shared<message_filters::Synchronizer<SyncPolicy2>> (
                SyncPolicy2 (10), *depth_filter_, *rgb_filter_);
            sync2_->registerCallback (std::bind (&DepthProjectionNode::depth_and_rgb_callback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
        } else if (use_mask_) {
            mask_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>> (
                this, "/mask");
            sync2_ = std::make_shared<message_filters::Synchronizer<SyncPolicy2>> (
                SyncPolicy2 (10), *depth_filter_, *mask_filter_);
            sync2_->registerCallback (std::bind (&DepthProjectionNode::depth_and_mask_callback,
                                                 this, std::placeholders::_1,
                                                 std::placeholders::_2));
        }
    }
}

void DepthProjectionNode::camera_info_callback (const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    fx_inv_ = 1.0 / msg->k[0];
    fy_inv_ = 1.0 / msg->k[4];
    camera_info_received_ = true;

    RCLCPP_INFO_ONCE (this->get_logger (),
                      "Camera info received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 1.0 / fx_inv_,
                      1.0 / fy_inv_, cx_, cy_);
}

sensor_msgs::msg::Image::SharedPtr DepthProjectionNode::convertDepthFormat (
    sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding == "32FC1") {
        cv::Mat depth_float = cv_bridge::toCvCopy (msg, "32FC1")->image;
        cv::Mat depth_uint16;
        depth_float.convertTo (depth_uint16, CV_16UC1, 1000.0);
        return cv_bridge::CvImage (msg->header, "16UC1", depth_uint16).toImageMsg ();
    }
    return msg;
}

bool DepthProjectionNode::validateImageDimensions (const sensor_msgs::msg::Image* depth_msg,
                                                   const sensor_msgs::msg::Image* rgb_msg,
                                                   const sensor_msgs::msg::Image* mask_msg) {
    if (rgb_msg && (rgb_msg->width != depth_msg->width || rgb_msg->height != depth_msg->height)) {
        RCLCPP_ERROR (this->get_logger (),
                      "Image dimensions do not match: RGB(%dx%d), Depth(%dx%d)", rgb_msg->width,
                      rgb_msg->height, depth_msg->width, depth_msg->height);
        return false;
    }
    if (mask_msg &&
        (mask_msg->width != depth_msg->width || mask_msg->height != depth_msg->height)) {
        RCLCPP_ERROR (this->get_logger (),
                      "Image dimensions do not match: Mask(%dx%d), Depth(%dx%d)", mask_msg->width,
                      mask_msg->height, depth_msg->width, depth_msg->height);
        return false;
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr DepthProjectionNode::projectDepthImage (
    const sensor_msgs::msg::Image* depth_msg, const sensor_msgs::msg::Image* rgb_msg,
    const sensor_msgs::msg::Image* mask_msg) {
    cv::Mat depth_img = cv_bridge::toCvCopy (*depth_msg)->image;
    cv::Mat rgb_img, mask_img;

    if (rgb_msg) {
        rgb_img = cv_bridge::toCvCopy (*rgb_msg)->image;
    }
    if (mask_msg) {
        mask_img = cv_bridge::toCvCopy (*mask_msg)->image;
    }

    const int w = depth_img.cols;
    const int h = depth_img.rows;

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    cloud->header.frame_id = "base_link";
    cloud->header.stamp = pcl_conversions::toPCL (this->now ());
    cloud->is_dense = true;
    cloud->points.reserve (w * h);

    for (int v = 0; v < h; v++) {
        for (int u = 0; u < w; u++) {
            float depth = depth_img.at<unsigned short> (v, u);
            std::uint32_t mask_value = 1;
            std::uint8_t r, g, b;

            if (mask_msg) {
                mask_value = static_cast<std::uint32_t> (mask_img.at<int> (v, u));
            }

            if (rgb_msg) {
                r = rgb_img.at<cv::Vec3b> (v, u)[2];
                g = rgb_img.at<cv::Vec3b> (v, u)[1];
                b = rgb_img.at<cv::Vec3b> (v, u)[0];
            } else {
                float depth_meters = depth * 0.001f;
                float depth_normalized = (depth_meters - min_dist_) / (max_dist_ - min_dist_);
                depth_normalized = std::max (0.0f, std::min (1.0f, depth_normalized));

                float hue = depth_normalized * 240.0f;
                int saturation = 125;
                int value = 255;

                cv::Mat hsv_pixel (1, 1, CV_32FC3, cv::Scalar (hue, saturation, value));
                cv::Mat rgb_pixel;
                cv::cvtColor (hsv_pixel, rgb_pixel, cv::COLOR_HSV2BGR);

                b = static_cast<std::uint8_t> (rgb_pixel.at<cv::Vec3f> (0, 0)[0]);
                g = static_cast<std::uint8_t> (rgb_pixel.at<cv::Vec3f> (0, 0)[1]);
                r = static_cast<std::uint8_t> (rgb_pixel.at<cv::Vec3f> (0, 0)[2]);
            }

            float x = depth * 0.001f;

            if (x > min_dist_ && x < max_dist_ && mask_value != 0) {
                float y = (static_cast<float> (u) - cx_) * x * fx_inv_;
                float z = (static_cast<float> (v) - cy_) * x * fy_inv_;

                pcl::PointXYZRGBL point (x, y, z, r, g, b, mask_value);
                cloud->points.push_back (point);
            }
        }
    }
    cloud->width = cloud->points.size ();
    cloud->height = 1;

    return cloud;
}

void DepthProjectionNode::depth_and_mask_callback (
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& mask_msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 5000,
                              "Camera info not received yet, skipping image processing");
        return;
    }

    auto converted_depth =
        convertDepthFormat (std::const_pointer_cast<sensor_msgs::msg::Image> (depth_msg));
    if (!validateImageDimensions (converted_depth.get (), nullptr, mask_msg.get ())) {
        return;
    }

    auto cloud = projectDepthImage (converted_depth.get (), nullptr, mask_msg.get ());

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg (*cloud, cloud_msg);
    cloud_publisher_->publish (cloud_msg);
}

void DepthProjectionNode::depth_and_rgb_callback (
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 5000,
                              "Camera info not received yet, skipping image processing");
        return;
    }

    auto converted_depth =
        convertDepthFormat (std::const_pointer_cast<sensor_msgs::msg::Image> (depth_msg));

    if (!validateImageDimensions (converted_depth.get (), rgb_msg.get ())) {
        return;
    }

    auto cloud = projectDepthImage (converted_depth.get (), rgb_msg.get (), nullptr);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg (*cloud, cloud_msg);
    cloud_publisher_->publish (cloud_msg);
}

void DepthProjectionNode::depth_mask_and_rgb_callback (
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& mask_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 5000,
                              "Camera info not received yet, skipping image processing");
        return;
    }

    auto converted_depth =
        convertDepthFormat (std::const_pointer_cast<sensor_msgs::msg::Image> (depth_msg));
    if (!validateImageDimensions (converted_depth.get (), rgb_msg.get (), mask_msg.get ())) {
        return;
    }

    auto cloud = projectDepthImage (converted_depth.get (), rgb_msg.get (), mask_msg.get ());

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg (*cloud, cloud_msg);
    cloud_publisher_->publish (cloud_msg);
}

void DepthProjectionNode::depth_callback (const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 5000,
                              "Camera info not received yet, skipping image processing");
        return;
    }

    auto converted_depth = convertDepthFormat (msg);
    if (!validateImageDimensions (converted_depth.get ())) {
        return;
    }

    auto cloud = projectDepthImage (converted_depth.get (), nullptr, nullptr);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg (*cloud, cloud_msg);
    cloud_publisher_->publish (cloud_msg);
}

int main (int argc, char** argv) {
    rclcpp::init (argc, argv);
    auto node = std::make_shared<DepthProjectionNode> ();
    rclcpp::spin (node);
    rclcpp::shutdown ();
    return 0;
}
