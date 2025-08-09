#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CloudVisualizerNode : public rclcpp::Node {
   public:
    CloudVisualizerNode ();

   private:
    void pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void visualize_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    std::unique_ptr<pcl::visualization::CloudViewer> viewer_;
};

CloudVisualizerNode::CloudVisualizerNode () : Node ("cloud_visualizer_node") {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (
        "/pointcloud", 10,
        std::bind (&CloudVisualizerNode::pointcloud_callback, this, std::placeholders::_1));

    viewer_ = std::make_unique<pcl::visualization::CloudViewer> ("Point Cloud Viewer");

    RCLCPP_INFO (this->get_logger (), "Cloud visualizer node started");
}

void CloudVisualizerNode::pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL (*msg, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2 (pcl_cloud, *rgb_cloud);

    visualize_cloud (rgb_cloud);
}

void CloudVisualizerNode::visualize_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    if (cloud->empty ()) {
        RCLCPP_WARN (this->get_logger (), "Received empty point cloud");
        return;
    }

    viewer_->showCloud (cloud);
    RCLCPP_DEBUG (this->get_logger (), "Visualized cloud with %zu points", cloud->size ());
}

int main (int argc, char **argv) {
    rclcpp::init (argc, argv);
    auto node = std::make_shared<CloudVisualizerNode> ();
    rclcpp::spin (node);
    rclcpp::shutdown ();
    return 0;
}
