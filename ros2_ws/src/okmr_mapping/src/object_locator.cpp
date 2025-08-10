#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <map>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "okmr_msgs/msg/detected_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct DetectedObject {
  Eigen::Vector3f position;
  int32 object_class;

  DetectedObject(Eigen::Vector3f position, int32 object_class) {
    this->position = position;
    this->object_class = object_class;
  }

  DetectedObject(okmr_msgs::msg::DetectedObject ros_msg) {
    position.x = ros_msg.x();
    position.y = ros_msg.y();
    position.z = ros_msg.z();
    object_class = ros_msg.object_class;
  }

  okmr_msgs::msg::DetectedObject to_ros_msg() {
    auto msg = okmr_msgs::msg::DetectedObject();
    msg.object_class = object_class;
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = position.z();
    return msg;
  }

  void update_position_estimate(Eigen::Vector3f new_position) {
    //  update position esitimate using exponential
    //  weighted average / complemntary filter
    //  see okmr_navigation/src/dead_reckoning.cpp for examples
  }
};

class ObjectLocator : public rclcpp::Node {
public:
  ObjectLocator() : Node("object_locator") {
    declare_parameter("max_pointclouds", 5);
    max_pointclouds_ = get_parameter("max_pointclouds").as_int();

    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &ObjectLocator::on_parameter_change, this, std::placeholders::_1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pointcloud", 10,
        std::bind(&ObjectLocator::pointcloud_callback, this,
                  std::placeholders::_1));

    object_service_ = this->create_service<okmr_msgs::srv::GetObjectsByClass>(
        "get_objects_by_class",
        std::bind(&ObjectLocator::get_objects_by_class, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "ObjectLocator node initialized with max_pointclouds=%d",
                max_pointclouds_);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void get_objects_by_class(
      const std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Request> request,
      std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Response> response);

  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Service<okmr_msgs::srv::GetObjectsByClass>::SharedPtr object_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  std::vector<DetectedObject> long_term_objects_;
  std::queue<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> pointcloud_queue_;
  int max_pointclouds_;
};

std::vector<DetectedObject>
find_objects_in_pointcloud(pcl::PointCloud<pcl::PointXYZRGBL> pointcloud) {
  std::vector<DetectedObject> detected_objects;

  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBL>(pointcloud));
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZRGBL>);

  // Create the filtering object: downsample the dataset using a leaf size of
  // 1cm
  pcl::VoxelGrid<pcl::PointXYZRGBL> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentation<pcl::PointXYZRGBL> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZRGBL>());

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int nr_points = (int)cloud_filtered->size();
  while (cloud_filtered->size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      break;
    }

    // Extract the planar inliers from the input cloud
    // pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
    // extract.setInputCloud(cloud_filtered);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    // extract.setNegative(true);
    // extract.filter(*cloud_f);
    //*cloud_filtered = *cloud_f;int
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGBL>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZRGBL>);
      std::map<uint32_t, int> label_counts;
      Eigen::Vector3f position_sum(0.0f, 0.0f, 0.0f);

      for (const auto &idx : cluster.indices) {
        const auto &point = (*cloud_filtered)[idx];
        cloud_cluster->push_back(point);

        // Count label occurrences and sum positions
        label_counts[point.label]++;
        position_sum += Eigen::Vector3f(point.x, point.y, point.z);
      } //*

      if (cloud_cluster->empty())
        continue;

      // Find the dominant label in the cluster
      auto dominant_label =
          std::max_element(label_counts.begin(), label_counts.end(),
                           [](const std::pair<uint32_t, int> &a,
                              const std::pair<uint32_t, int> &b) {
                             return a.second < b.second;
                           });

      // Calculate purity and discard impure clusters
      float purity = (float)dominant_label->second / cloud_cluster->size();
      if (purity < 0.7f) {
        continue;
      }

      // Calculate average position and add to detected objects
      Eigen::Vector3f avg_position = position_sum / cloud_cluster->size();
      detected_objects.emplace_back(avg_position, dominant_label->first);
    }

    return detected_objects;
  }

  void ObjectLocator::pointcloud_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // pointcloud is downsampled using voxelgrid in pcl
    // NOTE: make the leaf size a cube, and make it a live ros2 parameter
    // Use for initial downsampling
    // https://pointclouds.org/documentation/classpcl_1_1_approximate_voxel_grid.html#details
    //
    // append the downsampled pointcloud to the queue
    // if the the size of the queue excedes the max pointcloud parameter, pop
    // the queue
    //
    // concatinate all x pointclouds into one combined pointcloud (using +=
    // operator) set the header stamp of the combined pointcloud equal to the
    // ros2 message stamp
    //
    // downsample the combined pointcloud
    //
    // Use for downsampling concatenated pointcloud:
    // https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html#details
    //
    //   using the downsampled, combined pointcloud, run the
    //   find_objects_in_pointcloud algorithm
    //
    //   using the detected_objects returned by the clustering algo, update the
    //   estimates of the long term detected obejct array
    //      - update pose estimate if same class and inside predefined radius

    //  FUTURE WORK: pop pointclouds from the list based on header stamp instead
    //  of queue size, making x robust to slower pointcloud publishing
  }

  void ObjectLocator::get_objects_by_class(
      const std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Request> request,
      std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Response> response) {
    for (const auto &obj : long_term_objects_) {
      if (obj.object_class == request->object_class) {
        response->objects.push_back(obj.to_ros_msg());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Returning %zu objects of class %d",
                response->objects.size(), request->object_class);
  }

  rcl_interfaces::msg::SetParametersResult ObjectLocator::on_parameter_change(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
      if (param.get_name() == "max_pointclouds") {
        int new_value = param.as_int();
        if (new_value <= 0) {
          result.successful = false;
          result.reason = "max_pointclouds must be positive";
          return result;
        }

        max_pointclouds_ = new_value;
        RCLCPP_INFO(this->get_logger(), "Updated max_pointclouds to %d",
                    max_pointclouds_);

        while (static_cast<int>(pointcloud_queue_.size()) > max_pointclouds_) {
          pointcloud_queue_.pop();
        }
      }
    }

    return result;
  }

  int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectLocator>());
    rclcpp::shutdown();
    return 0;
  }
