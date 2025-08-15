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
#include "okmr_msgs/srv/get_objects_by_class.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct DetectedObject {
  Eigen::Vector3f position;
  int32_t object_class;

  DetectedObject(Eigen::Vector3f position, int32_t object_class) {
    this->position = position;
    this->object_class = object_class;
  }

  DetectedObject(const okmr_msgs::msg::DetectedObject& ros_msg) {
    position.x() = ros_msg.pose.position.x;
    position.y() = ros_msg.pose.position.y;
    position.z() = ros_msg.pose.position.z;
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

  void update_position_estimate(const Eigen::Vector3f& new_position) {
    // Update position estimate using exponential weighted average
    // See okmr_navigation/src/dead_reckoning.cpp for examples
    // Simple implementation: 0.8 * current + 0.2 * new
    position = position * 0.8f + new_position * 0.2f;
  }
};

// Forward declaration for helper function
std::vector<DetectedObject>
find_objects_in_pointcloud(const pcl::PointCloud<pcl::PointXYZRGBL>& pointcloud);

class ObjectLocator : public rclcpp::Node {
public:
  ObjectLocator() : Node("object_locator") {
    declare_parameter("max_pointclouds", 5);
    declare_parameter("leaf_size", 0.05);
    declare_parameter("max_update_distance", 0.5);
    
    max_pointclouds_ = get_parameter("max_pointclouds").as_int();
    leaf_size_ = get_parameter("leaf_size").as_double();
    max_update_distance_ = get_parameter("max_update_distance").as_double();

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
  double leaf_size_;
  double max_update_distance_; // 50cm
};

// Helper function implementation
std::vector<DetectedObject>
find_objects_in_pointcloud(const pcl::PointCloud<pcl::PointXYZRGBL>& pointcloud) {
  std::vector<DetectedObject> detected_objects;

  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBL>(pointcloud));
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZRGBL>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGBL> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);

  // Create the segmentation object for the planar model and set all parameters
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

  int nr_points = static_cast<int>(cloud_filtered->size());
  
  // Main processing loop - extract planes and find clusters
  while (cloud_filtered->size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
      break;
    }

    // Optional: Extract planar inliers and remove them
    // Uncomment this section if you want to remove planes (e.g., floor/walls)
    /*
    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
    */

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
      }

      if (cloud_cluster->empty()) {
        continue;
      }

      // Find the dominant label in the cluster
      auto dominant_label =
          std::max_element(label_counts.begin(), label_counts.end(),
                           [](const std::pair<uint32_t, int> &a,
                              const std::pair<uint32_t, int> &b) {
                             return a.second < b.second;
                           });

      // Calculate purity and discard impure clusters
      float purity = static_cast<float>(dominant_label->second) / 
                     static_cast<float>(cloud_cluster->size());
      if (purity < 0.7f) {
        continue;
      }

      // Calculate average position and add to detected objects
      Eigen::Vector3f avg_position = position_sum / 
                                     static_cast<float>(cloud_cluster->size());
      detected_objects.emplace_back(avg_position, dominant_label->first);
    }

    // We only process one plane's worth of clusters, then exit
    break;
  }

  return detected_objects;
}

// Method implementations for ObjectLocator class
void ObjectLocator::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Set the cloud timestamp from the ROS message
    cloud->header.stamp = pcl_conversions::toPCL(msg->header).stamp;

    // Downsample using VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_downsampled(
        new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::VoxelGrid<pcl::PointXYZRGBL> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.filter(*cloud_downsampled);
    
    // Append the downsampled pointcloud to the queue
    pointcloud_queue_.push(cloud_downsampled);
    
    // If the size of the queue exceeds the max pointcloud parameter, pop the queue
    while (static_cast<int>(pointcloud_queue_.size()) > max_pointclouds_) {
      pointcloud_queue_.pop();
    }
    
    // Concatenate all pointclouds into one combined pointcloud
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr combined_cloud(
        new pcl::PointCloud<pcl::PointXYZRGBL>);
    combined_cloud->header = cloud_downsampled->header;
    
    std::queue<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> temp_queue = pointcloud_queue_;
    while (!temp_queue.empty()) {
      *combined_cloud += *(temp_queue.front());
      temp_queue.pop();
    }
    
    // Downsample the combined pointcloud
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr final_cloud(
        new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::VoxelGrid<pcl::PointXYZRGBL> vg_combined;
    vg_combined.setInputCloud(combined_cloud);
    vg_combined.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg_combined.filter(*final_cloud);
    
    // Using the downsampled, combined pointcloud, run the find_objects_in_pointcloud algorithm
    std::vector<DetectedObject> detected_objects = 
        find_objects_in_pointcloud(*final_cloud);
    
    // Update long-term object tracking using the class member for max_update_distance
    
    // For each newly detected object
    for (const auto& detected_obj : detected_objects) {
      bool updated_existing = false;
      
      // Try to update existing objects
      for (auto& existing_obj : long_term_objects_) {
        // If same class and close enough, update position
        if (detected_obj.object_class == existing_obj.object_class) {
          float distance = (detected_obj.position - existing_obj.position).norm();
          if (distance < max_update_distance_) {  // Using class member instead of hardcoded value
            existing_obj.update_position_estimate(detected_obj.position);
            updated_existing = true;
            break;
          }
        }
      }
      
      // If no existing object was updated, add this as a new object
      if (!updated_existing) {
        long_term_objects_.push_back(detected_obj);
      }
    }
    
    // Log the current count of tracked objects
    RCLCPP_DEBUG(this->get_logger(), "Currently tracking %zu objects", 
                long_term_objects_.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
  }
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
    } else if (param.get_name() == "leaf_size") {
      double new_value = param.as_double();
      if (new_value <= 0.0) {
        result.successful = false;
        result.reason = "leaf_size must be positive";
        return result;
      }

      leaf_size_ = new_value;
      RCLCPP_INFO(this->get_logger(), "Updated leaf_size to %f", leaf_size_);
    } else if (param.get_name() == "max_update_distance") {
      double new_value = param.as_double();
      if (new_value <= 0.0) {
        result.successful = false;
        result.reason = "max_update_distance must be positive";
        return result;
      }

      max_update_distance_ = new_value;
      RCLCPP_INFO(this->get_logger(), "Updated max_update_distance to %f",
                  max_update_distance_);
    }
  }

  return result;
}

// Main function (moved outside the class)
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectLocator>());
  rclcpp::shutdown();
  return 0;
}