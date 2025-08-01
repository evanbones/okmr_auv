#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "okmr_msgs/msg/detected_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct DetectedObject {
    Eigen::Vector3f position;
    int32 object_class;

    DetectedObject (Eigen::Vector3f position, int32 object_class) {
        this->position = position;
        this->object_class = object_class;
    }

    DetectedObject (okmr_msgs::msg::DetectedObject ros_msg) {
        position.x = ros_msg.x ();
        position.y = ros_msg.y ();
        position.z = ros_msg.z ();
        object_class = ros_msg.object_class;
    }

    okmr_msgs::msg::DetectedObject to_ros_msg () {
        auto msg = okmr_msgs::msg::DetectedObject ();
        msg.object_class = object_class;
        msg.pose.position.x = position.x ();
        msg.pose.position.y = position.y ();
        msg.pose.position.z = position.z ();
        return msg;
    }

    void update_position_estimate (Eigen::Vector3f new_position) {
        //  update position esitimate using exponential
        //  weighted average / complemntary filter
        //  see okmr_navigation/src/dead_reckoning.cpp for examples
    }
};

class ObjectLocator : public rclcpp::Node {
   public:
    ObjectLocator () : Node ("object_locator") {
        declare_parameter ("max_pointclouds", 5);
        max_pointclouds_ = get_parameter ("max_pointclouds").as_int ();

        param_callback_handle_ = this->add_on_set_parameters_callback (
            std::bind (&ObjectLocator::on_parameter_change, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (
            "/pointcloud", 10,
            std::bind (&ObjectLocator::pointcloud_callback, this, std::placeholders::_1));

        object_service_ = this->create_service<okmr_msgs::srv::GetObjectsByClass> (
            "get_objects_by_class", std::bind (&ObjectLocator::get_objects_by_class, this,
                                               std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO (this->get_logger (), "ObjectLocator node initialized with max_pointclouds=%d",
                     max_pointclouds_);
    }

   private:
    void pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void get_objects_by_class (
        const std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Request> request,
        std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Response> response);

    rcl_interfaces::msg::SetParametersResult on_parameter_change (
        const std::vector<rclcpp::Parameter>& parameters);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Service<okmr_msgs::srv::GetObjectsByClass>::SharedPtr object_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::vector<DetectedObject> long_term_objects_;
    std::queue<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> pointcloud_queue_;
    int max_pointclouds_;
};

std::vector<DetectedObject> find_objects_in_pointcloud (
    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud) {
    // declare detected_objects list (it is the result to be returned)
    //
    // get euclidian clusters from the pointcloud
    // clustering algorithm:
    //  - https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
    //  NOTE: make the float inside seg.setDistanceThreshold (float); a live ros2 paramter
    //
    // iterate through each cluster pointcloud
    //  - iterate through every point in the cluster, recording the # of occurances of any labels
    //  and summing their position
    //    - get the the average position of the mode class inside the cluster.
    //    - if there is more than y% non-mode points, show a ros warn and discard the cluster
    //  - append the estimated position and class to the detected_objects array
    //
    // transform calculated average positions from base_link to map frame
    //   - note: need to add base_link to map frame transform publisher to okmr_navigation
    //
    //  return detected_objects
}

void ObjectLocator::pointcloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // pointcloud is downsampled using voxelgrid in pcl
    // NOTE: make the leaf size a cube, and make it a live ros2 parameter
    // Use for initial downsampling
    // https://pointclouds.org/documentation/classpcl_1_1_approximate_voxel_grid.html#details
    //
    // append the downsampled pointcloud to the queue
    // if the the size of the queue excedes the max pointcloud parameter, pop the queue
    //
    // concatinate all x pointclouds into one combined pointcloud (using += operator)
    // set the header stamp of the combined pointcloud equal to the ros2 message stamp
    //
    // downsample the combined pointcloud
    //
    // Use for downsampling concatenated pointcloud:
    // https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html#details
    //
    //   using the downsampled, combined pointcloud, run the find_objects_in_pointcloud algorithm
    //
    //   using the detected_objects returned by the clustering algo, update the estimates of the
    //   long term detected obejct array
    //      - update pose estimate if same class and inside predefined radius

    //  FUTURE WORK: pop pointclouds from the list based on header stamp instead of queue size,
    //  making x robust to slower pointcloud publishing
}

void ObjectLocator::get_objects_by_class (
    const std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Request> request,
    std::shared_ptr<okmr_msgs::srv::GetObjectsByClass::Response> response) {
    for (const auto& obj : long_term_objects_) {
        if (obj.object_class == request->object_class) {
            response->objects.push_back (obj.to_ros_msg ());
        }
    }

    RCLCPP_INFO (this->get_logger (), "Returning %zu objects of class %d",
                 response->objects.size (), request->object_class);
}

rcl_interfaces::msg::SetParametersResult ObjectLocator::on_parameter_change (
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        if (param.get_name () == "max_pointclouds") {
            int new_value = param.as_int ();
            if (new_value <= 0) {
                result.successful = false;
                result.reason = "max_pointclouds must be positive";
                return result;
            }

            max_pointclouds_ = new_value;
            RCLCPP_INFO (this->get_logger (), "Updated max_pointclouds to %d", max_pointclouds_);

            while (static_cast<int> (pointcloud_queue_.size ()) > max_pointclouds_) {
                pointcloud_queue_.pop ();
            }
        }
    }

    return result;
}

int main (int argc, char** argv) {
    rclcpp::init (argc, argv);
    rclcpp::spin (std::make_shared<ObjectLocator> ());
    rclcpp::shutdown ();
    return 0;
}
