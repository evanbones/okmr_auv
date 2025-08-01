#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
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
    }
};

// ros2 node that subscribes to a pointcloud2 message on /pointcloud (to be remapped at launch)
//  - maintains a long term list of detected objects
//
//  ------- IMPLEMTN UP TO HERE ----------
//
//  IDEA: pop pointclouds from the list based on header stamp instead of queue size, making x
//  robust to slower pointcloud publishing
//
// service:
// request: a class (int)
// response: list of positions of detected objects for that specific class of object.
//
// on callback:
//
// pointcloud is downsampled using voxelgrid in pcl
//
// last x downsampled pointclouds are saved using a queue. O(n)
// if the new pointcloud arrives and the size of the queue excedes x, pop the queue and append
// the new pointcloud
//
// concatinate all x pointclouds into one combined pointcloud
// downsample the combined pointcloud again, leaving a pointcloud that O(xn)
//  - realistically, each pointcloud should be significantly smaller after being initially
//  downsampled,
//   so the second downsampling should be fast, and will just remove duplicate points
//
//   using the downsampled, combined pointcloud, run the clustering algorithm
//
//   using the detected_objects returned by the clustering algo, update the estimates of the
//   long term detected obejct array
//      - update pose estimate if same class and inside predefined radius
//
//
// clustering algorithm:
//  - https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
//
//  detected_objects list
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
//
//
//
