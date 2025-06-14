from okmr_msgs.srv import GetPose
import rclpy
from rclpy.node import Node


def get_current_pose(parent_node):
    try:
        client = parent_node.create_client(GetPose, 'get_pose')
        
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=2.0):
            parent_node.get_logger().error('get_pose service not available after 2s timeout')
            return None
        
        # Make the blocking service call
        request = GetPose.Request()
        response = client.call(request, timeout_sec=2.0)
        #TODO: is this async call fine if we use multi threaded executor?
        
        # Check response and return pose
        if response is not None and hasattr(response, 'success') and response.success:
            return response.pose
        else:
            parent_node.get_logger().error('get_pose service returned failure or invalid response')
            return None
            
    except Exception as e:
        parent_node.get_logger().error(f'Exception in get_current_pose: {str(e)}')
        return None
