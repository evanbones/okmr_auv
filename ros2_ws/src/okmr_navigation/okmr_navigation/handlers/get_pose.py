from okmr_msgs.srv import GetPose
import rclpy
from rclpy.node import Node


def get_current_pose(parent_node):
    """
    Helper to get current pose via service call using a dedicated subnode.
    Creates a temporary node to make the service call without interfering with the parent node.
    """
    # Create a dedicated subnode for the service call
    subnode = Node('get_pose_client')
    
    try:
        # Create service client on the subnode
        client = subnode.create_client(GetPose, 'get_pose')
        
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=2.0):
            parent_node.get_logger().error('get_pose service not available after 2s timeout')
            return None
        
        # Make the service call
        request = GetPose.Request()
        future = client.call_async(request)
        
        # Spin the subnode until the future completes
        rclpy.spin_until_future_complete(subnode, future, timeout_sec=2.0)
        
        if not future.done():
            parent_node.get_logger().error('get_pose service call timed out')
            return None
        
        # Get the response
        response = future.result()
        
        # Check response and return pose
        if response is not None and hasattr(response, 'success') and response.success:
            return response.pose
        else:
            parent_node.get_logger().error('get_pose service returned failure or invalid response')
            return None
            
    except Exception as e:
        parent_node.get_logger().error(f'Exception in get_current_pose: {str(e)}')
        return None
    finally:
        # Always clean up the subnode
        subnode.destroy_node()
