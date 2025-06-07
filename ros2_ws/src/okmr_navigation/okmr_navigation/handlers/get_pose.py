from okmr_msgs.srv import GetPose


def get_current_pose(node):
    """Helper to get current pose via service call"""
    client = node.create_client(GetPose, 'get_pose')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('get_pose service not available')
        return None
    
    request = GetPose.Request()
    response = client.call(request)
    
    if response.success:
        return response.pose
    else:
        node.get_logger().error('Failed to get current pose from service')
        return None