from okmr_msgs.action import Movement
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.set_velocity_handler import handle_set_velocity

def handle_barrel_roll(goal_handle):
    """Execute barrel roll by disabling dead reckoning, calling handle_set_velocity, then re-enabling"""
    node = NavigatorActionServer.get_instance()
    
    # Step 1: Disable dead reckoning
    if not disable_dead_reckoning(node):
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = 'Failed to disable dead reckoning for barrel roll'
        return result
    
    node.get_logger().info("Dead reckoning disabled, starting barrel roll")
    
    # Step 2: Execute barrel roll using existing set velocity handler
    result = handle_set_velocity(goal_handle)
    
    # Step 3: Re-enable dead reckoning
    enable_success = enable_dead_reckoning(node)
    
    if not enable_success:
        # If the velocity command succeeded but we failed to re-enable dead reckoning
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = 'Barrel roll completed but failed to re-enable dead reckoning'
        return result
    
    node.get_logger().info("Barrel roll completed, dead reckoning re-enabled")
    
    result.debug_info = 'Barrel roll completed and re-enabled dead reckoning'
    
    return result


def disable_dead_reckoning(node):
    """Disable dead reckoning service"""
    client = node.create_client(SetDeadReckoningEnabled, '/set_dead_reckoning_enabled')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Dead reckoning service not available')
        return False
    
    request = SetDeadReckoningEnabled.Request()
    request.enable = False
    
    try:
        response = client.call(request)
        if response.success:
            node.get_logger().info(f'Dead reckoning disabled: {response.message}')
            return True
        else:
            node.get_logger().error(f'Failed to disable dead reckoning: {response.message}')
            return False
    except Exception as e:
        node.get_logger().error(f'Dead reckoning disable service call failed: {e}')
        return False


def enable_dead_reckoning(node):
    """Enable dead reckoning service"""
    client = node.create_client(SetDeadReckoningEnabled, '/set_dead_reckoning_enabled')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Dead reckoning service not available')
        return False
    
    request = SetDeadReckoningEnabled.Request()
    request.enable = True
    
    try:
        response = client.call(request)
        if response.success:
            node.get_logger().info(f'Dead reckoning enabled: {response.message}')
            return True
        else:
            node.get_logger().error(f'Failed to enable dead reckoning: {response.message}')
            return False
    except Exception as e:
        node.get_logger().error(f'Dead reckoning enable service call failed: {e}')
        return False
