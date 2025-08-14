from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_msgs.msg import ControlMode
from okmr_utils.logging import make_green_log
import time


def set_control_mode(mode):
    """
    Set the control mode for the vehicle.

    Args:
        mode: Control mode constant from ControlMode message
              (ControlMode.POSE, ControlMode.VELOCITY, etc.)

    Returns:
        bool: True if successful, False otherwise
    """
    node = NavigatorActionServer.get_instance()

    # Validate mode
    valid_modes = [
        ControlMode.POSE,
        ControlMode.VELOCITY,
        ControlMode.ACCELERATION,
        ControlMode.THRUST,
        ControlMode.THROTTLE,
        ControlMode.OFF,
    ]

    if mode not in valid_modes:
        node.get_logger().error(f"Invalid control mode: {mode}")
        return False

    try:
        # Create control mode message
        control_msg = ControlMode()
        control_msg.control_mode = mode

        # Get publisher and publish
        control_publisher = node.get_publisher("/control_mode", ControlMode, 10)
        for i in range(20):
            control_publisher.publish(control_msg)
            control_msg.header.stamp = node.get_clock().now()
            time.sleep(0.05)

        mode_names = {
            ControlMode.POSE: "POSE",
            ControlMode.VELOCITY: "VELOCITY",
            ControlMode.ACCELERATION: "ACCELERATION",
            ControlMode.THRUST: "THRUST",
            ControlMode.THROTTLE: "THROTTLE",
            ControlMode.OFF: "OFF",
        }
        if mode in [ControlMode.POSE, ControlMode.VELOCITY, ControlMode.OFF]:
            node.get_logger().info(
                f"Control mode set to {make_green_log(mode_names.get(mode, 'UNKNOWN'))}"
            )
        else:
            node.get_logger().warn(
                f"Control mode set to {make_green_log(mode_names.get(mode, 'UNKNOWN'))}. DEBUG ONLY"
            )
        return True

    except Exception as e:
        node.get_logger().error(f"Failed to set control mode {mode}: {str(e)}")
        return False
