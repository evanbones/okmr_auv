import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from okmr_msgs.msg import MaskOffset
import numpy as np


class MaskOffsetNode(Node):
    def __init__(self):
        super().__init__("mask_offset_node")
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "/mask", self.mask_callback, 10
        )

        self.publisher = self.create_publisher(MaskOffset, "/mask_offset", 10)

    def mask_callback(self, msg):
        try:
            mask_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        height, width = mask_image.shape
        non_zero_coords = np.nonzero(mask_image)

        if len(non_zero_coords[0]) == 0:
            offset_msg = MaskOffset()
            offset_msg.y_offset = 0.0
            offset_msg.z_offset = 0.0
            self.publisher.publish(offset_msg)
            return

        avg_row = np.mean(non_zero_coords[0])
        avg_col = np.mean(non_zero_coords[1])

        y_offset = -(avg_col - width / 2.0) / (width / 2.0)
        z_offset = -(avg_row - height / 2.0) / (height / 2.0)

        offset_msg = MaskOffset()
        offset_msg.y_offset = float(y_offset)
        offset_msg.z_offset = float(z_offset)

        self.publisher.publish(offset_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MaskOffsetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
