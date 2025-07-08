import rclpy
from rclpy.node import Node
from okmr_msgs.msg import MotorThrottle

import serial
import seaport as sp

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')

        #Set up subscription
        self.subscription = self.create_subscription(
            MotorThrottle,
            'motor_throttle',  # Match this with your publisher topic name
            self.motor_callback,
            10
        )

        #Set up serial communication 
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.seaport = sp.SeaPort(self.ser)
            self.get_logger().info("Serial connection to ESP32 established.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")
            self.ser = None
            self.seaport = None

    def motor_callback(self, msg: MotorThrottle):
        if not self.seaport:
            self.get_logger().warn("No serial connection. Message not sent.")
            return

        # data to send to ESP32
        data = {
            "fli": msg.fli,
            "fri": msg.fri,
            "bli": msg.bli,
            "bri": msg.bri,
            "flo": msg.flo,
            "fro": msg.fro,
            "blo": msg.blo,
            "bro": msg.bro
        }

        
        try:
            self.seaport.publish(1, data)
            self.get_logger().info(f"Sent to ESP32: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send to ESP32: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
