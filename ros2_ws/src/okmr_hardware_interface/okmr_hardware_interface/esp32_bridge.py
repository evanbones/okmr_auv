#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from okmr_msgs.msg import MotorThrottle
from okmr_msgs.msg import BatteryVoltage

import serial
import seaport as sp


class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__("esp32_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.motor_throttle_sub = self.create_subscription(
            MotorThrottle, "motor_throttle", self.motor_callback, 10
        )

        self.battery_voltage_pub = self.create_publisher(
            BatteryVoltage, "battery_voltage", 10
        )

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.seaport = sp.SeaPort(self.ser)
            self.get_logger().info(
                f"Serial connection to ESP32 established on {serial_port} at {baud_rate} baud."
            )

            self.seaport.subscribe(
                2, lambda data: self.environment_sensor_callback(data)
            )
            # self.seaport.subscribe(3, lambda data: imu_accel_callback(data))
            # self.seaport.subscribe(4, lambda data: imu_gyro_callback(data))
            # self.seaport.subscribe(5, lambda data: imu_meta_callback(data))
            self.seaport.subscribe(
                6, lambda data: self.sensor_board_analog_reading_callback(data)
            )
            self.seaport.subscribe(
                7, lambda data: self.sensor_board_digital_reading_callback(data)
            )
            self.seaport.start()

        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")
            self.ser = None
            self.seaport = None

    def environment_sensor_callback(self, data: dict):
        self.get_logger().info(f"Got environmental data: {dict}")

    def sensor_board_analog_reading_callback(self, data: dict):
        self.get_logger().info(f"Got analog data: {dict}")

    def sensor_board_digital_reading_callback(self, data: dict):
        self.get_logger().info(f"Got digital data: {dict}")

    def motor_callback(self, msg: MotorThrottle):
        if not self.seaport:
            self.get_logger().warn("No serial connection. Message not sent.")
            return

        self.get_logger().info("got motor throttel")

        data = {
            "0": int(msg.throttle[0]),
            "1": int(msg.throttle[1]),
            "2": int(msg.throttle[2]),
            "3": int(msg.throttle[3]),
            "4": int(msg.throttle[4]),
            "5": int(msg.throttle[5]),
            "6": int(msg.throttle[6]),
            "7": int(msg.throttle[7]),
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


if __name__ == "__main__":
    main()
