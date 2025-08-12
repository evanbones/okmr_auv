#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from okmr_msgs.msg import MotorThrottle
from okmr_msgs.msg import BatteryVoltage
from okmr_msgs.msg import MissionCommand
from std_msgs.msg import String

import serial
import seaport as sp


class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__("esp32_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("killswitch_address", 66)
        self.declare_parameter("killswitch_index", 0)
        self.declare_parameter("motor_count", 8)
        self.declare_parameter("mission_button_address", 66)
        self.declare_parameter("mission_button_index", 1)
        self.declare_parameter("mission_button_arm_time_ms", 3000)
        self.declare_parameter("motor_index_remapping", [0, 1, 2, 3, 4, 5, 6, 7])

        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.killswitch_address = (
            self.get_parameter("killswitch_address").get_parameter_value().integer_value
        )
        self.killswitch_index = (
            self.get_parameter("killswitch_index").get_parameter_value().integer_value
        )
        self.motor_count = (
            self.get_parameter("motor_count").get_parameter_value().integer_value
        )
        self.mission_button_address = (
            self.get_parameter("mission_button_address")
            .get_parameter_value()
            .integer_value
        )
        self.mission_button_index = (
            self.get_parameter("mission_button_index")
            .get_parameter_value()
            .integer_value
        )
        self.mission_button_arm_time_ms = (
            self.get_parameter("mission_button_arm_time_ms")
            .get_parameter_value()
            .integer_value
        )

        self.motor_index_remapping = parameter_value_to_python(
            self.get_parameter("motor_index_remapping")
        )

        self.get_logger().info(f"Motor remapping: {self.motor_index_remapping}")

        self.motor_throttle_sub = self.create_subscription(
            MotorThrottle, "motor_throttle", self.motor_callback, 10
        )

        self.ping_sub = self.create_subscription(String, "ping", self.ping_callback, 10)

        self.mission_command_sub = self.create_subscription(
            MissionCommand, "mission_command", self.mission_command_callback, 10
        )

        self.battery_voltage_pub = self.create_publisher(
            BatteryVoltage, "battery_voltage", 10
        )

        self.mission_command_pub = self.create_publisher(
            MissionCommand, "mission_command", 10
        )

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.seaport = sp.SeaPort(self.ser)
            self.get_logger().info(
                f"Serial connection to ESP32 established on {serial_port} at {baud_rate} baud."
            )
            self.get_logger().info(
                f"Killswitch configured: address={self.killswitch_address}, index={self.killswitch_index}"
            )
            self.get_logger().info(
                f"Mission button configured: address={self.mission_button_address}, index={self.mission_button_index}"
            )
            self.get_logger().info(
                f"Mission button arm time: {self.mission_button_arm_time_ms}ms"
            )

            # self.seaport.subscribe(3, lambda data: imu_accel_callback(data))
            # self.seaport.subscribe(4, lambda data: imu_gyro_callback(data))
            # self.seaport.subscribe(5, lambda data: imu_meta_callback(data))
            self.seaport.subscribe(  # make new msg type for leak sensor
                6, lambda data: self.sensor_board_analog_reading_callback(data)
            )
            self.seaport.subscribe(  # handle killswitch and button callback logic in THIS callback
                7, lambda data: self.sensor_board_digital_reading_callback(data)
            )
            self.seaport.subscribe(254, lambda data: self.pong_callback(data))
            self.seaport.subscribe(
                2, lambda data: self.environment_sensor_callback(data)
            )
            self.seaport.start()

        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")
            self.ser = None
            self.seaport = None

        # Killswitch state tracking
        self.killswitch_active = False
        self.last_killswitch_state = False

        # Mission button state tracking
        self.last_mission_button_state = False
        self.mission_button_press_time = None
        self.mission_armed = False

    def environment_sensor_callback(self, data: dict):
        self.get_logger().info(f"Got environment data: {data}")
        # FIXME new message needed for envorinment data
        pass

    def pong_callback(self, data: dict):
        self.get_logger().info(f"Got pong data: {data}")
        pass

    def sensor_board_analog_reading_callback(self, data: dict):
        self.get_logger().info(f"Got analog data: {data}")

    def sensor_board_digital_reading_callback(self, data: dict):
        """Handle digital inputs including killswitch from sensor boards"""
        # self.get_logger().info(f"Got digital data: {data}")

        # Check if this is killswitch data (configurable address)
        if (
            data.get("a") == self.killswitch_address
            and data.get("i") == self.killswitch_index
        ):
            self.killswitch_callback(data)

        # Check if this is mission button data (configurable address)
        elif (
            data.get("a") == self.mission_button_address
            and data.get("i") == self.mission_button_index
        ):
            self.mission_button_callback(data)

    def killswitch_callback(self, data: dict):
        """Handle killswitch state changes from ESP32"""

        try:
            # Expect data format: {"a": address, "i": index, "v": 0/1}
            # Killswitch is a dial - when pulled it stays active until physically reset
            killswitch_pulled = data["v"]

            # Update current physical state
            self.last_killswitch_state = killswitch_pulled

            # If killswitch is pulled and wasn't already active, trigger kill
            if killswitch_pulled and not self.killswitch_active:
                self.killswitch_active = True
                self.get_logger().error("HARDWARE KILLSWITCH ACTIVATED!")
                self.stop_all_motors()

                # Disarm mission button if armed
                if self.mission_armed:
                    self.mission_armed = False
                    self.get_logger().warn(
                        "Mission button disarmed due to killswitch activation"
                    )

                # Publish hardware kill command
                msg = MissionCommand()
                msg.command = MissionCommand.HARDWARE_KILL
                self.mission_command_pub.publish(msg)

            elif not killswitch_pulled and self.killswitch_active:
                # Killswitch dial was reset - automatically clear kill state
                self.killswitch_active = False
                self.get_logger().info("Hardware killswitch reset - system operational")

        except Exception as e:
            self.get_logger().error(f"Error processing killswitch data: {e}")

    def mission_button_callback(self, data: dict):
        """Handle mission button state changes from ESP32"""
        try:
            # Expect data format: {"a": address, "i": index, "v": 0/1}
            button_pressed = not data["v"]
            current_time = self.get_clock().now()

            # Detect button press (rising edge)
            if button_pressed and not self.last_mission_button_state:
                self.get_logger().info("Mission button pressed - starting hold timer")
                self.mission_button_press_time = current_time

            # Button is being held - check for arming
            elif button_pressed and self.last_mission_button_state:
                if self.mission_button_press_time is not None:
                    hold_duration_ms = (
                        current_time - self.mission_button_press_time
                    ).nanoseconds / 1e6

                    # Check if we should arm the system and start mission
                    if (
                        not self.mission_armed
                        and hold_duration_ms >= self.mission_button_arm_time_ms
                    ):
                        # Check killswitch before allowing arming
                        if self.killswitch_active:
                            self.get_logger().warn(
                                "Cannot arm system - hardware killswitch active"
                            )
                        else:
                            self.mission_armed = True
                            self.get_logger().info(
                                f"Mission button held for {hold_duration_ms:.0f}ms - SYSTEM ARMED"
                            )

                            # Immediately start mission when armed
                            msg = MissionCommand()
                            msg.command = MissionCommand.START_MISSION
                            self.mission_command_pub.publish(msg)
                            self.get_logger().info(
                                "Mission start command published - MISSION STARTED"
                            )

            # Detect button release (falling edge)
            elif not button_pressed and self.last_mission_button_state:
                if self.mission_button_press_time is not None:
                    hold_duration_ms = (
                        current_time - self.mission_button_press_time
                    ).nanoseconds / 1e6
                    self.get_logger().info(
                        f"Mission button released after {hold_duration_ms:.0f}ms"
                    )

                    # Small pulse (< arm time) = disarm if currently armed
                    if hold_duration_ms < self.mission_button_arm_time_ms:
                        if self.mission_armed:
                            self.mission_armed = False
                            self.get_logger().info(
                                "Mission button pulse detected - SYSTEM DISARMED"
                            )
                            self.stop_all_motors()
                        else:
                            self.get_logger().info(
                                f"Button released before arming time ({self.mission_button_arm_time_ms}ms)"
                            )

                    # Long press release - no action needed (mission already started when armed)
                    else:
                        self.get_logger().info(
                            "Mission button released after long press - no action needed"
                        )

                # Reset press time
                self.mission_button_press_time = None

            # Update button state
            self.last_mission_button_state = button_pressed

        except Exception as e:
            self.get_logger().error(f"Error processing mission button data: {e}")

    def stop_all_motors(self):
        try:
            for i in range(self.motor_count):
                data = {str(i): 1500.0}
                self.seaport.publish(1, data)
        except Exception as e:
            self.get_logger().error(f"Failed to send safety stop to ESP32: {e}")

    def mission_command_callback(self, msg: MissionCommand):
        """Handle mission command messages"""
        if msg.command == MissionCommand.HARDWARE_KILL_RESET:
            # Killswitch reset is now automatic when dial is physically reset
            if not self.last_killswitch_state and self.killswitch_active:
                self.killswitch_active = False
                self.get_logger().info(
                    "Hardware killswitch manually reset - system operational"
                )
            elif self.last_killswitch_state:
                self.get_logger().warn(
                    "Cannot reset killswitch while dial is still pulled"
                )
            else:
                self.get_logger().info("Killswitch already reset")

    def ping_callback(self, msg):
        self.seaport.publish(254, {"cmd": "ping"})

    def motor_callback(self, msg: MotorThrottle):
        if not self.seaport:
            self.get_logger().warn("No serial connection. Message not sent.")
            return

        # Check killswitch state before sending motor commands
        if not self.mission_armed:
            self.get_logger().warn("Motor command blocked - disarmed")
            # Send zero throttle to all motors as safety measure
            self.stop_all_motors()
        try:
            for i, throttle in enumerate(msg.throttle):
                if throttle != 0.0:
                    data = {str(self.motor_index_remapping[i]): float(throttle)}
                    self.seaport.publish(1, data)
                # self.get_logger().info(f"Sent to ESP32: {data}")
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
