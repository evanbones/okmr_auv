import rclpy
from rclpy.node import Node
from okmr_msgs.msg import MotorThrottle, MissionCommand, ControlMode
from std_msgs.msg import Header
import serial  # Import pyserial


class SerialOutputNode(Node):
    def __init__(self):
        super().__init__("serial_output_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("polling_rate", 10.0)  # Hz
        self.declare_parameter("leak_threshold", 200)
        serial_port_path = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        polling_rate = self.get_parameter("polling_rate").get_parameter_value().double_value
        self.leak_threshold = self.get_parameter("leak_threshold").get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(serial_port_path, baud_rate, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.subscription = self.create_subscription(
            MotorThrottle, "/motor_throttle", self.throttle_callback, 10
        )
        
        self.mission_publisher = self.create_publisher(MissionCommand, "/mission_command", 10)
        self.control_mode_publisher = self.create_publisher(ControlMode, "/control_mode", 10)
        
        self.timer = self.create_timer(1.0 / polling_rate, self.read_serial_callback)
        self.serial_buffer = ""

    def throttle_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        try:
            for i, throttle_value in enumerate(msg.throttle):
                serial_msg = f"{i}<{throttle_value}\n"
                self.serial_port.write(serial_msg.encode("utf-8"))
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def read_serial_callback(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
            
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8')
                self.serial_buffer += data
                
                # Process complete lines
                while '\n' in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                    self.process_serial_line(line.strip())
                    
        except serial.SerialException as e:
            self.get_logger().error(f"Error reading from serial port: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Error decoding serial data: {e}")
            self.serial_buffer = ""  # Clear buffer on decode error

    def process_serial_line(self, line):
        if '<' in line:
            try:
                parts = line.split('<')
                if len(parts) == 2:
                    sensor_type = parts[0]
                    value = int(parts[1])
                    
                    if sensor_type == "killswitch":
                        if value == 1:  # Killswitch disabled (pin HIGH)
                            # Send KILL_MISSION and OFF mode
                            mission_msg = MissionCommand()
                            mission_msg.command = MissionCommand.KILL_MISSION
                            self.mission_publisher.publish(mission_msg)
                            
                            control_msg = ControlMode()
                            control_msg.header = Header()
                            control_msg.header.stamp = self.get_clock().now().to_msg()
                            control_msg.control_mode = ControlMode.OFF
                            self.control_mode_publisher.publish(control_msg)
                    
                    elif sensor_type == "leak":
                        if value > self.leak_threshold:  # Leak detected
                            # Send KILL_MISSION and OFF mode
                            mission_msg = MissionCommand()
                            mission_msg.command = MissionCommand.KILL_MISSION
                            self.mission_publisher.publish(mission_msg)
                            
                            control_msg = ControlMode()
                            control_msg.header = Header()
                            control_msg.header.stamp = self.get_clock().now().to_msg()
                            control_msg.control_mode = ControlMode.OFF
                            self.control_mode_publisher.publish(control_msg)
                                
            except ValueError as e:
                self.get_logger().warn(f"Error parsing serial line '{line}': {e}")

    def destroy_node(self):
        # Close serial port if open
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialOutputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
