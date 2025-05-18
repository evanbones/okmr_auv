import rclpy
from rclpy.node import Node
from cascade_msgs.msg import TempReading

class TempSensor(Node):
    def __init__(self):
        super().__init__('temp_sensor')

        self.temp_publisher = self.create_publisher(TempReading, '/temp_sensor', 10)

        self.timer = self.create_timer(1, self.timer_callback)
        self.temperatures = {f'temp{i}': 0.0 for i in range(10)}

    def timer_callback(self):
        temp_files = [
            '/sys/class/thermal/thermal_zone0/temp',
            '/sys/class/thermal/thermal_zone1/temp',
            '/sys/class/thermal/thermal_zone2/temp',
            '/sys/class/thermal/thermal_zone3/temp',
            '/sys/class/thermal/thermal_zone4/temp',
            '/sys/class/thermal/thermal_zone5/temp',
            '/sys/class/thermal/thermal_zone6/temp',
            '/sys/class/thermal/thermal_zone7/temp',
            '/sys/class/thermal/thermal_zone8/temp'
        ]

        for i, temp_file in enumerate(temp_files):
            try:
                with open(temp_file, 'r') as file:
                    temp_str = file.read().strip()
                    temperature = int(temp_str) / 1000.0  # Convert from millidegrees to degrees
                    self.temperatures[f'temp{i}'] = temperature
            except FileNotFoundError:
                self.get_logger().error(f'Temperature sensor file {temp_file} not found')
            except Exception as e:
                self.get_logger().error(f'Error reading temperature from {temp_file}: {e}')

        msg = TempReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temp0 = self.temperatures['temp0']
        msg.temp1 = self.temperatures['temp1']
        msg.temp2 = self.temperatures['temp2']
        msg.temp3 = self.temperatures['temp3']
        msg.temp4 = self.temperatures['temp4']
        msg.temp5 = self.temperatures['temp5']
        msg.temp6 = self.temperatures['temp6']
        msg.temp7 = self.temperatures['temp7']
        msg.temp8 = self.temperatures['temp8']
        self.temp_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TempSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
