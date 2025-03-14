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
        t = []
        try:
            # Read the temperature from the system file
            
            t[0] = open('/sys/class/thermal/thermal_zone0/temp', 'r')
            t[1] = open('/sys/class/thermal/thermal_zone1/temp', 'r')
            t[2] = open('/sys/class/thermal/thermal_zone2/temp', 'r')
            t[3] = open('/sys/class/thermal/thermal_zone3/temp', 'r')
            t[4] = open('/sys/class/thermal/thermal_zone4/temp', 'r')
            t[5] = open('/sys/class/thermal/thermal_zone5/temp', 'r')
            t[6] = open('/sys/class/thermal/thermal_zone6/temp', 'r')
            t[7] = open('/sys/class/thermal/thermal_zone7/temp', 'r')
            t[8] = open('/sys/class/thermal/thermal_zone8/temp', 'r')
            
            for i in t:
                temp_str = i.read().strip()
                temperature = int(temp_str) / 1000.0
                self.temperatures[f'temp{i}'] = temperature
        except FileNotFoundError:
            self.get_logger().error('Temperature sensor file not found')
        except Exception as e:
            self.get_logger().error(f'Error reading temperature: {e}')
        msg = TempReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.Temp0 = t[0]
        msg.Temp1 = t[1]
        msg.Temp2 = t[2]
        msg.Temp3 = t[3]
        msg.Temp4 = t[4]
        msg.Temp5 = t[5]
        msg.Temp6 = t[6]
        msg.Temp7 = t[7]
        msg.Temp8 = t[8]
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
