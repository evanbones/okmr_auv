#!/usr/bin/env python3

import socket
import re
import rclpy
from rclpy.node import Node
from okmr_msgs.msg import Dvl
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import FluidPressure


class DvlDriverNode(Node):
    def __init__(self):
        super().__init__("dvl_driver")
        self.dvl_publisher = self.create_publisher(Dvl, "/dvl", 10)

        # Declare ROS2 parameter for beam range threshold
        self.declare_parameter("beam_range_threshold", 0.8)
        self.beam_range_threshold = (
            self.get_parameter("beam_range_threshold")
            .get_parameter_value()
            .double_value
        )

        # Store previous velocity values for outlier rejection
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0

        # Store previous beam distances for outlier rejection
        self.last_d1 = 0.0
        self.last_d2 = 0.0
        self.last_d3 = 0.0
        self.last_d4 = 0.0

    def udp_server(self):
        # Create a UDP socket
        sock_bottom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the host and port
        sock_bottom.bind(("0.0.0.0", 9010))

        while True:
            # Receive data from the client
            data, dvl_addr = sock_bottom.recvfrom(1024)  # buffer size is 1024 bytes
            data = data.decode("ASCII")

            # Regular expression to match the PNORBT8 message format
            pattern = r"\$PNORBT8,TIME=(-?\d+\.?\d*),DT1=(-?\d+\.\d+),DT2=(-?\d+\.\d+),VX=(-?\d+\.\d+),VY=(-?\d+\.\d+),VZ=(-?\d+\.\d+),FOM=(-?\d+\.\d+),D1=(-?\d+\.\d+),D2=(-?\d+\.\d+),D3=(-?\d+\.\d+),D4=(-?\d+\.\d+),BATT=(-?\d+\.\d+),SS=(-?\d+\.\d+),PRESS=(-?\d+\.\d+),TEMP=(-?\d+\.\d+),STAT=(0x[0-9A-Fa-f]+)"

            # Find the matches
            matches = re.search(pattern, data)

            if matches:
                # Extracting values from regex groups
                time_val = float(matches.group(1))
                dt1 = float(matches.group(2))
                dt2 = float(matches.group(3))
                vx = float(matches.group(4))
                vy = float(matches.group(5))
                vz = float(matches.group(6))
                fom = float(matches.group(7))
                d1 = float(matches.group(8))
                d2 = float(matches.group(9))
                d3 = float(matches.group(10))
                d4 = float(matches.group(11))
                battery = float(matches.group(12))
                speed_sound = float(matches.group(13))
                pressure = float(matches.group(14))
                temperature = float(matches.group(15))
                status = int(matches.group(16), 16)  # Convert hex string to int

                # DVL velocity validation
                velocity_magnitude = (vx**2 + vy**2 + vz**2) ** 0.5
                if velocity_magnitude > 30.0:
                    self.get_logger().warn(
                        f"DVL velocity too high ({velocity_magnitude:.3f} m/s), last velocity - VX: {self.last_vx:.3f}, VY: {self.last_vy:.3f}, VZ: {self.last_vz:.3f}",
                        throttle_duration_sec=1.0,
                    )
                    vx = self.last_vx
                    vy = self.last_vy
                    vz = 0.0
                else:
                    self.last_vx = vx
                    self.last_vy = vy
                    self.last_vz = vz

                # DVL beam distance validation
                beam_distances = [d1, d2, d3, d4]
                beam_range = max(beam_distances) - min(beam_distances)

                if beam_range > self.beam_range_threshold:
                    self.get_logger().warn(
                        f"DVL beam range too large ({beam_range:.3f} m), using previous values - D1: {self.last_d1:.3f}, D2: {self.last_d2:.3f}, D3: {self.last_d3:.3f}, D4: {self.last_d4:.3f}",
                        throttle_duration_sec=1.0,
                    )
                    d1 = self.last_d1
                    d2 = self.last_d2
                    d3 = self.last_d3
                    d4 = self.last_d4
                else:
                    self.last_d1 = d1
                    self.last_d2 = d2
                    self.last_d3 = d3
                    self.last_d4 = d4

                # Create DVL message
                dvl_msg = Dvl()
                dvl_msg.header.stamp = self.get_clock().now().to_msg()
                dvl_msg.header.frame_id = "dvl"

                # Set velocity (apply coordinate transformations if needed)
                dvl_msg.velocity.x = vx
                dvl_msg.velocity.y = -vy  # Transform coordinate system
                dvl_msg.velocity.z = -vz  # Transform coordinate system

                # Set environmental data
                dvl_msg.water_temperature = temperature

                # Create FluidPressure message for pressure
                pressure_msg = FluidPressure()
                pressure_msg.header.stamp = dvl_msg.header.stamp
                pressure_msg.header.frame_id = "dvl"
                pressure_msg.fluid_pressure = pressure
                pressure_msg.variance = 0.0  # Set appropriate variance if known
                dvl_msg.pressure = pressure_msg

                dvl_msg.figure_of_merit = fom

                # Set beam distances
                dvl_msg.beam_distances = [d1, d2, d3, d4]

                # Set additional data
                dvl_msg.battery_voltage = battery
                dvl_msg.speed_of_sound = speed_sound
                dvl_msg.status = status

                # Publish DVL message
                self.dvl_publisher.publish(dvl_msg)

                self.get_logger().debug(
                    f"DVL data parsed successfully - VX: {vx:.3f}, VY: {vy:.3f}, VZ: {vz:.3f}, FOM: {fom:.3f}"
                )

                # Output the extracted variables
                # print(f"VX: {vx}, VY: {vy}, VZ: {vz}")
                # print(f"D1: {d1}, D2: {d2}, D3: {d3}, D4: {d4}")
                # https://support.nortekgroup.com/hc/en-us/article_attachments/19558106638620
                # mode 358
            else:
                self.get_logger().debug(
                    f"DVL packet parsing failed - no regex matches found"
                )


def main(args=None):
    rclpy.init(args=args)
    node = DvlDriverNode()
    node.udp_server()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
