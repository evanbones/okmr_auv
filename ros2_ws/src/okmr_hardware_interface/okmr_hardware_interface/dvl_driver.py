#!/usr/bin/env python3

import socket
import re
import rclpy
from rclpy.node import Node
from okmr_msgs.msg import Dvl
from geometry_msgs.msg import Vector3


class DvlDriverNode(Node):
    def __init__(self):
        super().__init__("dvl_driver")
        self.dvl_publisher = self.create_publisher(Dvl, "/dvl", 10)

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
            pattern = r"TIME=(-?\d+\.?\d*),.*?VX=(-?\d+\.\d+),VY=(-?\d+\.\d+),VZ=(-?\d+\.\d+),FOM=(-?\d+\.\d+),D1=(-?\d+\.\d+),D2=(-?\d+\.\d+),D3=(-?\d+\.\d+),D4=(-?\d+\.\d+),BATT=(-?\d+\.\d+),SS=(-?\d+\.\d+),PRESS=(-?\d+\.\d+),TEMP=(-?\d+\.\d+),STAT=(0x[0-9A-Fa-f]+)"

            # Find the matches
            matches = re.search(pattern, data)

            if matches:
                # Extracting values from regex groups
                time_val = float(matches.group(1))
                vx = float(matches.group(2))
                vy = float(matches.group(3))
                vz = float(matches.group(4))
                fom = float(matches.group(5))
                d1 = float(matches.group(6))
                d2 = float(matches.group(7))
                d3 = float(matches.group(8))
                d4 = float(matches.group(9))
                battery = float(matches.group(10))
                speed_sound = float(matches.group(11))
                pressure = float(matches.group(12))
                temperature = float(matches.group(13))
                status = int(matches.group(14), 16)  # Convert hex string to int

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
                dvl_msg.pressure = pressure
                dvl_msg.figure_of_merit = fom

                # Set beam distances
                dvl_msg.beam_distances = [d1, d2, d3, d4]

                # Set additional data
                dvl_msg.battery_voltage = battery
                dvl_msg.speed_of_sound = speed_sound
                dvl_msg.status = status

                # Publish DVL message
                self.dvl_publisher.publish(dvl_msg)

                # Output the extracted variables
                # print(f"VX: {vx}, VY: {vy}, VZ: {vz}")
                # print(f"D1: {d1}, D2: {d2}, D3: {d3}, D4: {d4}")
                # https://support.nortekgroup.com/hc/en-us/article_attachments/19558106638620
                # mode 358
            else:
                print("No matches found.")


def main(args=None):
    rclpy.init(args=args)
    node = DvlDriverNode()
    node.udp_server()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
