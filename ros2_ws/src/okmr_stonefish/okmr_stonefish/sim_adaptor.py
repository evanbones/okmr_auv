#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import FluidPressure
from stonefish_ros2.msg import DVL
from okmr_msgs.msg import Dvl, MotorThrust


class SimAdaptor(Node):
    def __init__(self):
        super().__init__('sim_adaptor')
        
        # Publishers
        self.dvl_publisher = self.create_publisher(Dvl, '/dvl', 10)
        self.thruster_publisher = self.create_publisher(Float64MultiArray, '/stonefish/thruster_setpoints', 10)
        
        # Subscribers
        self.stonefish_dvl_sub = self.create_subscription(
            DVL, '/stonefish/dvl', self.dvl_callback, 10
        )
        self.stonefish_pressure_sub = self.create_subscription(
            FluidPressure, '/stonefish/pressure', self.pressure_callback, 10
        )
        self.motor_thrust_sub = self.create_subscription(
            MotorThrust, '/motor_thrust', self.motor_thrust_callback, 10
        )
        
        # Storage for pressure data to combine with DVL
        self.latest_pressure = None
        
        self.get_logger().info('SimAdaptor node started')

    def dvl_callback(self, msg):
        """Convert Stonefish DVL message to OKMR DVL message"""
        dvl_msg = Dvl()
        dvl_msg.header = msg.header
        
        # Copy velocity data
        dvl_msg.velocity = msg.velocity
        
        # Set simulated environmental data
        dvl_msg.water_temperature = 15.0  # Simulated water temperature in deg C
        
        # Use latest pressure data if available, convert from Pa to dBar
        if self.latest_pressure is not None:
            # Create pressure message with dBar conversion
            pressure_msg = FluidPressure()
            pressure_msg.header = self.latest_pressure.header
            # Convert from Pa to dBar (1 dBar = 10000 Pa)
            pressure_msg.fluid_pressure = self.latest_pressure.fluid_pressure / 10000.0
            pressure_msg.variance = self.latest_pressure.variance / (10000.0 * 10000.0)  # Convert variance accordingly
            dvl_msg.pressure = pressure_msg
        else:
            # Create default pressure message in dBar
            pressure_msg = FluidPressure()
            pressure_msg.header = msg.header
            pressure_msg.fluid_pressure = 10.1325  # Default atmospheric pressure in dBar (101325 Pa / 10000)
            pressure_msg.variance = 0.0
            dvl_msg.pressure = pressure_msg
        
        dvl_msg.figure_of_merit = 0.1  # Simulated figure of merit in m/s
        
        # Convert altitude to beam distances (simulated 4-beam DVL)
        beam_distance = float(msg.altitude) if msg.altitude > 0 else 0.0
        dvl_msg.beam_distances = [beam_distance, beam_distance, beam_distance, beam_distance]
        
        # Set simulated additional data
        dvl_msg.battery_voltage = 12.0  # Simulated battery voltage
        dvl_msg.speed_of_sound = 1500.0  # Typical speed of sound in water
        dvl_msg.status = 0  # OK status
        
        self.dvl_publisher.publish(dvl_msg)

    def pressure_callback(self, msg):
        """Store latest pressure data for DVL message"""
        self.latest_pressure = msg

    def motor_thrust_callback(self, msg):
        """Convert OKMR MotorThrust to Float64MultiArray for Stonefish thrusters"""
        thrust_array = Float64MultiArray()
        
        # Map OKMR motor names to Stonefish thruster order
        # Based on the thruster configuration in ogopogo.scn:
        # ThrusterFRO, ThrusterFLO, ThrusterBRO, ThrusterBLO, ThrusterFRI, ThrusterFLI, ThrusterBRI, ThrusterBLI
        thrust_array.data = [
            msg.fro,  # Front Right Outer
            msg.flo,  # Front Left Outer  
            msg.bro,  # Back Right Outer
            msg.blo,  # Back Left Outer
            msg.fri,  # Front Right Inner (Vertical)
            msg.fli,  # Front Left Inner (Vertical)
            msg.bri,  # Back Right Inner (Vertical)
            msg.bli   # Back Left Inner (Vertical)
        ]
        
        self.thruster_publisher.publish(thrust_array)
        self.get_logger().debug(f'Published thruster setpoints: {thrust_array.data}')



def main(args=None):
    rclpy.init(args=args)
    sim_adaptor = SimAdaptor()
    
    try:
        rclpy.spin(sim_adaptor)
    except KeyboardInterrupt:
        pass
    finally:
        sim_adaptor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
