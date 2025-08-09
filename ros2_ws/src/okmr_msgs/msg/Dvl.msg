std_msgs/Header header

# DVL velocity measurements
geometry_msgs/Vector3 velocity  # VX, VY, VZ speeds in m/s

# Environmental measurements
float32 water_temperature       # Water temperature in deg C
sensor_msgs/FluidPressure pressure  # Pressure measurement
float32 figure_of_merit        # Figure of Merit in m/s

# Beam distance measurements (4 beams)
float32[4] beam_distances      # D1, D2, D3, D4 - Vertical distance to bottom in meters

# Additional data
float32 battery_voltage        # Battery voltage in V
float32 speed_of_sound        # Speed of sound in water in m/s
uint32 status                 # Status bits
