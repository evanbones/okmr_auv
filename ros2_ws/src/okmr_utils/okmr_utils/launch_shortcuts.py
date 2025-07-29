from launch.actions import SetEnvironmentVariable

debug_ros_args = [
    "--log-level",
    "debug",
    "--log-level",
    "rcl:=warn",  # Suppress noisy RCL debug messages
    "--log-level",
    "rcl_action:=warn",  # Suppress RCL action client messages
    "--log-level",
    "rmw_fastrtps_cpp:=warn",
]

color_output = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")
