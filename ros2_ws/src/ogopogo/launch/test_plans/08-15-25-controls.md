# 08-11-25 controls test
1. launch corresponding launch file
ros2 launch ogopogo pool_tests/08-11-25-controls.launch.py

ros2 run crl_parameter_gui parameter_server

ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1600.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1600.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1500.0, 1600.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1500.0, 1500.0, 1600.0, 1500.0, 1500.0, 1500.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1500.0, 1500.0, 1500.0, 1600.0, 1500.0, 1500.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1600.0, 1500.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1600.0, 1500.0]}" -r 100
ros2 topic pub /motor_throttle okmr_msgs/MotorThrottle "{throttle: [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1600.0]}" -r 100

ros2 action send_goal /movement_command okmr_msgs/action/Movement "{movement_command: {command: 1, translation: {x: 2.0}}}"
ros2 action send_goal /movement_command okmr_msgs/action/Movement "{movement_command: {command: 1, translation: {y: 2.0}}}"
ros2 action send_goal /movement_command okmr_msgs/action/Movement "{movement_command: {command: 1, translation: {z: -1.0}}}"

ros2 action send_goal /movement_command okmr_msgs/action/Movement "{movement_command: {command: 7, timeout_sec: 6.0}}"

