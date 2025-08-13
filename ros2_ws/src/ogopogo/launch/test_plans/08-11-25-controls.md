# 08-11-25 controls test
1. launch corresponding launch file
```ros2 launch ogopogo pool_tests/08-11-25-controls.launch.py```
2. ensure all nodes are launched
    - dead reckoning
    - all control layers (pose, vel, accel, thrust alloc)
    - all hardware interface nodes
    - camera nodes

3. Ensure that the thrust allocator has the correct paramters loaded
4. send wrench commands to /wrench_target
5. check /motor_thrust and /motor_throttle to see if output makes sense

DEAD RECKONING
1. send enable command to dead reckoning node
ros2 service call /set_dead_reckoning_enabled okmr_msgs/SetDeadReckoningEnabled "{enable: true}"
2. verify no outlier velocity readings from dvl (-32 or similar)
3. plot in foxglove 
4. rotate and push sub, ensuring position makes sense and doesnt overly drift

VELOCITY + ACCEL PID TUNING
1. set all controller gains to 0
2. set control mode to VELOCITY
ros2 topic pub /control_mode okmr_msgs/ControlMode "{control_mode: 1}" --once
3. check that all systems correctly enabled

Per Axis

2. set a low velocity gain (0.5 - 2)
3. send velocity commands to /goal_velocity

ros2 topic pub /goal_velocity okmr_msgs/GoalVelocity "{integrate: false, duration: 5.0, twist: {linear: {x: 0.5}}}" --once
ros2 topic pub /goal_velocity okmr_msgs/GoalVelocity "{integrate: false, duration: 5.0, twist: {linear: {y: 0.5}}}" --once 
ros2 topic pub /goal_velocity okmr_msgs/GoalVelocity "{integrate: false, duration: 5.0, twist: {linear: {z: 0.5}}}" --once
ros2 topic pub /goal_velocity okmr_msgs/GoalVelocity "{integrate: false, duration: 5.0, twist: {angular: {x: 10.0}}}" --once
ros2 topic pub /goal_velocity okmr_msgs/GoalVelocity "{integrate: false, duration: 5.0, twist: {angular: {y: 10.0}}}" --once
ros2 topic pub /goal_velocity okmr_msgs/GoalVelocity "{integrate: false, duration: 5.0, twist: {angular: {z: 10.0}}}" --once

4. slowly increase kmass_? on the axis youre tuning, until the auv responds to the velocity command. 
    NOTE: the auv doesnt need to actually reach the desired speed, just need to start moving reasonably fast

5. slowly increase kdrag_? on the axis, until the auv is able to reach the desired velocity 

POSE + VELOCITY TUNING
1. enable pose control layer
ros2 topic pub /control_mode okmr_msgs/ControlMode "{control_mode: 0}" --once

2. slowly increase pose p gain, start around 0.1

3. if the response is sluggish, increase velocity p gain

