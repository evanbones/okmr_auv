# okmr_controls_old

The okmr_controls_old package is a legacy implementation of the okmr AUV control systems. 

## Purpose
Legacy nodes for AUV controls systems, primarily 
[PID controllers](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
and very basic thrust allocation (PIDCombiner node).

Not to be used for actual control systems implementation anymore, only for reference.

In 2023/24, the PIDController and PIDCombiner nodes were used for a basic cascading PID based control system, 
with two nodes per axis, one for relative pose -> velocity control, and one for velocity -> motor throttle (%).

## Nodes

### PIDController
Implements a basic
[PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
with ROS2 parameters that can be loaded at startup (i.e. they are NOT live configurable)

#### Inputs
type: okmr_msgs/msg/SensorReading (Deprecated)

topics: /PID/XXX/target, /PID/XXX/actual

Uses the target and actual values to calculate the error term used in the PID controller.

#### Ouputs
type: okmr_msgs/msg/SensorReading (Deprecated)
topics: /PID_correction/XXX

Output of the PID controller

### PIDCombiner
Implements a VERY basic thrust allocator.
Reference [this report](https://publications-cnrc.canada.ca/eng/view/ft/?id=43560a68-dee6-4a39-9a91-dfd570c19654)
for more information on what a thrust allocator does.

Essentially takes in the PID output from the velocity control layer and turns it into actual motor throttles.
The calculation is very hardcoded, and assumes that the thrusters perfectly cancel each other out (i.e. they do not 

#### Inputs
type: okmr_msgs/msg/SensorReading (Deprecated)

topics: /PID_correction/(yaw, pitch, roll, surge, sway, heave)

Subscribes to the PID outputs of every axis

#### Outputs
type: okmr_msgs/msg/MotorThrottle

topics: /motor_throttle

Output motor throttles, intended to be used by whatever hardware interface you're using.

## Future work
None, this package is deprecated / legacy.

