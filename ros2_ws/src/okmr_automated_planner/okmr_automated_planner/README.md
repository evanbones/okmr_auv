# Automated Planner using Nested State Machines

The automated planner uses the python transitions library for the state machine implementation

Docs and Repo:  https://github.com/pytransitions/transitions

Although there is a Hierarchal State Machine in the library, 
its features didnt fully match up with our needs

## testing instructions
The automated planner interfaces with the okmr_navigation navigator_action_server to send movement requests, 
so it must be launched to test movement commands. 

The navigator_action_server has a test mode which allows you to send
movement commands that dont actually use the control systems, and instead just wait for MovementCommand.timeout_sec seconds.
(See okmr_msgs/msg/MovementCommand.msg) for more details

The launch file inside this package called "test_automated_planner.launch.py" allows you to 
test state machines with movement commands in isolation, so that you dont need to launch a simulation + the control systems
to test your code.

Use the following command (replacing the root_config:=xyzabc portion) to launch the state machine you are testing

This will automatically call the initialize() method on your state machine, and the on_enter_initializing() method will be
called as a result

``` bash
ros2 launch okmr_automated_planner test_automated_planner.launch.py root_config:='task_state_machines/finding_gate.yaml'
```

## naming conventions

 - state names: doing_thing
 - linear transitions: doing_thing_done

## Code structure:
### state_machines
This folder contains all state machine implementations

### base_state_machine.py
This file defines the BaseStateMachine, the superclass to all state machines used in the package.

It contains all the common methods that the state machine implementations need

