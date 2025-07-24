# Automated Planner using Nested State Machines

The automated planner uses the python transitions library for the state machine implementation

Docs and Repo:  https://github.com/pytransitions/transitions

Although there is a Hierarchal State Machine in the library, 
its features didnt fully match up with our needs

## naming conventions (PLEASE PLEASE PLEASE USE)

 - state names: doing_thing
 - linear transitions: doing_thing_done

## Code structure:
### state_machines
This folder contains all state machine implementations

### base_state_machine.py
This file defines the BaseStateMachine, the superclass to all state machines used in the package.

It contains all the common methods that the state machine implementations need

Run with --ros-args --log-level DEBUG --log-level rcl:=INFO to see all debug info (timers)
