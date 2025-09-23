# UBC Okanagan Marine Robotics AUV Software

## Overview

This repository contains the software for UBC Okanagan Marine Robotic's AUV's.

ROS2 is heavily utilized for inter process communication, software configuration, and package management.

Other software libraries used are dependant on specific packages. eg.
- okmr_object_detection uses PyTorch and ONNX
- okmr_controls uses Eigen 
- okmr_mapping uses PCL

### Package Structure
The codebase is organized into different ros2 packages, with each one containing code related to a 
specific area of the system. For example:
- okmr_controls is for PID related code
- okmr_navigation is for navigation and movement coordination
- okmr_automated_planner is for the high level decision making state machines

Check the specific README.md files inside each package for more details

### Controls Hierarchy
The overall system is designed in a layered manner, with L5 systems making high level decisions,
and every system below that being a step in the chain that makes the L5 systems request a reality


Refresh / processing rate is the main factor that determines what layer a subsystem is in. The slower, the higher the layer. 

This also strongly defines what programming language the code should be written in. 

- L5 = Mission Plan (e.g., root state machaine in Automated Planner) ~30 second period
- L4 = Behavior Logic (specific task state machines, motion planner) ~5 second period
- L3 = Action Coordinators and Perception (navigator, mapper, object detection, system health) 5-20hz
- L2 = PID manager and Action Executors 20-200hz
- L1 = PID controllers ~200hz
- L0 = hardware I/O (motor/sensor interfaces) ~200hz

### System Diagram

![System Diagram](/diagrams/SystemDiagram.png)
