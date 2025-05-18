# Marine Design Software

## Overview

This repository contains the software for Okanagan Marine Robotic's AUV's

### Install Instructions
``` bash
git clone --recurse-submodules https://github.com/Okanagan-Marine-Robotics/okmr_auv.git
cd okmr_auv/ros2_ws
colcon build --packages-select okmr_msgs
source install/setup.bash
TODO CREATE DEPENDANCY INSTALLING SCRIPT
colcon build
```
All documentation can be found [here](https://docs.google.com/document/d/1PhQ2q0ED-8mXD5I1RwiWC_7xKGJy2q97WKZ9-5tj8FE/edit?usp=sharing)

Current tasks can be found [here](https://docs.google.com/spreadsheets/d/1-QFCbOIapLfAIvq882iY_Cj5Way3pCkZ1IorhzPteBw/edit?usp=sharing)
![ROS Graph](/diagrams/IMG_6097.png)
