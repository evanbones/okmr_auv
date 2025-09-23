# OKMR 2025 Software Application Challenge

Welcome to the Okanagan Marine Robotics (OKMR) software application challenge! 
This challenge will help you get familiar with ROS2 and our software ecosystem.

If you have Linux or ROS2 expereince, this should take ~30-60 minutes, otherwise,
you can expect to spend ~2-4 hours to get everything up and running.

## Prerequisites

Before starting, ensure you have access to a system that can run Ubuntu 24.04 or 25.04. This can be:
- Native Ubuntu installation
- Virtual Machine (VM)
    - on macOS UTM works well: [UTM VM Link](https://mac.getutm.app/)
- Windows Subsystem for Linux (WSL)
    - highly recommended if you are using Windows
- Docker container
- Remote server access

**Feel free to use AI or your tutorial of choice to get this working.**

## Challenge Tasks

### 1. Install ROS2

Install ROS2 Jazzy or Kilted on your chosen system.

**Installation Guide:** Follow the official 
[ROS2 Jazzy Jalisco Installation Tutorial](https://docs.ros.org/en/jazzy/Installation.html) 
or 
[ROS2 Kilted Kaiju Installation Tutorial](https://docs.ros.org/en/kilted/Installation.html)
based on the version of Ubuntu you installed. **(ubuntu 24 = jazzy, ubuntu 25 = kilted)**.

> **Note:** Choose the installation method that best fits your system and experience level. The goal is to get ROS2 running, regardless of the specific setup method. Generally, installing Ubuntu on a VM or through WSL will be the easiest to work with.

### 2. Basic ROS2 Pub/Sub Example

1. In one terminal, start a listener:
   ```bash
   ros2 topic echo /okmr_application std_msgs/msg/String
   ```

2. In another terminal, publish a message:
   ```bash
   ros2 topic pub /okmr_application std_msgs/msg/String "{data: '{YOUR NAME} is locked in on okmr software team'}"
   ```

3. Take a screenshot showing both terminals with the message being published and received.

### 3. Work with OKMR Messages

1. Clone the entire okmr_auv repo 
2. cd into the ros2_ws folder 
3. compile the `okmr_msgs` package 
    - ```colcon build --packages-select okmr_msgs```
    - Ensure you source the workspace afterwards using: ```source install/setup.bash```
4. Repeat the pub/sub example using an OKMR message type of your choice
    - [/ros2_ws/src/okmr_msgs/msg/]
5. Take a screenshot of the working example


## Submission

Complete the Google Form with the following:

### Screenshots Required
- Screenshot of basic ROS2 pub/sub example
- Screenshot of OKMR messages pub/sub example

### Knowledge Assessment Questions
Answer these questions in the Google Form:

**Basic ROS2 Knowledge:**
- What is ROS2? (Research using Google or AI tools)

**OKMR System Understanding:**
Read the main project README.md and answer:
- How does OKMR use ROS2?
- What are some examples of software packages used in the OKMR AUV system?
- Summarize the general structure of the software system based on your understanding

**Main README:** [/README.md]

Good luck!

