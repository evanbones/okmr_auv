# OKMR 2025 AUV Software Application Challenge

Welcome to the Okanagan Marine Robotics (OKMR) software application challenge! 
This challenge will help you get familiar with ROS2 and our software ecosystem.

If you have Linux or ROS2 experience, this should take ~30-60 minutes.

Otherwise, you can **expect to spend ~2-4 hours to get everything up and running**.

> This challenge intentionally does not provide every step in detail. You are expected to troubleshoot issues and fill in gaps using documentation, AI tools, and your problem-solving skills.

> However, no prior experience with Linux or ROS is required. We designed this challenge to be achievable for anyone with a strong motivation to learn, as we care more about your ability to troubleshoot and use documentation than what you already know.

## Prerequisites

Before starting, ensure you have access to a system that can run Ubuntu 24.04 or 25.04. This can be:
- Native Ubuntu installation
- Virtual Machine (VM)
    - on macOS UTM works well: [UTM VM Link](https://mac.getutm.app/)
- Windows Subsystem for Linux (WSL)
    - highly recommended if you are using Windows
- Docker container
- Remote server access


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

1. In one terminal, publish a message:
    ```bash
   ros2 topic pub /okmr_application std_msgs/msg/String "{data: '{YOUR NAME} is locked in on okmr software team'}"
   ```

2. In another terminal, start a listener:
   ```bash
   ros2 topic echo /okmr_application std_msgs/msg/String
   ```

3. Take a screenshot showing both terminals with the message being published and received.

### 3. Work with OKMR Messages

1. Clone the entire okmr_auv repository 
2. cd into the ros2_ws folder 
3. compile the `okmr_msgs` package 
    - ```colcon build --packages-select okmr_msgs```
    - Ensure you source the workspace afterwards using: ```source install/setup.bash```
4. Repeat the pub/sub example using an OKMR message type of your choice
    - [Link to message definitions](/ros2_ws/src/okmr_msgs/msg/)
5. Take a screenshot of the working example

## Submission

Complete the Google Form with the following:

### Screenshots Required
- Screenshot of basic ROS2 pub/sub example
- Screenshot of OKMR messages pub/sub example

### Knowledge Assessment Questions
Answer these questions in the Google Form:

**Basic ROS2 Knowledge:**
- What is ROS2? (~100 words or 2-4 sentences is enough) 

**OKMR System Understanding:**
Read the main project [README](/README.md) and very briefly answer:
- How does OKMR use ROS2?
- What are some examples of software packages used in the OKMR AUV system?
- Summarize the general structure of the software system based on the system diagram at the bottom of the [README](/README.md)

Good luck! If you need an extension on the challenge (for a good reason), please reach out to OKMR via email or instagram.

