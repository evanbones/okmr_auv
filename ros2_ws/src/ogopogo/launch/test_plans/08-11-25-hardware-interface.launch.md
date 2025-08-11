# Hardware Interface Test Plan — 2025-08-11

This plan guides you through verifying the hardware interface nodes and testing motor commands on the AUV.

---

## 1. Launch & Verify Nodes

- **Launch** the corresponding launch file.
- **Check active nodes:**
  ```sh
  ros2 node list
  ```
  Ensure the following are running:
  - `realsense2_camera` nodes
  - `okmr_hardware_interface` nodes

---

## 2. Visualize in Foxglove

- On computer youre sshing from, open Foxglove at:
  ```
  http://<Jetson-IP>:8765
  ```

---

## 3. Motor Command Testing

### 3.1 Basic Communication Test

```sh
ros2 topic pub /ping std_msgs/msg/String "data: 'test_ping'"
```

### 3.2 Motor Throttle Control

**Reset/Off (All Throttles Zero):**
```sh
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

**Test Each Motor (Set One to 1600):**
```sh
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [1600, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 1600, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 1600, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 0.0, 1600, 0.0, 0.0, 0.0, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 0.0, 0.0, 1600, 0.0, 0.0, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 0.0, 0.0, 0.0, 1600, 0.0, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1600, 0.0]"
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle "throttle: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1600]"
```



---

### 3.3 Motor Thrust Control

**Stop (All Thrusts Zero):**
```sh
ros2 topic pub /motor_thrust okmr_msgs/msg/MotorThrust "thrust: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

**Forward Example (Modify as needed):**
```sh
ros2 topic pub /motor_thrust okmr_msgs/msg/MotorThrust "thrust: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]"
```

**Save Surge Sway Heave:**
```sh
ros2 topic pub /motor_thrust okmr_msgs/msg/MotorThrust "thrust: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

---

### 3.4 Battery Voltage Monitoring

(Future implementation)
```sh
ros2 topic echo /battery_voltage
```

---

## 4. I/O Reference

### Subscriptions (Receives)
1. **`okmr_msgs/msg/MotorThrottle`**
   - Topic: `/motor_throttle`
   - Controls the 8 thrusters.
   - Message:
     ```yaml
     float64[8] throttle    # Array of 8 throttle values (-1.0 to 1.0)
     std_msgs/Header header # Timestamp info
     ```
2. **`std_msgs/msg/String`**
   - Topic: `/ping`
   - Used for communication testing.

### Publications (Outputs)
1. **`okmr_msgs/msg/BatteryVoltage`**
   - Topic: `/battery_voltage`
   - Publishes battery status.

---
