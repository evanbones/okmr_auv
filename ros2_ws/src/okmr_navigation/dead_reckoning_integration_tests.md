# Dead Reckoning Node - Automated Integration Test Plan

## Overview
This document describes comprehensive automated integration tests for the dead reckoning node to verify sensor fusion, state estimation, and control functionality.

## Test Environment Setup
- **Test Framework**: Python-based ROS2 integration test using `launch_testing`
- **Duration**: Each test should run for 10-30 seconds with synthetic data
- **Data Sources**: Simulated IMU and DVL data with known ground truth
- **Verification**: Compare published outputs against expected values within tolerance

---

## Test Suite 1: Basic Functionality Tests

### Test 1.1: Node Initialization and Topics
**Objective**: Verify node starts correctly and publishes expected topics

**Test Steps**:
1. Launch dead reckoning node with default parameters
2. Wait 2 seconds for initialization
3. Verify all expected topics are published:
   - `/pose` (geometry_msgs/PoseStamped)
   - `/velocity` (geometry_msgs/Twist) 
   - `/acceleration` (geometry_msgs/Accel)
4. Verify all services are available:
   - `/set_dead_reckoning_enabled`
   - `/clear_pose`
   - `get_pose_twist_accel`

**Expected Results**: All topics published at ~200Hz, all services responsive

### Test 1.2: Parameter Validation
**Objective**: Verify parameter descriptors and range validation

**Test Steps**:
1. Query all parameter descriptors using `ros2 param describe`
2. Attempt to set invalid parameter values (e.g., negative frequency, alpha > 1.0)
3. Verify parameters reject invalid values
4. Set valid parameter values and confirm they're applied

**Expected Results**: 
- All parameters have descriptive text and valid ranges
- Invalid values rejected with appropriate error messages
- Valid parameter changes applied immediately

---

## Test Suite 2: Sensor Data Processing Tests

### Test 2.1: IMU Data Processing
**Objective**: Test IMU coordinate transforms and attitude estimation

**Test Synthetic Data**:
- **Static Case**: IMU at rest (only gravity acceleration, zero angular velocity)
- **Rotation Case**: Pure yaw rotation at 10°/sec for 5 seconds
- **Tilt Case**: Static 30° roll, 45° pitch orientations

**Test Steps**:
1. Inject synthetic IMU data with known orientations
2. Disable dead reckoning (integration off)
3. Monitor `/velocity` angular components and `/acceleration` outputs
4. Verify coordinate frame transformations (D455 → ROS2)
5. Check gravity compensation in linear acceleration

**Expected Results**:
- **Static**: Roll/pitch converge to gravity-derived angles, zero angular velocity
- **Rotation**: Angular velocity ~10°/sec in yaw, attitude integrates correctly
- **Tilt**: Linear acceleration shows gravity compensation working

### Test 2.2: DVL Data Processing  
**Objective**: Test DVL velocity integration and acceleration calculation

**Test Synthetic Data**:
- **Constant Velocity**: 1.0 m/s forward for 10 seconds
- **Acceleration**: Linear velocity ramp from 0 to 2.0 m/s over 5 seconds
- **8Hz Timing**: DVL data at realistic 8Hz update rate

**Test Steps**:
1. Inject synthetic DVL data with known velocities
2. Monitor `/velocity` linear components
3. Monitor `/acceleration` for DVL-derived acceleration
4. Verify DVL acceleration calculated using proper DVL timing (not update loop timing)

**Expected Results**:
- **Constant**: Velocity converges to 1.0 m/s, acceleration near zero
- **Acceleration**: Velocity ramps correctly, acceleration ~0.4 m/s²
- **Timing**: DVL acceleration updates at 8Hz, not 200Hz

---

## Test Suite 3: Sensor Fusion Tests

### Test 3.1: DVL-IMU Velocity Fusion
**Objective**: Test complementary filtering between DVL and IMU

**Test Scenario**: 
- DVL reports constant 1.0 m/s forward velocity
- IMU reports 0.5 m/s² forward acceleration (should integrate to velocity)
- Run for 10 seconds with dead reckoning enabled

**Test Steps**:
1. Start with DVL velocity = 1.0 m/s, IMU accel = 0.5 m/s²
2. Monitor velocity convergence and integration
3. Test with different `dvl_velocity_alpha` values (0.5, 0.9, 0.99)
4. Verify high-frequency velocity updates between 8Hz DVL measurements

**Expected Results**:
- Velocity should blend DVL measurements with IMU integration
- Higher alpha → more DVL influence, lower alpha → more IMU influence
- Velocity published at 200Hz even with 8Hz DVL

### Test 3.2: DVL-IMU Acceleration Fusion
**Objective**: Test acceleration complementary filtering

**Test Scenario**:
- DVL velocity jumps (creating DVL-derived acceleration)  
- IMU reports different acceleration
- Compare published acceleration blend

**Test Steps**:
1. Send DVL velocity step change (0 → 2 m/s over 0.125s = 16 m/s²)
2. Send conflicting IMU acceleration (8 m/s²)
3. Monitor published `/acceleration` with different `dvl_accel_alpha` values
4. Verify fusion mathematics: `accel = alpha*dvl_accel + (1-alpha)*imu_accel`

**Expected Results**:
- With alpha=0.3: acceleration ≈ 0.3×16 + 0.7×8 = 10.4 m/s²
- Acceleration blending follows complementary filter equation

---

## Test Suite 4: Integration and Dead Reckoning Tests

### Test 4.1: Pose Integration Logic
**Objective**: Test dead reckoning pose integration when enabled/disabled

**Test Scenario**: Vehicle moves in known pattern with dead reckoning enabled

**Test Steps**:
1. Enable dead reckoning via service call
2. Inject DVL velocity: 1.0 m/s forward for 5 seconds
3. Monitor `/pose` position integration  
4. Clear pose via service, verify reset to origin
5. Disable dead reckoning, continue velocity → verify no further pose integration
6. Re-enable dead reckoning → verify integration resumes

**Expected Results**:
- **Enabled**: Position integrates to ~5.0m forward after 5 seconds
- **Clear**: Position resets to (0,0,0)
- **Disabled**: Position stops integrating despite continued velocity
- **Re-enabled**: Integration resumes from current position

### Test 4.2: Coordinate Frame Transformations
**Objective**: Test velocity rotation from body to world frame

**Test Scenario**: Vehicle rotated 90° yaw, then moves forward in body frame

**Test Steps**:
1. Set attitude to 90° yaw rotation (facing sideways)
2. Inject body-frame velocity: 1.0 m/s forward
3. Monitor world-frame position integration
4. Expected: vehicle moves sideways in world frame

**Expected Results**:
- Body velocity rotated correctly to world frame
- Position integrates in world Y-direction (not X-direction)

---

## Test Suite 5: Edge Cases and Robustness Tests

### Test 5.1: Missing Sensor Data
**Objective**: Test behavior with missing or stale sensor data

**Test Steps**:
1. Start with normal IMU+DVL data
2. Stop DVL data (simulate sensor failure)
3. Verify node continues with IMU-only operation
4. Stop IMU data → verify graceful degradation
5. Resume sensor data → verify recovery

**Expected Results**:
- **DVL loss**: Velocity integration continues with IMU, no DVL acceleration
- **IMU loss**: Attitude estimation stops, but DVL velocity still used
- **Recovery**: Normal operation resumes when sensors return

### Test 5.2: High Dynamic Conditions  
**Objective**: Test filter stability with aggressive maneuvers

**Test Synthetic Data**:
- High angular rates (>180°/sec)
- High accelerations (>5g)
- Rapid velocity changes

**Test Steps**:
1. Inject high-dynamic synthetic data
2. Monitor for filter instability or divergence
3. Verify outputs remain bounded and reasonable
4. Check complementary filter doesn't produce oscillations

**Expected Results**:
- Filters remain stable under high dynamics
- No unbounded growth or oscillations
- Attitude estimation switches to gyro-only mode when appropriate

### Test 5.3: Parameter Sensitivity Analysis
**Objective**: Test system behavior across parameter ranges

**Test Steps**:
1. Run standard test scenario with varying parameters:
   - `complementary_filter_alpha`: 0.8, 0.95, 0.999
   - `dvl_velocity_alpha`: 0.1, 0.5, 0.95
   - `update_frequency`: 50Hz, 200Hz, 500Hz
2. Measure convergence time, stability, noise characteristics
3. Verify system remains functional across parameter space

**Expected Results**:
- System functional across all tested parameter ranges
- Performance trades documented (e.g., smoothing vs responsiveness)

---

## Test Suite 6: Service Interface Tests

### Test 6.1: Dead Reckoning Enable/Disable Service
**Objective**: Test dead reckoning control service

**Test Steps**:
1. Call `/set_dead_reckoning_enabled` with `enable: false`
2. Verify service response indicates success
3. Inject motion data → verify no pose integration
4. Call with `enable: true` → verify integration resumes
5. Test service with invalid requests

**Expected Results**:
- Service calls succeed with appropriate response messages
- Dead reckoning behavior changes immediately
- Invalid requests handled gracefully

### Test 6.2: Pose Clear Service
**Objective**: Test pose reset functionality

**Test Steps**:
1. Accumulate some pose through integration
2. Call `/clear_pose` service
3. Verify pose resets to origin (0,0,0)
4. Verify integration continues from new origin
5. Test service response messages

**Expected Results**:
- Pose immediately resets to origin
- Integration continues normally from reset point
- Service provides clear success/failure feedback

---

## Test Implementation Notes

### Test Data Generation
```python
# Example synthetic IMU data generation
def generate_imu_data(duration, frequency, angular_vel, linear_accel):
    # Generate timestamped IMU messages with known values
    # Include D455 coordinate frame transformations
    pass

# Example synthetic DVL data generation  
def generate_dvl_data(duration, frequency, velocity_profile):
    # Generate DVL messages with realistic 8Hz timing
    # Include all DVL fields (temperature, pressure, etc.)
    pass
```

### Verification Tolerances
- **Position**: ±0.1m for integration tests
- **Velocity**: ±0.05 m/s for steady-state
- **Acceleration**: ±0.5 m/s² for fusion tests  
- **Attitude**: ±2° for complementary filter
- **Timing**: ±10ms for frequency verification

### Test Automation Framework
- Use `launch_testing` for full ROS2 integration
- Implement custom test nodes for data injection
- Record bag files for debugging failed tests
- Generate test reports with pass/fail statistics
- Integration with CI/CD pipeline for regression testing

---

## Expected Test Duration
- **Full test suite**: ~15 minutes
- **Quick smoke test**: ~3 minutes (basic functionality only)
- **Parameterized sweep**: ~1 hour (multiple parameter combinations)

This comprehensive test plan ensures the dead reckoning node functions correctly across all operational scenarios and maintains robustness under real-world conditions.