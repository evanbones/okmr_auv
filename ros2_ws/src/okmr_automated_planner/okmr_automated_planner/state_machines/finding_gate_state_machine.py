from okmr_msgs.msg import MovementCommand
from okmr_msgs.srv import Status
from geometry_msgs.msg import Vector3
# TODO: Replace with actual BoundingBox message when available
from std_msgs.msg import String as BoundingBox  # Placeholder

from okmr_automated_planner.base_state_machine import BaseStateMachine

class FindingGateStateMachine(BaseStateMachine):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # Gate detection tracking
        self.confidence_threshold = 0.8  # Confidence threshold for gate detection
        self.validation_frame_threshold = 10  # Frames needed to validate gate found
        self.high_confidence_frame_count = 0
        self.last_scanning_direction = 'CW'  # Track last scanning direction
        self.scans_completed = 0
        self.max_scan_attempts = 3
        self.cached_gate_bounding_box = None
        
        # Subscribe to gate detection topic
        self.gate_detection_subscription = self.ros_node.create_subscription(
            BoundingBox,
            '/gate_detection',
            self.gate_detection_callback,
            10
        )
        self._subscriptions.append(self.gate_detection_subscription)

    def gate_detection_callback(self, msg):
        """Callback for gate detection messages"""
        # TODO: Replace with actual BoundingBox message parsing when available
        # For now, simulate confidence as 1.0
        confidence = 1.0
        self.cached_gate_bounding_box = msg
        
        if confidence > self.confidence_threshold:
            self.high_confidence_frame_count += 1
            
            # If currently scanning, initiate look at gate sequence
            if self.is_scanningCW() or self.is_scanningCCW():
                self.ros_node.get_logger().info(f"High confidence gate detected (confidence: {confidence:.2f}), initiating look at gate")
                self.lookAtGate()

    def on_enter_initializing(self):
        # start up object detection model
        # on request success, initializingDone
        self.queued_method = self.initializingDone
        pass

    def on_enter_initialized(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SPIN
        movement_msg.goal_velocity = GoalVelocity()

        speed = 30.0 #30 degree / sec on yaw axis
        movement_msg.goal_velocity.twist.angular.z = speed
        movement_msg.goal_velocity.duration = 360.0 / speed
        movement_msg.goal_velocity.integrate = True
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.doneScan,
            on_failure=self.handle_movement_failure
            on_acceptance=self.exitInitialized
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send scanning CW movement command")
            self.abort()

    def done_scan(self):
        self.scans_completed += 1
        if self.scans_completed >= self.max_scan_attempts:
            self.abort()

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SPIN
        movement_msg.goal_velocity = GoalVelocity()
    
        acceptance_callback = None
        if self.last_scanning_direction == 'CW':
            speed = -30.0 #-30 degree / sec on yaw axis
            acceptance_callback = self.scanningCWDone
        else:
            speed = 30.0
            acceptance_callback = self.scanningCCWDone

        movement_msg.goal_velocity.twist.angular.z = speed
        movement_msg.goal_velocity.duration = 360.0 / abs(speed)
        movement_msg.goal_velocity.integrate = True
        
        success = self.movement_client.send_movement_command(
                movement_msg,
                on_success=self.doneScan, #once the movement succeeds, call this method again
                on_failure=self.handle_movement_failure
                on_acceptance=acceptance_callback
            )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send scanning CW movement command")
            self.abort()
        
    def on_enter_scanningCW(self):
        self.last_scanning_direction = 'CW'

    def on_enter_scanningCCW(self):
        self.last_scanning_direction = 'CCW'
    
    def on_enter_initializeLookingAtGate(self):
        """Initialize looking at gate, sets up movement"""
        #TODO use self.cached_gate_bounding_box to calculate where to look
        #ex. if FOV of camera is 90 deg, and center is on the far edges
        #we will want to do a relative yaw rotation of ~40 degrees
        
        calculated_yaw_rotation = 20.0

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation = Vector3(x=0.0, y=0.0, z=0.0)
        movement_msg.rotation = Vector3(x=0.0, y=0.0, z=calculated_yaw_rotation)  # Small rotation to center
        movement_msg.duration = 15.0 #generous time estimate to rotate
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.validateGateFound,
            on_failure=self.handle_movement_failure
            on_accepted=self.initializeLookingAtGateDone
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send look at gate movement command")
            self.abort()

    def validateGateFound(self):
        """Validate if we have enough high confidence frames to confirm gate"""
        
        if self.high_confidence_frame_count >= self.validation_frame_threshold:
            self.gateFound()
        else:
            self.ros_node.get_logger().warn(f"Gate validation failed. Only {self.high_confidence_frame_count} frames received, but {self.validation_frame_threshold} needed to continue")
            # Resume scanning in opposite direction from last scan
            if self.last_scanning_direction == 'CW':
                self.resumingScanningCCW()
            else:
                self.resumingScanningCW()

    def on_enter_integratingGateData(self):
        """Integrate gate data with mapping system"""
        # TODO: Send mapping data integration request
        self._send_mapping_integration_request()
        # For now, immediately complete
        self.queued_method = self.integratingGateDataDone

    def _send_mapping_integration_request(self):
        self.ros_node.get_logger().warn("TODO IMPLEMENT: Sending mapping data integration request")
    
    def on_completion(self):
        # disable object detection
        self.ros_node.get_logger().info("FindingGate state machine completed successfully")
        pass

    def handle_movement_failure(self):
        """Handle movement action failure"""
        self.ros_node.get_logger().error("Movement action failed")
        self.abort()

