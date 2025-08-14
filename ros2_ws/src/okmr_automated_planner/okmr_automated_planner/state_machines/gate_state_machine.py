from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled, GetObjectsByClass
from okmr_msgs.msg import MovementCommand, MissionCommand, DetectedObject


class GateStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "scan_duration",
            "value": 10.0,
            "descriptor": "Time to scan for gates before querying object_locator",
        },
        {
            "name": "approach_distance",
            "value": 3.0,
            "descriptor": "Distance to maintain from gate when locked on",
        },
        {
            "name": "scan_yaw_rate",
            "value": 0.3,
            "descriptor": "Yaw rate during scanning (rad/s)",
        }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.scan_duration = self.get_local_parameter("scan_duration")
        self.approach_distance = self.get_local_parameter("approach_distance")
        self.scan_yaw_rate = self.get_local_parameter("scan_yaw_rate")
        self.target_gate = None

    def on_enter_initializing(self):
        # Initialize and transition to waiting for mission start
        self.queued_method = self.initialized

    def mission_command_callback(self, msg):
        """Handle incoming mission command messages"""
        if msg.command == MissionCommand.START_MISSION:
            if self.is_waiting_for_mission_start():
                self.ros_node.get_logger().info("Mission start command received")
                self.mission_start_received()
            else:
                self.ros_node.get_logger().warn(
                    "Mission start command received but a mission is already running"
                )
        elif msg.command == MissionCommand.KILL_MISSION:
            self.ros_node.get_logger().warn("Mission kill command received")
            self.abort()

    def on_enter_waiting_for_mission_start(self):
        """Wait for subscription to /mission_command topic"""
        self.ros_node.get_logger().info("Waiting for mission start command...")
        self.add_subscription(
            MissionCommand, "/mission_command", self.mission_command_callback
        )

    def check_subsystem_enable_success(self, future):
        """Check response from dead reckoning service"""
        try:
            response = future.result()
            if response.success:
                self.ros_node.get_logger().info(
                    f"Dead reckoning enabled successfully: {response.message}"
                )
                self.enabling_subsystems_done()
            else:
                self.ros_node.get_logger().error(
                    f"Failed to enable dead reckoning: {response.message}"
                )
                self.abort()
        except Exception as e:
            self.ros_node.get_logger().error(f"Dead reckoning service call failed: {e}")
            self.abort()

    def on_enter_enabling_subsystems(self):
        # Enable dead reckoning
        request = SetDeadReckoningEnabled.Request()
        request.enable = True

        self.send_service_request(
            SetDeadReckoningEnabled,
            "/set_dead_reckoning_enabled",
            request,
            self.check_subsystem_enable_success,
        )

    def on_enter_scanning_for_gate(self):
        """Start scanning movement to allow object_locator to build map"""
        self.ros_node.get_logger().info("Starting gate scanning...")
        
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SCAN_YAW
        movement_msg.yaw_rate = self.scan_yaw_rate
        movement_msg.timeout_sec = self.scan_duration

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanning_complete,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send scanning movement command")
            self.queued_method = self.abort

    def scanning_complete(self):
        """Called when scanning timeout is reached - query object_locator for gates"""
        self.ros_node.get_logger().info("Scanning complete, querying for gates...")
        
        request = GetObjectsByClass.Request()
        request.object_class = DetectedObject.GATE

        self.send_service_request(
            GetObjectsByClass,
            "/get_objects_by_class",
            request,
            self.handle_gate_query_response,
        )

    def handle_gate_query_response(self, future):
        """Process response from object_locator service"""
        try:
            response = future.result()
            if len(response.objects) > 0:
                # Found gates - select the closest one
                closest_gate = min(response.objects, 
                                 key=lambda obj: (obj.pose.x**2 + obj.pose.y**2 + obj.pose.z**2)**0.5)
                
                self.target_gate = closest_gate
                self.ros_node.get_logger().info(
                    f"Gate found at position: x={closest_gate.pose.x:.2f}, "
                    f"y={closest_gate.pose.y:.2f}, z={closest_gate.pose.z:.2f}"
                )
                self.gate_found()
            else:
                self.ros_node.get_logger().warn("No gates detected during scan")
                self.abort()
        except Exception as e:
            self.ros_node.get_logger().error(f"Gate query service call failed: {e}")
            self.abort()

    def on_enter_gate_locked(self):
        """Gate is locked - calculate approach vector and prepare for approach"""
        if self.target_gate is None:
            self.ros_node.get_logger().error("No target gate available")
            self.abort()
            return

        gate_pos = self.target_gate.pose
        self.ros_node.get_logger().info(
            f"Gate locked at: x={gate_pos.x:.2f}, y={gate_pos.y:.2f}, z={gate_pos.z:.2f}"
        )
        
        # Calculate distance to gate
        distance = (gate_pos.x**2 + gate_pos.y**2 + gate_pos.z**2)**0.5
        self.ros_node.get_logger().info(f"Distance to gate: {distance:.2f}m")
        
        # Immediately proceed to approach
        self.queued_method = self.gate_locked_done

    def on_enter_approaching_gate(self):
        """Move towards the detected gate"""
        if self.target_gate is None:
            self.ros_node.get_logger().error("No target gate for approach")
            self.abort()
            return

        gate_pos = self.target_gate.pose
        
        # Calculate approach position (stop at approach_distance from gate)
        distance = (gate_pos.x**2 + gate_pos.y**2 + gate_pos.z**2)**0.5
        approach_factor = max(0.1, (distance - self.approach_distance) / distance)
        
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = gate_pos.x * approach_factor
        movement_msg.translation.y = gate_pos.y * approach_factor
        movement_msg.translation.z = gate_pos.z * approach_factor

        self.ros_node.get_logger().info(
            f"Approaching gate: moving x={movement_msg.translation.x:.2f}, "
            f"y={movement_msg.translation.y:.2f}, z={movement_msg.translation.z:.2f}"
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.approaching_gate_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send approach movement command")
            self.queued_method = self.abort

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Gate State Machine Completed"))