from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_automated_planner.state_machines.gate_state_machine import GateStateMachine
from okmr_automated_planner.state_machines.slalom_state_machine import SlalomStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand
 
class RunOneRootStateMachine(BaseStateMachine):
    PARAMETERS = [
        {
            "name": "distance_down",
            "value": 0.3,
            "descriptor": "distance to move down",
        },
        {
            "name": "turn_marker_one",
            "value": 45, #TODO change depending on it irl
            "descriptor": "angle to turn at marker one",                                                           
        },
        {
            "name": "turn_marker_two",
            "value": 45, #TODO change depending on it irl
            "descriptor": "angle to turn at marker two",
        }
    ]
        
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.distance_down = self.get_local_parameter("distance_down")
        self.ros_node.get_logger().info(f"Distance down: {self.distance_down}")
       
        self.turn_marker_one = self.get_local_parameter("turn_marker_one")
        self.turn_marker_two = self.get_local_parameter("turn_marker_two")
  
    def on_enter_initializing(self):
        # check system state
        # transition to waiting for mission start
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
            # self.abort()
   
    def on_enter_waiting_for_mission_start(self):
        """Wait for subscription to /mission_command topic"""
        self.ros_node.get_logger().info("Waiting for mission start command...")
        self.add_subscription(
            MissionCommand, "/mission_command", self.mission_command_callback
        )
    
    def check_subsystem_enable_success(self, future):
        """Check response from dead reckoning service, making sure its success"""
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
        # Create service request
        request = SetDeadReckoningEnabled.Request()
        request.enable = True

        # Send the service request with callback
        self.send_service_request(
            SetDeadReckoningEnabled,
            "/set_dead_reckoning_enabled",
            request,
            self.check_subsystem_enable_success,
        )

    def on_enter_moving_down(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.z = -self.distance_down

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_down_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send sinking movement command")
            self.queued_method = self.abort

    def on_enter_gate(self):
        self.start_sub_state_machine(
            GateStateMachine,
            success_callback=self.rotating_scan_done,
            fail_callback=self.abort,
        )
    
    def on_enter_turn_marker_one(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_marker_one

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_marker_one_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send marker 1 turn command")
            self.queued_method = self.abort

    def on_enter_turn_marker_two(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_marker_two

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_marker_two_done,
            on_failure=self.abort,
        )
        if not success:
            self.ros_node.get_logger().error("Failed to send marker 2 turn command")
            self.queued_method = self.abort
    
    def on_enter_slalom(self):
        self.start_sub_state_machine(
            SlalomStateMachine,
            success_callback=self.slalom_done,
            fail_callback=self.abort,
        )

