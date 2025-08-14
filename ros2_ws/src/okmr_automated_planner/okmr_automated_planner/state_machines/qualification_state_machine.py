from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand


class QualificationStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "distance_forward",
            "value": 2.0,
            "descriptor": "distance to move forward",
        },
        {
            "name": "distance_down",
            "value": 1.5,
            "descriptor": "distance to move down",
        }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.distance_forward = self.get_local_parameter("distance_forward")
        self.distance_down = self.get_local_parameter("distance_down")

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
            self.abort()

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

    def on_enter_moving_forward(self):

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send forward movement command")
            self.queued_method = self.abort

    
    def on_enter_surfacing(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.surfacing_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send surfacing movement command"
            )
            self.queued_method = self.abort
        pass

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Qualification State Machine Exiting"))
