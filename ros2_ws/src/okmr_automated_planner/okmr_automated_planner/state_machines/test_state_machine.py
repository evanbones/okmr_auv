from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand


class TestStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "barrel_roll_speed",
            "value": 100.0,
            "descriptor": "degree per second to do barrel roll",
        },
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.barrel_roll_speed = self.get_local_parameter("barrel_roll_speed")

    def on_enter_initializing(self):
        # check system state
        # placeholder queued method
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

    def on_enter_sinking(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.z = -1.5

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.sinking_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send sinking movement command")
            self.queued_method = self.abort

    def on_enter_test_scan(self):
        self.start_current_state_sub_machine(
            success_callback=self.test_scan_done, fail_callback=self.abort
        )

    def on_enter_roll(self):

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.x = 90.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.roll_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send roll movement command")
            self.queued_method = self.abort

    def on_enter_barrel_rolling(self):
        """Execute barrel roll using the new BARREL_ROLL command"""
        number_of_rolls = 2

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.BARREL_ROLL
        movement_msg.goal_velocity.twist.angular.x = self.barrel_roll_speed
        movement_msg.goal_velocity.duration = (
            360.0 / self.barrel_roll_speed * number_of_rolls
        )
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.barrel_rolling_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send barrel roll movement command"
            )
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
        self.ros_node.get_logger().info(make_green_log("Test State Machine Exiting"))
