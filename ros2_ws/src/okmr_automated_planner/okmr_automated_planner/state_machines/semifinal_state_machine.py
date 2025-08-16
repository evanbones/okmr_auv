from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand
import time


class SemifinalStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "distance_forward1",
            "value": 4.0,
            "descriptor": "distance to move forward in segment 1",
        },
        {
            "name": "distance_forward2",
            "value": 2.0,
            "descriptor": "distance to move forward in segment 2",
        },
        {
            "name": "distance_forward3",
            "value": 2.0,
            "descriptor": "distance to move forward in segment 3",
        },
        {
            "name": "distance_forward4",
            "value": 4.0,
            "descriptor": "distance to move forward in segment 4",
        },
        {
            "name": "distance_down",
            "value": 0.75,
            "descriptor": "distance to move down",
        },
        {
            "name": "turning_angle1",
            "value": 60.0,
            "descriptor": "turning angle for turn 1 (degrees)",
        },
        {
            "name": "turning_angle2",
            "value": -40.0,
            "descriptor": "turning angle for turn 2 (degrees)",
        },
        {
            "name": "turning_angle3",
            "value": 00.0,
            "descriptor": "turning angle for turn 3 (degrees)",
        },
        {
            "name": "turning_angle4",
            "value": 0.0,
            "descriptor": "turning angle for turn 4 (degrees)",
        },
        {
            "name": "barrel_roll_angle",
            "value": 360.0,
            "descriptor": "barrel roll rotation angle (degrees)",
        },
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.distance_forward1 = self.get_local_parameter("distance_forward1")
        self.distance_forward2 = self.get_local_parameter("distance_forward2")
        self.distance_forward3 = self.get_local_parameter("distance_forward3")
        self.distance_forward4 = self.get_local_parameter("distance_forward4")
        self.distance_down = self.get_local_parameter("distance_down")

        self.turning_angle1 = self.get_local_parameter("turning_angle1")
        self.turning_angle2 = self.get_local_parameter("turning_angle2")
        self.turning_angle3 = self.get_local_parameter("turning_angle3")
        self.turning_angle4 = self.get_local_parameter("turning_angle4")
        self.barrel_roll_angle = self.get_local_parameter("barrel_roll_angle")

        self.ros_node.get_logger().info(
            f"Semifinal parameters - Forward distances: [{self.distance_forward1}, {self.distance_forward2}, {self.distance_forward3}, {self.distance_forward4}], "
            f"Down: {self.distance_down}, Turning angles: [{self.turning_angle1}, {self.turning_angle2}, {self.turning_angle3}, {self.turning_angle4}], "
            f"Barrel roll: {self.barrel_roll_angle}"
        )

    def on_enter_initializing(self):
        # check system state
        # transition to waiting for mission start
        self.queued_method = self.initialized

    def mission_command_callback(self, msg):
        """Handle incoming mission command messages"""
        if msg.command == MissionCommand.START_MISSION:
            if self.is_waiting_for_mission_start():
                self.ros_node.get_logger().info("Mission start command received")
                time.sleep(3.0)
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
        movement_msg.altitude = 1.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_down_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send sinking movement command")
            self.queued_method = self.abort

    def on_enter_moving_forward1(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward1

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward1_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send forward movement command 1"
            )
            self.queued_method = self.abort

    def on_enter_moving_forward2(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward2

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward2_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send forward movement command 2"
            )
            self.queued_method = self.abort

    def on_enter_moving_forward3(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward3

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward3_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send forward movement command 3"
            )
            self.queued_method = self.abort

    def on_enter_moving_forward4(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward4

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward4_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send forward movement command 4"
            )
            self.queued_method = self.abort

    def on_enter_turning1(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turning_angle1

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turning1_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send turning command 1")
            self.queued_method = self.abort

    def on_enter_turning2(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turning_angle2

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turning2_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send turning command 2")
            self.queued_method = self.abort

    def on_enter_turning3(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turning_angle3

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turning3_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send turning command 3")
            self.queued_method = self.abort

    def on_enter_turning4(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turning_angle4

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turning4_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send turning command 4")
            self.queued_method = self.abort

    def on_enter_barrel_roll(self):
        """Execute barrel roll using the new BARREL_ROLL command"""
        number_of_rolls = 2

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.BARREL_ROLL
        movement_msg.goal_velocity.twist.angular.x = -100.0
        movement_msg.goal_velocity.duration = 7.2
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.barrel_roll_done,
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

    def on_completion(self):
        self.ros_node.get_logger().info(
            make_green_log("Semifinal State Machine Exiting")
        )
