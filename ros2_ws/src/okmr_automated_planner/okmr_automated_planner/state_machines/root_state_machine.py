from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.msg import MissionCommand, ServoCommand


class RootStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "gate_distance",
            "value": 5.0,
            "descriptor": "distance to move forward through gate",
        },
        {
            "name": "turn_marker_one_angle",
            "value": 90.0,
            "descriptor": "angle to turn at marker one",
        },
        {
            "name": "turn_marker_two_angle",
            "value": 90.0,
            "descriptor": "angle to turn at marker two",
        },
        {
            "name": "turn_to_octagon_angle",
            "value": 90.0,
            "descriptor": "angle to turn toward octagon",
        },
        {
            "name": "approaching_slalom_distance",
            "value": 3.0,
            "descriptor": "distance to move forward approaching slalom",
        },
        {
            "name": "approaching_dropper_distance",
            "value": 2.5,
            "descriptor": "distance to move forward approaching dropper",
        },
        {
            "name": "approaching_octagon_distance",
            "value": 4.0,
            "descriptor": "distance to move forward approaching octagon",
        },
        {
            "name": "return_home_distance",
            "value": -10.0,
            "descriptor": "distance to move backward returning home",
        },
        {
            "name": "pass_gate_distance",
            "value": 3.0,
            "descriptor": "distance to move forward passing gate",
        },
        {
            "name": "slalom_turn_one_angle",
            "value": 45.0,
            "descriptor": "angle for first slalom turn",
        },
        {
            "name": "slalom_move_forward_one_distance",
            "value": 2.0,
            "descriptor": "distance for first slalom forward movement",
        },
        {
            "name": "slalom_turn_two_angle",
            "value": -90.0,
            "descriptor": "angle for second slalom turn",
        },
        {
            "name": "slalom_move_forward_two_distance",
            "value": 2.0,
            "descriptor": "distance for second slalom forward movement",
        },
        {
            "name": "mission_altitude",
            "value": 0.75,
            "descriptor": "target altitude for mission operations",
        },
        {
            "name": "translation_timeout_factor",
            "value": 3.0,
            "descriptor": "timeout multiplier for translation movements (distance * factor = timeout)",
        },
        {
            "name": "barrel_roll_angular_velocity",
            "value": -200.0,
            "descriptor": "angular velocity for barrel roll (degrees/sec)",
        },
        {
            "name": "barrel_roll_duration",
            "value": 3.6,
            "descriptor": "duration for barrel roll (seconds)",
        },
        {
            "name": "servo_pwm",
            "value": 1500.0,
            "descriptor": "PWM value for servo actuation",
        },
        {
            "name": "servo_index",
            "value": 0,
            "descriptor": "index of servo to actuate",
        },
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.gate_distance = self.get_local_parameter("gate_distance")
        self.turn_marker_one_angle = self.get_local_parameter("turn_marker_one_angle")
        self.turn_marker_two_angle = self.get_local_parameter("turn_marker_two_angle")
        self.turn_to_octagon_angle = self.get_local_parameter("turn_to_octagon_angle")
        self.approaching_slalom_distance = self.get_local_parameter(
            "approaching_slalom_distance"
        )
        self.approaching_dropper_distance = self.get_local_parameter(
            "approaching_dropper_distance"
        )
        self.approaching_octagon_distance = self.get_local_parameter(
            "approaching_octagon_distance"
        )
        self.return_home_distance = self.get_local_parameter("return_home_distance")
        self.pass_gate_distance = self.get_local_parameter("pass_gate_distance")
        self.slalom_turn_one_angle = self.get_local_parameter("slalom_turn_one_angle")
        self.slalom_move_forward_one_distance = self.get_local_parameter(
            "slalom_move_forward_one_distance"
        )
        self.slalom_turn_two_angle = self.get_local_parameter("slalom_turn_two_angle")
        self.slalom_move_forward_two_distance = self.get_local_parameter(
            "slalom_move_forward_two_distance"
        )
        self.mission_altitude = self.get_local_parameter("mission_altitude")
        self.translation_timeout_factor = self.get_local_parameter(
            "translation_timeout_factor"
        )
        self.barrel_roll_angular_velocity = self.get_local_parameter(
            "barrel_roll_angular_velocity"
        )
        self.barrel_roll_duration = self.get_local_parameter("barrel_roll_duration")
        self.servo_pwm = self.get_local_parameter("servo_pwm")
        self.servo_index = self.get_local_parameter("servo_index")

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
        request = SetDeadReckoningEnabled.Request()
        request.enable = True

        self.send_service_request(
            SetDeadReckoningEnabled,
            "/set_dead_reckoning_enabled",
            request,
            self.check_subsystem_enable_success,
        )

    def on_enter_sinking_start(self):
        self.record_initial_start_time()
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.altitude = self.mission_altitude

        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.sinking_start_done,
            on_failure=self.sinking_start_done,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send sinking movement command")
            self.queued_method = self.abort

    def on_enter_gate(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.gate_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.gate_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.gate_done,
            on_failure=self.gate_done,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send gate movement command")
            self.queued_method = self.abort

    def on_enter_barrel_roll(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.BARREL_ROLL
        movement_msg.goal_velocity.twist.angular.x = self.barrel_roll_angular_velocity
        movement_msg.goal_velocity.duration = self.barrel_roll_duration
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.barrel_roll_done,
            on_failure=self.barrel_roll_done,
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
            on_failure=self.surfacing_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send surfacing movement command"
            )
            self.queued_method = self.abort

    def on_enter_turn_marker_one(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_marker_one_angle
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_marker_one_done,
            on_failure=self.turn_marker_one_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send turn marker one movement command"
            )
            self.queued_method = self.abort

    def on_enter_approaching_slalom(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.approaching_slalom_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.approaching_slalom_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.approaching_slalom_done,
            on_failure=self.approaching_slalom_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send approaching slalom movement command"
            )
            self.queued_method = self.abort

    def on_enter_slalom_turn_one(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.slalom_turn_one_angle
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.slalom_turn_one_done,
            on_failure=self.slalom_turn_one_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send slalom turn one movement command"
            )
            self.queued_method = self.abort

    def on_enter_slalom_move_forward_one(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.slalom_move_forward_one_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.slalom_move_forward_one_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.slalom_move_forward_one_done,
            on_failure=self.slalom_move_forward_one_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send slalom move forward one movement command"
            )
            self.queued_method = self.abort

    def on_enter_slalom_turn_two(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.slalom_turn_two_angle
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.slalom_turn_two_done,
            on_failure=self.slalom_turn_two_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send slalom turn two movement command"
            )
            self.queued_method = self.abort

    def on_enter_slalom_move_forward_two(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.slalom_move_forward_two_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.slalom_move_forward_two_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.slalom_move_forward_two_done,
            on_failure=self.slalom_move_forward_two_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send slalom move forward two movement command"
            )
            self.queued_method = self.abort

    def on_enter_turn_marker_two(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_marker_two_angle
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_marker_two_done,
            on_failure=self.turn_marker_two_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send turn marker two movement command"
            )
            self.queued_method = self.abort

    def on_enter_approaching_dropper(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.approaching_dropper_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.approaching_dropper_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.approaching_dropper_done,
            on_failure=self.approaching_dropper_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send approaching dropper movement command"
            )
            self.queued_method = self.abort

    def on_enter_actuating_servo(self):
        servo_msg = ServoCommand()
        servo_msg.index = self.servo_index
        servo_msg.pwm = self.servo_pwm

        servo_publisher = self.ros_node.create_publisher(
            ServoCommand, "/servo_command", 10
        )
        servo_publisher.publish(servo_msg)

        self.ros_node.get_logger().info(
            f"Servo actuated: index={self.servo_index}, pwm={self.servo_pwm}"
        )

        self.queued_method = self.actuating_servo_done

    def on_enter_turn_to_octagon(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.turn_to_octagon_angle
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_to_octagon_done,
            on_failure=self.turn_to_octagon_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send turn to octagon movement command"
            )
            self.queued_method = self.abort

    def on_enter_approaching_octagon(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.approaching_octagon_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.approaching_octagon_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.approaching_octagon_done,
            on_failure=self.approaching_octagon_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send approaching octagon movement command"
            )
            self.queued_method = self.abort

    def on_enter_surfacing_octagon(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.surfacing_octagon_done,
            on_failure=self.surfacing_octagon_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send surfacing octagon movement command"
            )
            self.queued_method = self.abort

    def on_enter_sinking_octagon(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.sinking_octagon_done,
            on_failure=self.sinking_octagon_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send sinking octagon movement command"
            )
            self.queued_method = self.abort

    def on_enter_return_home(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.return_home_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.return_home_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.return_home_done,
            on_failure=self.return_home_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send return home movement command"
            )
            self.queued_method = self.abort

    def on_enter_pass_gate(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.pass_gate_distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            abs(self.pass_gate_distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.pass_gate_done,
            on_failure=self.pass_gate_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send pass gate movement command"
            )
            self.queued_method = self.abort

    def on_enter_surface(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.surface_done,
            on_failure=self.surface_done,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send surface movement command")
            self.queued_method = self.abort

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Root State Machine Exiting"))
