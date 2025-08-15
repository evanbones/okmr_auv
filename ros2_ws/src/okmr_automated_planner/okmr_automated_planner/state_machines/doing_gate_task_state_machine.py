from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_msgs.msg import MovementCommand
from okmr_msgs.msg import GoalVelocity
from okmr_msgs.srv import Status
from geometry_msgs.msg import Vector3
from okmr_automated_planner.state_machines.sideways_scan_state_machine import SidewaysScanStateMachine


class DoingsharkTaskStateMachine(BaseStateMachine):

    PARAMETERS = [
    #Copied from finding shark because shark object detection could be same as shark???????
    #       {
    #         "name": "frame_confidence_threshold",
    #         "value": 0.8,
    #         "descriptor": 'threshold that decides if a shark detection is "good enough"',
    #     },
    #     {
    #         "name": "initial_detection_frame_threshold",
    #         "value": 3,
    #         "descriptor": "how many detected frames to begin rotating towards detection",
    #     },
    #     {
    #         "name": "true_positive_frame_threshold",
    #         "value": 20,
    #         "descriptor": "how many frames to consider detection as true positive",
    #     },
    #     {
    #         "name": "max_scan_attempts",
    #         "value": 4,
    #         "descriptor": "how many times do we scan back and forth before giving up",
    #     },
    #     {
    #         "name": "scan_speed",
    #         "value": 30.0,
    #         "descriptor": "how fast to rotate while looking for shark",
    #     },
    #     {
    #         "name": "scan_angle",
    #         "value": 360.0,
    #         "descriptor": "how much of an angle to cover while searching for shark",
    #     },
    ]
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Initialize parameters if needed
        # self.confidence_threshold = self.get_local_parameter("frame_confidence_threshold")
        # self.initial_detection_frame_threshold = self.get_local_parameter("initial_detection_frame_threshold")
        # self.true_positive_frame_threshold = self.get_local_parameter("true_positive_frame_threshold")
        # self.max_scan_attempts = self.get_local_parameter("max_scan_attempts")
        # self.scan_speed = self.get_local_parameter("scan_speed")
        # self.scan_angle = self.get_local_parameter("scan_angle")

    def detection_callback(self, msg):
        """Callback for shark detection messages"""
        # TODO: Replace with actual BoundingBox message parsing when available
        # For now, simulate confidence as 1.0
        confidence = 1.0
        self.cached_shark_bounding_box = msg

        if confidence > self.confidence_threshold:
            self.high_confidence_frame_count += 1

            # If currently scanning, initiate look at shark sequence
            if (
                (self.is_scanning_cw() or self.is_scanning_ccw())
                and self.high_confidence_frame_count
                > self.initial_detection_frame_threshold
            ):
                self.ros_node.get_logger().info(
                    make_green_log(
                        f"High confidence detection (confidence: {confidence:.2f}), following detection"
                    )
                )
                self.object_detected()
        #TODO maybe fix this logic if cooked
        else:
            self.ros_node.get_logger().info(
                make_green_log(
                    f"confidence not met, no shark found (confidence: {confidence:.2f})"
                )
            )
            
    def on_enter_initializing(self):
        self.queued_method = self.initialized
        pass
    def on_enter_approaching_gate(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        #TODO Hard coded qurstion mark ?
        #movement_msg.translation.x = 

    # def on_enter_sideways_scan(self):
    #     self.start_sub_state_machine(
    #         SidewaysScanStateMachine
    #         success_callback = self.sideways_scan_shark_done,
    #         failure_callback = self.abort,
    #     )
    #     pass
    def on_enter_return_initial_gate(self):
    #TODO lowk idk whats hapenning
        pass


   


    def on_completion(self):
        # disable object detection
        pass

states = [
    InitialState,
    SidewaysScanStateMachine,
    ApproachingGate,
    ReturnInitialGate,
    CompletedState,
]
