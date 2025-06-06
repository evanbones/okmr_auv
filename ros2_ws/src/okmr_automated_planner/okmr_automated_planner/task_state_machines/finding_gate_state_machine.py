from okmr_msgs.msg import MovementCommand
from okmr_msgs.srv import Status

from okmr_automated_planner.base_state_machine import BaseStateMachine

class FindingGateStateMachine(BaseStateMachine):

    def send_navigator_status_check(self):
        self.send_service_request(Status, '/navigator_status', )

    def on_enter_initializing(self):
        # start up object detection model
        self.queued_method = self.initializingDone
        pass

    def on_enter_initialized(self):
        # add callback waiting for gateFound
        self.queued_method = self.gateDetectionOn
        pass

    def on_enter_scanningCW(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SPIN
        movement_msg.rotation_speed = 30.0
        self.publish_on_topic(MovementCommand, '/movement_command', movement_msg)
        # send request
        # add callback waiting for movement completion
        #   callback triggers scanningCWDone
        pass

    def on_enter_scanningCCW(self):
        # send request
        # add callback waiting for movement completion
        #   callback triggers scanningCCWDone
        pass

    def on_completion(self):
        # disable object detection
        pass

