from okmr_automated_planner.base_state_machine import BaseStateMachine


class DoingGateTaskStateMachine(BaseStateMachine):

    def on_enter_initializing(self):
        # enable shark detection
        pass

    # TODO need to add all on_enter_* methods

    def on_completion(self):
        # disable object detection
        pass
