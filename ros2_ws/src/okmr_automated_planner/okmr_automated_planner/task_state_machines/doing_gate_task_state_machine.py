from okmr_automated_planner.base_state_machine import BaseStateMachine

class DoingGateTaskStateMachine(BaseStateMachine):

    def on_enter_initializing(self):
        # ? what to do on init ?
        pass

    def on_enter_initialized(self):
        # 
        pass

    def on_completion(self):
        # disable object detection
        pass

