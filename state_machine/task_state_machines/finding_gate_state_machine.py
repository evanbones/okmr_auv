from base_state_machine import BaseStateMachine

class FindingGateStateMachine(BaseStateMachine):

    def on_enter_initializing(self):
        # start up object detection model
        pass

    def on_enter_initialized(self):
        # add callback waiting for gateFound
        pass

    def on_enter_scanningCW(self):
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

