from task_state_machine import BaseStateMachine

class FindGateStateMachine(BaseStateMachine):
    def on_enter_idle(self):
        print("entered idle state")

    def on_enter_scanningCW(self):
        print("Started CW scan")
        #send 360 scan request
        #wait until spin done
        #   ping the navigator to ask if the action is done (is the status of the navigator success = true and ongoing = false)
        #bc

    def on_enter_scanningCCW(self):
        print("Started CCW scan")

    def on_enter_surfaced(self):
        print("Surfaced")
