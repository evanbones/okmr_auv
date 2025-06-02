from task_state_machine import BaseStateMachine

class FindingGateStateMachine(BaseStateMachine):
    def on_enter_initializing(self):
        print(f"{self.name}: entered initializing state")

    def on_enter_scanningCW(self):
        print("Started CW scan")
        #send 360 scan request
        #if request success:
        #   add callback waiting for movement completion
        #   add callback waiting for gateFound
        #callback will 

    def on_enter_scanningCCW(self):
        print("Started CCW scan")

    def on_completion(self):
        #disable object detection

