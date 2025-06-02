from base_state_machine import BaseStateMachine

class MasterStateMachine(BaseStateMachine):
    
    def on_enter_sinking():
        self.record_initial_start_time()
        #unfreeze dead reckoning

    def on_enter_findingGate(self):
        print("Started findingGate!")
        self.start_sub_machine(self.state.sub_machine, 
                               success_callback=self.findingGateDone, 
                               failure_callback=self.arbitrate_failure)

    def arbitrate_failure(self):
        self.abort()
    
