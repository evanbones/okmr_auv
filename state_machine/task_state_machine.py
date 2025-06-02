import threading
import rclpy
from transitions import Machine

class TaskStateMachine:
    states = ['uninitialized', 'done', 'aborted']# required states
    task_states = [] # states implemented by sub classes

    #required transition
    transitions = [ 
        { 'trigger': 'abort', 'source': '*', 'dest': 'aborted' }, #required transition (from anything to aborted)
    ]
    #not an actual transition, but 'start' transition must be implemented by subclass
    task_transitions = [{ 'trigger': 'start', 'source': 'idle', 'dest': '*' }]

    def __init__(self,  done_callback=None, fail_callback=None):
        super.__init__
        self.aborted = False
        self.success = False
        self.done_callback = done_callback
        self.fail_callback = fail_callback
        
        
        

    def run(self):
        self.aborted = False
        self.success = False
        self.start() #transition from idle to next state

    def on_enter_aborted(self):
        print("aborting!!!")
        self.aborted = True
        self.check_completion()

    def check_completion(self):
        if self.state == 'done' or self.aborted:
            self.cleanup_state_resources()
            if self.success and self.done_callback:
                self.done_callback()
            elif (self.aborted or not self.success) and self.fail_callback:
                self.fail_callback()

    


