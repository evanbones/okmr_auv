import threading
import rclpy
from transitions import Machine

class TaskStateMachine:
    states = ['idle', 'done', 'aborted']# required states
    task_states = [] # states implemented by sub classes

    #required transition
    transitions = [ 
        { 'trigger': 'abort', 'source': '*', 'dest': 'aborted' }, #required transition (from anything to aborted)
    ]
    #not an actual transition, but 'start' transition is to be implemented by subclass
    task_transitions = [{ 'trigger': 'start', 'source': 'idle', 'dest': '*' }]

    def __init__(self, ros_node, done_callback=None, fail_callback=None):
        self.node = ros_node
        self.aborted = False
        self.success = False
        self.done_callback = done_callback
        self.fail_callback = fail_callback
        self._subscriptions = []
        self._timers = []
        self._clients = []
        
        self.machine = Machine(
            model=self, 
            states=self.states + self.task_states, 
            transitions=self.transitions + self.task_transitions,
            initial='idle',
            after_state_change='check_completion'
        )

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

    def add_subscription(self, topic, msg_type, callback):
        sub = self.node.create_subscription(msg_type, topic, callback, 10)
        self._subscriptions.append(sub)
        return sub

    def add_timer(self, duration, callback):
        timer = self.node.create_timer(duration, callback)
        self._timers.append(timer)
        return timer

    def add_service_client(self, service_client):
        self._clients.append(action_client)
        return service_client

    def cleanup_state_resources(self):
        for sub in self._subscriptions:
            try:
                self.node.destroy_subscription(sub)
            except:
                pass
        self._subscriptions.clear()

        for timer in self._timers:
            try:
                self.node.destroy_timer(timer)
            except:
                pass
        self._timers.clear()

        self._clients.clear()


