import threading
from transitions import Machine

class BaseStateMachine(Machine):
    
    def __init__(self, node, states, transitions, *args, **kwargs):
        self.done_callback = done_callback
        self.fail_callback = fail_callback
        self.node = node
        self.states = states
        self.transitions = transitions

        self._subscriptions = []
        self._timers = []
        self._clients = []

        #"global" aborted and success states
        self.machine_aborted = False 
        self.machine_success = False
        
        Machine.__init__(
            model=self, 
            states=self.states, 
            transitions=self.transitions,
            initial='uninitialized'
            **kwargs #allows for specifics like "after_state_change"
        )

    def run(self):
        self.aborted = False
        self.success = False
        self.start() #transition from idle to next state
        #all state machines must implement the start transition!

    def on_enter_aborted(self):
        print("aborting!!!")
        self.aborted = True
        self.check_completion()

    def check_completion(self):
        if self.state == 'done' or self.machine_aborted:
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

    
