import threading
from transitions import Machine
from state_node import StateNode

class BaseStateMachine(Machine):
    mandatory_states = ['uninitialized','initializing','initialized', 'done', 'aborted']
    mandatory_transitions = [
                                { 'trigger': 'start', 'source': 'uninitialized', 'dest': 'initializing' },
                                { 'trigger': 'initializingDone', 'source': 'initializing', 'dest': 'initialized' },
                                { 'trigger': 'finish', 'source': '*', 'dest': 'done' }
                                { 'trigger': 'abort', 'source': '*', 'dest': 'aborted' }
                              ]
    def __init__(self, 
                 name, 
                 ros_node, 
                 states, 
                 transitions, 
                 success_callback=None, 
                 fail_callback=None, 
                 *args, **kwargs):

        self.state_cls = StateNode
        self.name = name
        self.ros_node = ros_node
        self.states = states
        self.transitions = transitions
        
        self.success_callback = success_callback
        self.fail_callback = fail_callback
        
        self._subscriptions = []
        self._timers = []
        self._clients = []

        self.initial_start_time = 0
        self.state_start_time = 0

        #"global" aborted and success states
        self.machine_aborted = False 
        self.machine_success = False
        
        self.add_mandatory_state()
        self.add_mandatory_transitions()

        Machine.__init__(
            model=self, 
            states=self.states, 
            transitions=self.transitions,
            initial='uninitialized',
            after_state_change='after_state_change'
            **kwargs #allows for additional arguments
        )

    def add_mandatory_transitions(self):
        for state in self._mandatory_states:
            if state not in self.states:
                self.states.append(state)

    def add_mandatory_states(self):
        mandatory_triggers = [transition['trigger'] for transition in self._mandatory_transitions]
        for transition in self._mandatory_transitions:
            if transition["trigger"] not in mandatory_triggers:
                self.transitions.append(transition)

    def record_initial_start_time(self):
        self.initial_start_time =  self.ros_node.get_clock().now()

    def record_state_start_time(self):
        self.state_start_time =  self.ros_node.get_clock().now()
    
    def on_completion(self):
        #to be implemented by sub classes if desired
        pass

    def after_state_change(self):
        self.record_state_start_time()
        self.log_state_change()
        self.check_completion()

    def log_state_change(self):
        self.ros_node.get_logger().info(f"State Change\t{self.name}\t{self.state}")
        print(f"State Change\t{self.name}\t{self.state}")

    def check_completion(self):
        if self.state == 'done' or self.state == 'aborted':
            self.on_completion()
            self.cleanup_ros2_resources()
            #optional success and failure callbacks
            #can be used to hand control back to a parent machine
            if self.success and self.done_callback:
                self.success_callback()
            elif self.success and self.fail_callback:
                self.fail_callback()

    def start_sub_machine(self, sub_machine):
        self.current_sub_machine = sub_machine
        threading.Thread(target=sub_machine.start, daemon=True).start()

    def add_subscription(self, topic, msg_type, callback):
        sub = self.ros_node.create_subscription(msg_type, topic, callback, 10)
        self._subscriptions.append(sub)
        return sub

    def add_timer(self, duration, callback):
        timer = self.ros_node.create_timer(duration, callback)
        self._timers.append(timer)
        return timer

    def add_service_client(self, service_client):
        self._clients.append(service_client)
        return service_client

    def cleanup_ros2_resources(self):
    '''
    Cleans up all ROS2 resources from this Machine
    '''
        for sub in self._subscriptions:
            try:
                self.ros_node.destroy_subscription(sub)
            except:
                pass
        self._subscriptions.clear()

        for timer in self._timers:
            try:
                self.ros_node.destroy_timer(timer)
            except:
                pass
        self._timers.clear()

        self._clients.clear()

    
