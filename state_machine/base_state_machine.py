import threading
from transitions import Machine
from state_node import StateNode

class BaseStateMachine(Machine):
    mandatory_states = ['uninitialized','initializing','initialized', 'done', 'aborted']
    mandatory_transitions = [
                                { 'trigger': 'initialize', 'source': 'uninitialized', 'dest': 'initializing' },
                                { 'trigger': 'initializingDone', 'source': 'initializing', 'dest': 'initialized' },
                                { 'trigger': 'finish', 'source': '*', 'dest': 'done' },
                                { 'trigger': 'abort', 'source': '*', 'dest': 'aborted' },
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
        self.machine_name = name
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

        self.success = False
        
        self.add_mandatory_states()
        self.add_mandatory_transitions()

        super().__init__(
            model=self, 
            states=self.states, 
            transitions=self.transitions,
            initial='uninitialized',
            after_state_change='state_change_callback',
            *args,
            **kwargs #allows for additional arguments
        )

        self.current_sub_machine = None

    def add_mandatory_transitions(self):
        for state in self.mandatory_states:
            if state not in self.states:
                self.states.append(state)

    def add_mandatory_states(self):
        mandatory_triggers = [transition['trigger'] for transition in self.mandatory_transitions]
        all_triggers = [transition['trigger'] for transition in self.transitions]
        for transition in self.mandatory_transitions:
            if transition["trigger"] not in all_triggers:
                self.transitions.append(transition)

    def record_initial_start_time(self):
        self.initial_start_time =  self.ros_node.get_clock().now()

    def record_state_start_time(self):
        self.state_start_time =  self.ros_node.get_clock().now()
    
    def on_completion(self):
        #to be implemented by sub classes if desired
        pass

    def state_change_callback(self):
        self.record_state_start_time()
        self.log_state_change()
        self.check_completion()

    def log_state_change(self):
        self.ros_node.get_logger().info(f"State Change \t Machine: {self.machine_name} \t State: {self.state}")
        #print(f"State Change\t{self.name}\t{self.state}")

    def check_completion(self):
        if self.state == 'done' or self.state == 'aborted':
            self.on_completion()
            self.cleanup_ros2_resources()
            #optional success and failure callbacks
            #can be used to hand control back to a parent machine
            if self.success and self.done_callback:
                self.success_callback()
            elif not self.success and self.fail_callback:
                self.fail_callback()

    def start_sub_machine(self, sub_machine, success_callback=None, failure_callback=None):
        self.current_sub_machine = sub_machine
        sub_machine.success_callback = success_callback
        sub_machine.failure_callback = failure_callback
        threading.Thread(target=sub_machine.initialize, daemon=True).start()

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

    
