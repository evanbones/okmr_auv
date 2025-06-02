from transitions import State

class StateNode(State):
    def __init__(self, name, timeout=None, machine=None):
        super().__init__(name=name)
        self.timeout = timeout
        self.machine = machine #Referance to a BaseStateMachine

    
