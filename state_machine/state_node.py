from transitions import State

class StateNode(State):
    def __init__(self, name, timeout=None, sub_machine=None):
        super().__init__(name=name)
        self.timeout = timeout
        self.sub_machine = sub_machine #Referance to a BaseStateMachine

    
