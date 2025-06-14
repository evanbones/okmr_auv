from transitions import State

class StateNode(State):
    #State that can also contain a machine, and other desired attributes 
    def __init__(self, name, timeout=-1, sub_machine=None):
        super().__init__(name=name)
        self.timeout = timeout
        self.sub_machine = sub_machine #Reference to a BaseStateMachine

    
