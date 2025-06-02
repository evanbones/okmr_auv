from transitions import State

class StateNode(State):
    def __init__(state_dict, machine=None):
        super.__init__(state_dict)
        self.is_machine = False
        if machine:
            self.is_machine = True
        self.machine = machine #Referance to a BaseStateMachine

