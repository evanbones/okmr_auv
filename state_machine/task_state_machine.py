
class TaskStateMachine:
    task_states = [] # states implemented by sub classes

    #required transition
    transitions = [ 
        { 'trigger': 'abort', 'source': '*', 'dest': 'aborted' }, #required transition (from anything to aborted)
    ]
    #not an actual transition, but 'start' transition must be implemented by subclass
    task_transitions = [{ 'trigger': 'start', 'source': 'idle', 'dest': '*' }]

    def __init__(self,  ):
        super.__init__
        
        
        
        

    

    

    

    


