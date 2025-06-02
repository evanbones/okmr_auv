import os
from machine_config_file_parser import MachineConfigFileParser

def StateMachineFactory:
    def createStateMachine(self, config_yaml: str) -> 'BaseStateMachine':
        if os.fileExists(config_yaml):
            #arbitrate between the different state machine types and return the correct one.
            #if not matching a declared machine type, return none, since a machine with no implementation is useless
        return None

