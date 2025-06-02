import os
from machine_config_file_parser import MachineConfigFileParser
from base_state_machine import BaseStateMachine
from master_state_machine import MasterStateMachine
from task_state_machines.finding_gate_state_machine import FindingGateStateMachine

class StateMachineFactory:
    @staticmethod
    def createMachineFromConfig(config_yaml: str, ros_node) -> 'BaseStateMachine':
        """
        Creates a state machine based on the provided YAML configuration file.
        Recursively creates sub-machines for states with config_path attribute.
        
        Args:
            config_yaml: Path to the YAML configuration file
            ros_node: ROS2 node to be used by the state machine
            
        Returns:
            An instance of the appropriate state machine subclass, or None if creation fails
        """
        if not os.path.exists(config_yaml):
            print(f"Config file not found: {config_yaml}")
            return None
            
        # Parse the configuration file
        config_parser = MachineConfigFileParser(config_yaml)
        machine_name = config_parser.name
        states = config_parser.get_states()
        transitions = config_parser.get_transitions()
        
            # Create state objects for each state, recursively creating sub-machines if needed
        state_objects = []
        for state_dict in states:
            # If this state has a config_path, it means it has a sub-machine
            sub_machine = None
            if 'config_path' in state_dict:
                sub_config_path = state_dict['config_path']
                state_dict.pop('config_path')
                # Recursively create the sub-machine
                sub_machine = StateMachineFactory.createMachineFromConfig(
                    sub_config_path, 
                    ros_node
                )

            state_objects.append(StateNode(state_dict, sub_machine))
            
            # Create the appropriate machine instance based on name
        machine_instance = None
        if machine_name == "master":
            machine_instance = MasterStateMachine(
                name=machine_name,
                ros_node=ros_node,
                states=state_objects,
                transitions=transitions
            )
        elif machine_name == "findingGate":
            machine_instance = FindingGateStateMachine(
                name=machine_name,
                ros_node=ros_node,
                states=state_objects,
                transitions=transitions
            )
        else:
            print(f"No implementation found for machine type: {machine_name}")
            return None
            
        return machine_instance
