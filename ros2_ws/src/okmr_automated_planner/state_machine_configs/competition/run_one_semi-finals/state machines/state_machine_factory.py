import os
from okmr_automated_planner.machine_config_file_parser import MachineConfigFileParser
from okmr_automated_planner.state_node import StateNode
from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_automated_planner import state_machines
from okmr_automated_planner.state_machines import RunOneRootStateMachine


class StateMachineFactory:
    @staticmethod
    def createMachineFromConfig(
        config_yaml: str, ros_node, config_base_path=None
    ) -> "BaseStateMachine":
        """
        Creates a state machine based on the provided YAML configuration file.
        Recursively creates sub-machines for states with config_path attribute.

        Args:
            config_yaml: Path to the YAML configuration file (relative to config_base_path)
            ros_node: ROS2 node to be used by the state machine
            config_base_path: Base directory for configuration files

        Returns:
            An instance of the appropriate state machine subclass, or None if creation fails
        """
        # Construct full path using base path if provided
        full_config_path = config_yaml
        if config_base_path:
            full_config_path = os.path.join(config_base_path, config_yaml)
            ros_node.get_logger().debug(f"Using full config path: {full_config_path}")

        if not os.path.exists(full_config_path):
            ros_node.get_logger().fatal(f"Config file not found: {full_config_path}")
            raise ValueError(f"Config file not found: {full_config_path}")

        # Parse the configuration file
        config_parser = MachineConfigFileParser(full_config_path)
        machine_name = config_parser.name
        states = config_parser.get_states()
        transitions = config_parser.get_transitions()

        # Create state objects for each state, recursively creating sub-machines if needed
        state_objects = []
        for state_dict in states:

            state_node = StateNode(state_dict["name"])

            # If this state has a config_path, it means it has a sub-machine
            if "config_path" in state_dict:
                sub_config_path = state_dict["config_path"]
                # Recursively create the sub-machine with the same config base path
                sub_machine = StateMachineFactory.createMachineFromConfig(
                    sub_config_path, ros_node, config_base_path
                )
                sub_machine.name = state_dict[
                    "name"
                ]  # setting the name = state_name allows for the same config to be used multiple times with unique parameters names
                # ex. can have finding_marker used twice in root.yaml, but with different parameters
                state_node.sub_machine = sub_machine

            # if theres a timeout in the config, set it for the StateNode
            if "timeout" in state_dict:
                state_node.timeout = state_dict["timeout"]

            state_objects.append(state_node)

        # Create the appropriate machine instance based on name
        config_to_class_dict = {
            "run_one_root": state_machines.RunOneRootStateMachine,
            "gate": state_machines.GateStateMachine,
            "slalom": state_machines.SlalomStateMachine,
        }

        if not machine_name in config_to_class_dict.keys():
            raise ValueError(
                f"No implementation found for machine type: {machine_name}"
            )

        machine_instance = config_to_class_dict[machine_name](
            name=machine_name,
            ros_node=ros_node,
            states=state_objects,
            transitions=transitions,
        )

        return machine_instance
