# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build/Run Commands
- Run state machine: `python3 automated_planner.py`
- Generate state machine graph: Uncomment lines in `automated_planner.py` that generate graph visualization

## Code Style Guidelines
- **Imports**: Group imports (stdlib first, then external libraries, then local modules)
- **Classes**: Use CamelCase for class names (e.g., `BaseStateMachine`, `FindingGateStateMachine`)
- **Methods/Functions**: Use snake_case for method names (e.g., `add_subscription`, `cleanup_ros2_resources`)
- **Variables**: Use snake_case for variable names
- **Error Handling**: Use try/except blocks with specific exceptions when possible
- **ROS2 Integration**: Always clean up ROS2 resources (subscriptions, timers, clients)
- **State Machines**: Define states and transitions clearly in YAML configs when possible
- **Documentation**: Use docstrings for classes and non-trivial methods
- **Indentation**: 4 spaces
- **Config Files**: Use YAML for state machine configurations (states, transitions, timeouts)