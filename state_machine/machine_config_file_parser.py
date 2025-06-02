import yaml

class MachineConfigFileParser:
    _mandatory_states = ['uninitialized', 'done', 'aborted']
    _mandatory_transitions = [{ 'trigger': 'abort', 'source': '*', 'dest': 'aborted' }]

    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self._states = []#list of states ['a', 'b', 'c']
        self._transitions = [] #list of dictionaries {trigger: 'doneA', source: 'a', dest: 'b'}
        self._timeouts = {} #dictionary of timeouts {'a' : 30.0} in seconds
        self._config_paths = {} #dictionary of config paths for sub machines {'a' : 'task_configs/a.yaml'}
        self._load_yaml()

    def _load_yaml(self):
        with open(self.yaml_path, 'r') as file:
            config = yaml.safe_load(file)

        for state in config.get('states', []):
            if isinstance(state, dict):
                state_name = state.get('name')
                self.states.append(state_name)
                if 'timeout_sec' in state and state_name:
                    self.timeouts[state_name] = state['timeout_sec']
                if 'config_path' in state and state_name:
                    self._config_paths[state_name] = state['config_path']
            else:
                self._states.append(state)# if the state is not a dict in the yaml

        self.transitions = config.get('transitions', [])

        for state in self.mandatory_states:
            if state not in self.states:
                self.states.append(state)

    def get_states(self):
        return self._states

    def get_transitions(self):
        return self._transitions

    def get_timeouts(self):
        return self._timeouts

    def get_config_paths(self):
        return self._config_paths
