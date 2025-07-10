import rclpy
from threading import Lock

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from okmr_msgs.action import Movement
from okmr_msgs.msg import MovementCommand

class NavigatorActionServer(Node):
    _instance = None
    _lock = Lock()

    def __init__(self, command_handlers, test_command_handlers):
        super().__init__('navigator_action_server')
        
        self.declare_parameter('test_mode', False)
        self.declare_parameter('feedback_rate', 10.0)
        self.declare_parameter('freeze_linear_velocity_threshold', 0.1)
        self.declare_parameter('freeze_angular_velocity_threshold', 0.1)

        self.command_handlers = command_handlers
        self.test_command_handlers = test_command_handlers
        
        self._goal_handle = None #this is the only goal handle allowed to run
        self._goal_lock = Lock()
        self._publishers = {}  # Dictionary to store reusable publishers
        self._action_server = ActionServer(
            self,
            Movement,
            'movement_command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback
        )
        
        test_mode = self.get_parameter('test_mode').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.freeze_linear_velocity_threshold = self.get_parameter('freeze_linear_velocity_threshold').value
        self.freeze_angular_velocity_threshold = self.get_parameter('freeze_angular_velocity_threshold').value

        self.get_logger().info(f'Navigator Action Server started (test_mode: {test_mode})')

    def get_publisher(self, topic_name, msg_type, qos=10):
        """Get or create a publisher for the given topic"""
        if topic_name not in self._publishers:
            self._publishers[topic_name] = self.create_publisher(msg_type, topic_name, qos)
        return self._publishers[topic_name]

    @classmethod
    def get_instance(cls, command_handlers=None, test_command_handlers=None):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    if command_handlers is None or test_command_handlers is None:
                        raise RuntimeError("NavigatorActionServer instance not created yet. Must provide handlers on first call.")
                    cls._instance = cls(command_handlers, test_command_handlers)
        return cls._instance

    def goal_callback(self, goal_request):
        """Accept new goals and allow preemption of existing goals."""
        self.get_logger().info('New goal received')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        #code for preemption found in:
        # https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py
        #not sure why never mentioned in official tutorials?
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()#i assume this works because the default handler just calls this anyway

    def cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().warn('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        requested_movement = goal_handle.request.command_msg
        test_mode = self.get_parameter('test_mode').value
        
        handler_dict = self.test_command_handlers if test_mode else self.command_handlers
        
        if requested_movement.command in handler_dict:
            handler = handler_dict[requested_movement.command]
            return handler(goal_handle)
        else:
            self.get_logger().error(f'Unknown command: {requested_movement.command}')
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Unknown command: {requested_movement.command}'
            return result
    

