from base_state_machine import BaseStateMachine
from okmr_msgs.msg import Status, BatteryVoltage

class MasterStateMachine(BaseStateMachine):
    
    def system_status_callback(self, msg):
        if msg.status == Status.SUCCESS:
            self.remove_subscription("system_status")
            #need to remove subscription so "initializingDone" isnt called again
            self.initializingDone()

    def battery_voltage_callback(self, msg):
        voltage_sum = round(sum(msg.cell_voltages), 3)

        if voltage_sum >= self.ros_node.get_max_battery_voltage():
            self.ros_node.get_logger().fatal(f"Battery Voltage Critically High \t{voltage_sum}V")
            self.abort()

        if voltage_sum <= self.ros_node.get_min_battery_voltage():
            self.ros_node.get_logger().fatal(f"Battery Voltage Critically Low \t{voltage_sum}V")
            self.abort()

        elif voltage_sum <= self.ros_node.get_low_battery_voltage():
            self.ros_node.get_logger().warn(f"Battery Voltage Low \t{voltage_sum}V")

    def on_enter_initializing(self):
        self.add_subscription("battery_voltage", BatteryVoltage, self.battery_voltage_callback)
        self.add_subscription("system_status", Status, self.system_status_callback)

    def on_enter_initialized(self):
        self.queued_method = self.missionStartReceived

    def on_enter_sinking(self):
        self.record_initial_start_time()
        self.queued_method = self.sinkingDone
        #unfreeze dead reckoning

    def on_enter_findingGate(self):
        self.start_current_state_sub_machine(
                                            success_callback=self.findingGateDone, 
                                            #fail_callback=self.abort
                                            )
        self.ros_node.get_logger().info(f"{self.get_time_since_state_start()}")

    def on_enter_aborted(self):
        self.ros_node.get_logger().fatal(f"Master State Machine Aborted")
    
