from okmr_automated_planner.base_state_machine import BaseStateMachine
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
        self.add_subscription( BatteryVoltage, "battery_voltage", self.battery_voltage_callback)
        #self.add_subscription( Status, "system_status", self.system_status_callback)
        self.queued_method = self.initializingDone
        # add SystemStatus message with more information
        # once all systems show healthy, allow next state
        # system health node should:
        # check if all required services are available
        # check if all required nodes are running
        #   can get a list of all nodes and:
        #       ensure that there are no duplicates
        #       ensure the list contains all desired nodes
        #
        # send status check to all nodes that have one (currently navigator only?)
        #   add status check to mapper and object detection nodes
        #
        # check connection to microcontroller (ESP32)
        # check connection to DVL
        #   can add a service to the dvl driver
        # check connection to cameras
        #   check that frames are being published?
        #   maybe there is a better way to test?
        # check system temperatures
        # check battery voltages
        # check leak sensor outputs?
        #
        # health check can be set as a timer
        #   certain checks can be more frequent?
        # system health node should also subscribe to certain topics all the time
        # ex. battery voltage, current draw, motor throttle output (high throttle can be a fault)


    def on_enter_initialized(self):
        self.queued_method = self.missionStartReceived

    def on_enter_sinking(self):
        self.record_initial_start_time()
        self.queued_method = self.sinkingDone
        #unfreeze dead reckoning

    def on_enter_findingGate(self):
        self.start_current_state_sub_machine(
                                            success_callback=self.findingGateDone, 
                                            fail_callback=self.abort
                                            )
        self.ros_node.get_logger().info(f"{self.get_time_since_state_start()}")

    def on_enter_aborted(self):
        self.ros_node.get_logger().fatal(f"Master State Machine Aborted")
    
