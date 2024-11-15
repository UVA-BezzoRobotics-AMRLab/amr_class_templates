#!/usr/bin/env python

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

from crazyflie_interfaces.srv import *
from crazyflie_interfaces.msg import LogDataGeneric

from rcl_interfaces.srv import *
from rcl_interfaces.msg import *


START_Kp = 10.0
START_Kd = 0.0

class PIDTunerNode(Node):
    def __init__(self, cf_name = 'cf80'):
        super().__init__('pid_tuner')
        self.cf = cf_name

        qos_profile    = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.gyro_sub  = self.create_subscription(LogDataGeneric, f"/{self.cf}/gyro_data", self.gyro_callback, qos_profile)
        self.accel_sub = self.create_subscription(LogDataGeneric, f"/{self.cf}/accel_data", self.accel_callback, qos_profile)
        self.vel_sub   = self.create_subscription(Twist, f"/{self.cf}/cmd_vel_legacy", self.vel_callback, qos_profile)
        
        self.timer_period_s = 1.0
        self.setup_timer    = self.create_timer(self.timer_period_s, self.setup)
        self.cmd_timer      = None

        # Variables for Node
        self.pitch_kp = START_Kp
        self.pitch_kd = START_Kd

        self.vel_active = False

        # Clients to get/set parameters
        param_names = [
            f"{self.cf}.params.pid_rate.pitch_kp", 
            f"{self.cf}.params.pid_rate.pitch_kd",
            f"{self.cf}.params.pid_rate.pitch_ki"]

        self.kp_idx = 0
        self.kd_idx = 1
        self.ki_idx = 2

        self.set_client       = self.create_client(SetParameters, '/crazyflie_server/set_parameters')
        self.set_msg          = SetParameters.Request()
        self.param_set_params = [Parameter(name=nm, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE)) for nm in param_names]
        self.set_future       = None

        # ------------------------------------- #
        # Optional: Add variables you want here #
                
        # ------------------------------------- #
        

    def setup(self):
        if self.set_client.service_is_ready():
            self.get_logger().warn("Parameter service ready!")

            self.param_set_params[self.kp_idx].value.double_value = self.pitch_kp
            self.param_set_params[self.kd_idx].value.double_value = self.pitch_kd
            self.param_set_params[self.ki_idx].value.double_value = 0.0

            self.set_msg.parameters = self.param_set_params
            self.set_future         = self.set_client.call_async(self.set_msg)

            self.setup_timer.cancel()
            self.cmd_timer = self.create_timer(self.timer_period_s, self.control_func)

            # --------------------------------------- #
            # Optional: Additional Setup actions here #

            # --------------------------------------- #
        else:
            self.get_logger().info("Parameter service NOT ready")

    def control_func(self):
        if self.ang_vel_received and self.accel_received and self.vel_active:
            self.ang_vel_received = False
            self.accel_received   = False

            # First, checking if call to set parameter has finished
            if self.set_future:
                if self.set_future.done():
                    response = self.set_future.result()
                    if response is not None:
                        for rst in response.results:
                            if not rst.successful:
                                self.get_logger().warn(f"Failed to set parameter. Reason: {rst.reason}")
                    else:
                        self.get_logger().warn("Call to set parameter service failed")
                    
                    self.set_future = None
                    
            # If call has finished, future will be set to "None"
            # Update gains and set parameters again
            if not self.set_future:
                # ---------------------------- #
                #    Your logic goes here      #

                # Example: Setting proportional and derivative gains
                self.pitch_kp = 0.0
                self.pitch_kd = 0.0
    
                # ---------------------------  #

                # Call to service to set parameters
                self.param_set_params[self.kp_idx].value.double_value = self.pitch_kp
                self.param_set_params[self.kd_idx].value.double_value = self.pitch_kd

                self.set_msg.parameters = self.param_set_params
                self.set_future         = self.set_client.call_async(self.set_msg)
 

    def gyro_callback(self, msg):
        self.get_logger().info(f"Ang. Velocity X: {msg.values[0]}  deg/s")
        self.get_logger().info(f"Ang. Velocity Y: {msg.values[1]}  deg/s")
        self.get_logger().info(f"Ang. Velocity Z: {msg.values[2]}  deg/s")

        # ------------------------------------------ #
        #    Store angular velocity data [deg/s]     #

        # ------------------------------------------ #        

    def accel_callback(self, msg):
        self.get_logger().info(f"Acceleration X: {msg.values[0]}  deg/s")
        self.get_logger().info(f"Acceleration Y: {msg.values[1]}  deg/s")
        self.get_logger().info(f"Acceleration Z: {msg.values[2]}  deg/s")

        # ------------------------------------------ #
        #       Store accleration data [g]           #

        # ------------------------------------------ #    

    def vel_callback(self, msg):
        self.vel_active = msg.linear.z > 0.0
        
def main(args=None):
    rclpy.init(args=args)
    
    pid_tune_node = PIDTunerNode()
    rclpy.spin(pid_tune_node)

    pid_tune_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
