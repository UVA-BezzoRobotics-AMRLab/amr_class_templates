#!/usr/bin/env python

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from enum import Enum
from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Arm
from crazyflie_interfaces.srv import Land
from crazyflie_interfaces.srv import Takeoff

from builtin_interfaces.msg import Duration 

from rcl_interfaces.srv import *
from rcl_interfaces.msg import *
from geometry_msgs.msg import PoseStamped

class VehicleState(Enum):
    IDLE = 0
    ARMED = 1
    IN_AIR = 2
    LANDING = 3
    LANDED = 4

class TakeoffNode(Node):

    def __init__(self, cf_name='cf80'):
        super().__init__('takeoff_node')
        self.cf = cf_name

        self.arm_client = self.create_client(Arm, "/all/arm")
        self.takeoff_client = self.create_client(Takeoff, "/all/takeoff")
        self.land_client = self.create_client(Land, "/all/land")

        qos_profile    = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.z_sub = self.create_subscription(PoseStamped, f"/{self.cf}/pose", self.z_callback, qos_profile)

        self.setup_timer = self.create_timer(5.0, self.setup)
        self.ctrl_timer = None

        self.state = VehicleState.IDLE  # Initialize the state to IDLE
        self.arm_future = None  # Future object for async service command
        self.takeoff_future = None  # Future object for async takeoff command
        self.landing_future = None  # Future object for async land command

        self.z_desired = 0.75
        self.z_height = None

        param_names = [
            f"{self.cf}.params.posCtlPid.zKp",
            f"{self.cf}.params.posCtlPid.zKi",
            f"{self.cf}.params.posCtlPid.zKd",
            f"{self.cf}.params.posCtlPid.zKff"]

        self.kp_idx = 0
        self.ki_idx = 1
        self.kd_idx = 2
        self.kff_idx = 3

        self.set_client       = self.create_client(SetParameters, '/crazyflie_server/set_parameters')
        self.set_msg          = SetParameters.Request()
        self.param_set_params = [Parameter(name=nm, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE)) for nm in param_names]
        self.set_future       = None

    def z_callback(self, data):
        self.z_height = data.pose.position.z


    def setup(self):
        if self.state == VehicleState.IDLE:

            self.get_logger().info("Vehicle is IDLE, waiting to arm...")
            self.arm_vehicle()

        elif self.state == VehicleState.ARMED:
            if self.takeoff_future is None:
                self.get_logger().info("Vehicle armed, initiating takeoff...")
                self.initiate_takeoff()
                self.state = VehicleState.IN_AIR

        elif self.state == VehicleState.IN_AIR:
            self.get_logger().info("Vehicle is in the air, waiting to land...")

            if self.set_future is None and self.takeoff_future.done() and abs(self.z_height-self.z_desired) < 5e-2:
                self.get_logger().info("setting all params to 0")
                self.param_set_params[self.kp_idx].value.double_value = 15.0
                self.param_set_params[self.kd_idx].value.double_value = 0.0
                self.param_set_params[self.ki_idx].value.double_value = 0.0
                self.param_set_params[self.kff_idx].value.double_value = 0.0

                self.set_msg.parameters = self.param_set_params
                self.set_future         = self.set_client.call_async(self.set_msg)

            elif self.set_future is not None and self.set_future.done():
                self.get_logger().info("Takeoff initiated, waiting for 5 seconds...")
                self.setup_timer.destroy()
                self.setup_timer = self.create_timer(15.0, self.attempt_land)

        elif self.state == VehicleState.LANDING:
            self.get_logger().info("Landing in progress...")

        elif self.state == VehicleState.LANDED:
            self.get_logger().info("Vehicle has landed. Waiting for next command...")

    def arm_vehicle(self):

        self.get_logger().info("Sending Arming Request...")
        arm_req = Arm.Request(arm=True)
        self.arm_future = self.arm_client.call_async(arm_req)
        self.state = VehicleState.ARMED
        self.setup_timer.destroy()
        self.setup_timer = self.create_timer(0.1, self.check_arm_status)

    def check_arm_status(self):

        if self.arm_future.done():
            self.get_logger().info("Vehicle armed successfully.")
            self.state = VehicleState.ARMED
            self.setup_timer.destroy()
            self.setup_timer = self.create_timer(0.5, self.setup)

    def initiate_takeoff(self):

        takeoff_duration = Duration()
        takeoff_duration.sec = 2
        takeoff_duration.nanosec = 0
        takeoff_req = Takeoff.Request(height=self.z_desired,duration=takeoff_duration)
        self.takeoff_future = self.takeoff_client.call_async(takeoff_req)

    def attempt_land(self):

        # if self.takeoff_future.done():
        self.get_logger().info("Takeoff successful, preparing to land in 5 seconds...")
        self.state = VehicleState.LANDING

        land_duration = Duration()
        land_duration.sec = 2
        land_duration.nanosec = 0
        land_req = Land.Request(duration=land_duration)

        self.landing_future = self.land_client.call_async(land_req)
        self.setup_timer.destroy()
        self.setup_timer = self.create_timer(5.0, self.check_landing_status)

    def check_landing_status(self):

        if self.landing_future.done():
            self.get_logger().info("Vehicle has landed successfully.")
            self.state = VehicleState.LANDED
            self.setup_timer.destroy()
            self.setup_timer = self.create_timer(5.0, self.setup)

    def control_func(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    
    takeoff_node = TakeoffNode('cf80')
    rclpy.spin(takeoff_node)

    takeoff_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

