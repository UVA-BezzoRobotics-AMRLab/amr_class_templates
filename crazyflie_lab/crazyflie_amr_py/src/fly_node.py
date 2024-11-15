#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Arm

class FlyNode(Node):
    def __init__(self, cf_name = 'cf80'):
        super().__init__('fly_node')

        self.arm_client  = self.create_client(Arm, "/all/arm")
        self.vel_pub     = self.create_publisher(Twist, f'/{cf_name}/cmd_vel_legacy', 10)
        self.setup_timer = self.create_timer(5.0, self.setup)
        self.ctrl_timer  = None

        self.is_armed   = False # Crazyflie has been armed
        self.arm_future = None  # Future object for async service command
        
        self.fly_cmd = Twist()           # Thrust command
        self.fly_cmd.linear.z  = 0.0     # Initially send zero thrust
        self.fly_active_thrust = 55000.0 # Thrust command once active

        self.ctrl_period_s = 1.0/100.0  # 100 Hz

    def setup(self):
        if(self.is_armed):
            self.get_logger().info("Turning on Thrust")
            self.fly_cmd.linear.z = self.fly_active_thrust
            
            self.setup_timer.destroy()
        else:
            if(self.arm_future):
                if self.arm_future.done():
                    self.get_logger().info("Is Armed")
                    self.is_armed = True 
                    
                    self.ctrl_timer  = self.create_timer(self.ctrl_period_s, self.control_func)
                    
                    self.setup_timer.destroy()
                    self.setup_timer = self.create_timer(0.5, self.setup)
            else:
                self.get_logger().info("Sending Arming Requst")
                arm_req          = Arm.Request(arm=True)
                self.arm_future  = self.arm_client.call_async(arm_req)
                
                self.setup_timer.destroy()
                self.setup_timer = self.create_timer(0.1, self.setup)

    def control_func(self):
        self.vel_pub.publish(self.fly_cmd)

def main(args=None):
    rclpy.init(args=args)
    
    fly_node = FlyNode('cf80')
    rclpy.spin(fly_node)

    fly_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
