#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseArray

class GapReader(Node):

    def __init__(self):
        super().__init__('gap_reader')

        # setup subscriber to Imu msg from /TTB01/imu with buffer size = 10
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscriber_ = self.create_subscription(
            PoseArray,
            '/TTB06/gaps',
            self.gap_callback,
            qos_profile
        )

    def gap_callback(self, msg):
        gapList = msg.poses

        for i in range(0,len(gapList)-1, 2):
            p1 = gapList[i].position
            p2 = gapList[i+1].position
            
            print(f'gap {i//2}:\n\tstart: ({p1.x}, {p1.y})')
            print(f'\tend: ({p2.x}, {p2.y})')

def main(args=None):
    rclpy.init(args=args)

    gap_reader = GapReader()

    rclpy.spin(gap_reader)

    gap_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()