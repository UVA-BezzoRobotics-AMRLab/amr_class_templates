import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class TTBController(Node):

    def __init__(self):
        super().__init__('ttb_turn')

        # setup publisher for Twist msg to /TTB01/cmd_vel with buffer size = 10
        self.publisher_ = self.create_publisher(
            Twist,
            '/TTB01/cmd_vel',
            10
        )

        # setup subscriber to Imu msg from /TTB01/imu with buffer size = 10
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscriber_ = self.create_subscription(
            Imu,
            '/TTB01/imu',
            self.imu_callback,
            qos_profile
        )

        # setup controller to run at 10hz (period=.1s) and call method controller_callback
        timer_period=.1
        self.timer = self.create_timer(timer_period, self.controller_callback)

        # when count >=20, stop moving vehicle
        self.count = 0

    def controller_callback(self):
        # create msg which makes TTB speed .2 m/s and angular velocity 1.0rad/s
        # if count >=20, stop moving

        msg = Twist()
        if self.count < 20:
            msg.linear.x = .2
            msg.angular.z = 1.0
            self.count += 1

        self.publisher_.publish(msg)

    def imu_callback(self, msg):
        # print angular velocity from imu message to console
        ang_vel = msg.angular_velocity.z
        self.get_logger().info(f'Angular velocity = {ang_vel}')

def main(args=None):
    rclpy.init(args=args)

    ttb_controller = TTBController()

    rclpy.spin(ttb_controller)

    ttb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
