import rclpy
import utilities as ut
from random import choice
from string import ascii_uppercase

from std_msgs.msg import String
from rclpy.qos import qos_profile_default


class MinimalPublisher:

    def __init__(self, node):
        # Define all publishers
        self.str_publisher_ = node.create_publisher(String, '~/temp_str', qos_profile=qos_profile_default)
        timer_period = 1  # seconds

        # Define a new timer for each pub
        self.str_timer = node.create_timer(timer_period, self.str_timer_callback)

    def str_timer_callback(self):
        str_msg = String()
        str_msg.data = ''.join(choice(ascii_uppercase) for i in range(250000))
        self.str_publisher_.publish(str_msg)
        print('Publishing: ', len(str_msg.data))


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')

    minimal_publisher = MinimalPublisher(node)
    minimal_publisher  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
