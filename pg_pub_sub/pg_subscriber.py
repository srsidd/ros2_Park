import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../src/playground/'))

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import String


class MinimalSubscriber:

    def __init__(self, node):
        self.subscription = node.create_subscription(String, '~/temp_str', self.listener_callback, qos_profile=qos_profile_default)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print("I heard - ", len(msg.data))


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')

    minimal_subscriber = MinimalSubscriber(node)
    minimal_subscriber  # prevent unused variable warning
    while rclpy.ok():
        rclpy.spin_once(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
