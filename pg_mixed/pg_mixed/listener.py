import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import String


class MinimalSubscriber:

    def __init__(self, node):
        self.subscription = node.create_subscription(String, 'topic', self.listener_callback, qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print('I heard: [%s]' % msg.data)


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('minimal_publisher')

    minimal_subscriber = MinimalSubscriber(node)
    minimal_subscriber  # prevent unused variable warning
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
