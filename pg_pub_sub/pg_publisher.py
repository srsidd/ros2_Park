import rclpy
import utilities as ut

from std_msgs.msg import String, ByteMultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from steelcast_msgs.msg import CommandID, ServerResponse
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

class MinimalPublisher:

    def __init__(self, node):
        # Define all publishers
        self.str_publisher_ = node.create_publisher(String, '~/steelcast_compressed_img', qos_profile_default)
        # self.publisher_ = node.create_publisher(ServerResponse, '~/steelcast_res', qos_profile_default)
        self.cmd_publisher_ = node.create_publisher(CommandID, '~/steelcast_cmd', qos_profile_sensor_data)
        # self.img_publisher_ = node.create_publisher(Image, 'topic', qos_profile_default)
        timer_period = 1  # seconds

        # Define a new timer for each pub
        # self.timer = node.create_timer(timer_period, self.timer_callback)
        # self.cmd_timer = node.create_timer(timer_period, self.cmd_timer_cb)
        # self.img_timer = node.create_timer(timer_period, self.img_timer_cb)

         # Define all cached messages
        # self.msg = ByteMultiArray()
        self.cmd_msg = CommandID()
        # self.img_msg = Image()

        # Define all message initializations
        # self.msg.layout.dim.append(MultiArrayDimension())
        self.cmd_timer_cb()
        # self.timer_callback()

    def img_timer_cb(self):
        self.img_msg.header.stamp = ut.get_current_time()
        self.img_msg.encoding = 'rgb'
        self.img_msg.height = 100
        self.img_msg.width = 100
        self.img_msg.step = 1
        self.img_publisher_.publish(self.img_msg)
        print("I sent = ", self.img_msg)

    def cmd_timer_cb(self):
        self.cmd_msg.cmd_id = 2
        # self.cmd_msg.recording_info.id = 0
        # self.cmd_msg.recording_info.format = 1
        # self.cmd_msg.playback_id = 0
        # self.cmd_msg.compression_info.effort = 5
        # self.cmd_msg.compression_info.quality = 5
        # self.cmd_msg.compression_info.scale_width = 1280
        # self.cmd_msg.compression_info.scale_height = 720
        self.cmd_publisher_.publish(self.cmd_msg)
        print("I sent = ", self.cmd_msg)

    def timer_callback(self):
        # self.msg.layout.dim[0].label = 'img'
        # self.msg.layout.dim[0].size = 3
        # self.msg.layout.dim[0].stride = 1
        # self.msg.data = [bytes(0)]
        # self.publisher_.publish(self.msg)
        # print('Publishing: "%s"' % self.msg.data)
        # str_msg = String()
        str_msg = String()
        str_msg.data = "Helloo"
        self.str_publisher_.publish(str_msg)
        print('Publishing: ', str_msg)
        print()
        # self.i += 2


def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('minimal_publisher')

    minimal_publisher = MinimalPublisher(node)
    minimal_publisher  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(node, 0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
