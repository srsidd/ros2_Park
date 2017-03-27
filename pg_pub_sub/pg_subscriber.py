# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../src/playground/'))

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
# from src.libdecode import libdecode
from std_msgs.msg import String, Char, Int8, UInt8MultiArray
from steelcast_msgs.msg import CommandID, ServerResponse
# from sidewinder_msgs.msg import *


class MinimalSubscriber:

    def __init__(self, node):
        self.subscription = node.create_subscription(String, '~/boo', self.listener_callback, qos_profile_sensor_data)
        self._img_sub = node.create_subscription(UInt8MultiArray, '~/steelcast_compressed_img', self._encoded_img_callback, qos_profile_default)
        # self.srv_res_sub = node.create_subscription(ServerResponse, '~/steelcast_res', self.srv_res_cb, qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        # self.srv_res_sub

    def listener_callback(self, msg):
        # print('I heard: [%s]' % msg.stamp)
        # print('I heard: [%s]' % msg.cmd_id)
        # print('I heard: [%s]' % msg.recording_info.id)
        print("I heard - ", msg)

    def srv_res_cb(self, msg):
        print("I heard srv ress - ", msg)

    def _encoded_img_callback(self, msg):
        print("came to img cb")
        print(msg)

def main(args=None):
    rclpy.init(args)

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
