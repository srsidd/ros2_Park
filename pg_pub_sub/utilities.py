#!/usr/bin/env python

import time
from builtin_interfaces.msg import Time


'''
    Temporary method to get current time till ros2 rclpy comes up with it's own implementation yet.
    Can be removed once rclpy comes up with a working version of ROS time. 
'''
def get_current_time():
    current_time = Time()
    current_time.sec = int(time.time())
    current_time.nanosec = int((time.time() - current_time.sec) * 1000000000)
    return current_time
