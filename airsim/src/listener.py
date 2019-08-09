#!/usr/bin/env python

# Example ROS node for publishing AirSim images.

# AirSim Python API
import setup_path 
import airsim

import rospy

# ROS Image message
from sensor_msgs.msg import Image

# #回调函数输入的应该是msg
# def callback(gps):
#     distance = math.sqrt(math.pow(gps.x, 2)+math.pow(gps.y, 2)) 
#     rospy.loginfo('Listener: GPS: distance=%f, state=%s', distance, gps.state)
def callback(msg1):
    rospy.loginfo('msg.width %s:'%msg1.width)

def listener():
    rospy.init_node('listener', anonymous=True)
    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('airsim/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

