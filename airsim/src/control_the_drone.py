#!/usr/bin/env python
#coding=utf-8
# airsim
import rospy
import setup_path
import airsim
from my_demo.msg import velocity

client = airsim.MultirotorClient()
def callback(velocity):
    vx=velocity.vx
    vy=velocity.vy
    rospy.loginfo('Listener: vx: %f,vy: %f ', velocity.vx, velocity.vy)
    client.moveByVelocityZAsync(vx, vy, -1.5, 1, airsim.DrivetrainType.ForwardOnly,airsim.YawMode(False, 0)).join()
def listener():
    rospy.init_node('listener_control', anonymous=True)
    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('velocity', velocity, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()