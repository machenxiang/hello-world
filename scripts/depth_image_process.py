#!/usr/bin/env python
# coding=utf-8
# airsim
import airsim
# standard python
import math
import sys
import numpy as np

# ROS
import rospy
import tf2_ros
# ROS Image message
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from px4_command.msg import command

#设置模式
Idle=0
Takeoff=1
Move_ENU=2
Move_Body=3
Hold=4
Land=5
Disarm=6
Failsafe_land=7
#定义command对象
command_now=command()

yaw = 0
pi = 3.14159265483

vx = 0
vy = 0
driving = 0
help = False
global comid
comid = 0

pub = rospy.Publisher("/px4/command", command, queue_size=10)

command_now=command()

# Source: https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/image.py
name_to_dtypes = {
    "rgb8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "rgb16": (np.uint16, 3),
    "rgba16": (np.uint16, 4),
    "bgr8": (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "bgr16": (np.uint16, 3),
    "bgra16": (np.uint16, 4),
    "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8": (np.uint8, 1),
    "bayer_bggr8": (np.uint8, 1),
    "bayer_gbrg8": (np.uint8, 1),
    "bayer_grbg8": (np.uint8, 1),
    "bayer_rggb16": (np.uint16, 1),
    "bayer_bggr16": (np.uint16, 1),
    "bayer_gbrg16": (np.uint16, 1),
    "bayer_grbg16": (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1": (np.uint8, 1),
    "8UC2": (np.uint8, 2),
    "8UC3": (np.uint8, 3),
    "8UC4": (np.uint8, 4),
    "8SC1": (np.int8, 1),
    "8SC2": (np.int8, 2),
    "8SC3": (np.int8, 3),
    "8SC4": (np.int8, 4),
    "16UC1": (np.int16, 1),
    "16UC2": (np.int16, 2),
    "16UC3": (np.int16, 3),
    "16UC4": (np.int16, 4),
    "16SC1": (np.uint16, 1),
    "16SC2": (np.uint16, 2),
    "16SC3": (np.uint16, 3),
    "16SC4": (np.uint16, 4),
    "32SC1": (np.int32, 1),
    "32SC2": (np.int32, 2),
    "32SC3": (np.int32, 3),
    "32SC4": (np.int32, 4),
    "32FC1": (np.float32, 1),
    "32FC2": (np.float32, 2),
    "32FC3": (np.float32, 3),
    "32FC4": (np.float32, 4),
    "64FC1": (np.float64, 1),
    "64FC2": (np.float64, 2),
    "64FC3": (np.float64, 3),
    "64FC4": (np.float64, 4)
}


def depth_callback(ros_image):
    global comid
    bridge = CvBridge()
    # 使用cv_bridge转换ROS图像至OpenCV格式# 深度图像是单通道32位图像
    depth_image = bridge.imgmsg_to_cv2(ros_image, "32FC1")
    # print depth_image

    # img1d = np.array(depth_image.image_data_float, dtype=np.float)
    img1d = depth_image * 3.5 + 30
    img1d[img1d > 255] = 255
    img2d = np.reshape(img1d, (144, 256))
    depth = np.array(img2d, dtype=np.uint8)

    vx, vy= avoidance(depth)
    command_now.command = Move_Body;  # //机体系下移动
    command_now.comid = comid;
    comid = comid + 1;
    command_now.sub_mode = 2;  # xy 速度控制模式 z 位置控制模式
    command_now.vel_sp[0] = vx;
    command_now.vel_sp[1] = vy;
    command_now.pos_sp[2] = 0;
    command_now.yaw_sp = 0;
    pub.publish(command_now)
    rate1.sleep()



def avoidance(image):
    global yaw, vx, vy, vz, driving
    vz = 0
    top = np.vsplit(image, 2)[0]
    bands = np.hsplit(top, [50, 100, 150, 200])
    mines = [np.min(x) for x in bands]
    l2 = np.mean(np.hsplit(top, [50, 100, 150, 200])[0])
    l1 = np.mean(np.hsplit(top, [50, 100, 150, 200])[1])
    c = np.mean(np.hsplit(top, [50, 100, 150, 200])[2])
    r1 = np.mean(np.hsplit(top, [50, 100, 150, 200])[3])
    r2 = np.mean(np.hsplit(top, [50, 100, 150, 200])[4])
    dirction = [l2, l1, c, r1, r2]
    print(dirction)
    current = mines[2]
    print(current)

    if (current < 100):
        print(dirction)
        print("whoops - we are about to crash, so stopping and looking for path!")
        if (dirction.index(max(dirction)) == 0):
            change = -2 * pi / 10
        elif (dirction.index(max(dirction)) == 1):
            change = -pi / 10
        elif (dirction.index(max(dirction)) == 2):
            change = 0  # center strip, go straight
        elif (dirction.index(max(dirction)) == 3):
            change = pi / 10
        else:
            change = 2 * pi / 10
        print("change=", change)
        yaw = (yaw + change)
        vx = 1 * math.cos(yaw);
        vy = 1 * math.sin(yaw);
        print("switching angle", math.degrees(yaw), vx, vy)
        driving = dirction.index(max(dirction))
        x = int(driving * 50)
        cv2.rectangle(image, (x, 50), (x + 50, 100), (0, 255, 0), 2)
        cv2.imshow("Top", image)
        if (vx == 0 and vy == 0):
            vx = 1 * math.cos(yaw);
            vy = 1 * math.sin(yaw);


        return vx, vy

    else:
        change = 0
        yaw = (yaw + change)
        vx = 1 * math.cos(yaw);
        vy = 1 * math.sin(yaw);
        print("switching angle", math.degrees(yaw), "vx", vx, "vy", vy)
        if (vx == 0 and vy == 0):
            vx = 1 * math.cos(yaw);
            vy = 1 * math.sin(yaw);



        return vx, vy
    key = cv2.waitKey(1) & 0xFF;


def process_depth_image(frame):
    # Just return the raw image	for	this demo
    return frame

#
# def listener():
#     #rospy.init_node('depth_image_process', anonymous=True)
#
#     # Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
#     #rospy.Subscriber('airsim/depth', Image, depth_callback)
#     rospy.spin()


if __name__ == '__main__':
    sleep_time = 3
    rospy.init_node('depth_image_process', anonymous=True)
    rate1 = rospy.Rate(30)
    rate=rospy.Rate(1)
    a=input("please input 1 for takeoff:")
    if(a==1):
        print("takeoff")
        command_now.command = Takeoff
        pub.publish(command_now)
        print("complete takeoff")
        rate.sleep()
    a = input("please input 2 climb:")
    if(a==2):
        i=0
        print("climb")
        while(i<sleep_time):
        #>>>>>>>>>>>>>>>>>>>>>>>>爬升到5m>>>>>>>>>>>>>>>>>>>>>
            command_now.command = Move_Body;
            command_now.sub_mode = 0;#位置控制模式
            command_now.pos_sp[0] = 0;
            command_now.pos_sp[1] = 0;
            command_now.pos_sp[2] = 5;
            command_now.yaw_sp = 0;
            command_now.comid = comid;
            comid=comid+1;
            pub.publish(command_now)
            print("complete climb")
            i=i+1
        print("complete climb")
        rate.sleep()
    a = input("please input 3 move:")
    if(a==3):
        print("move")
        rospy.Subscriber('airsim/depth', Image, depth_callback)
        rospy.spin()







