#!/usr/bin/env python
#coding=utf-8
################airsim图像获取#########################
import airsim
# standard python
import numpy as np
# ROS
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#opencv
import cv2
#矩阵完全显示
np.set_printoptions(threshold = 1e6)



#Image
IMAGE_WIDTH = 144  # resolution should match values in settings.json
IMAGE_HEIGHT = 256
name_to_dtypes = {
	"rgb8":    (np.uint8,  3),
	"rgba8":   (np.uint8,  4),
	"rgb16":   (np.uint16, 3),
	"rgba16":  (np.uint16, 4),
	"bgr8":    (np.uint8,  3),
	"bgra8":   (np.uint8,  4),
	"bgr16":   (np.uint16, 3),
	"bgra16":  (np.uint16, 4),
	"mono8":   (np.uint8,  1),
	"mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
	"bayer_rggb8":	(np.uint8,  1),
	"bayer_bggr8":	(np.uint8,  1),
	"bayer_gbrg8":	(np.uint8,  1),
	"bayer_grbg8":	(np.uint8,  1),
	"bayer_rggb16":	(np.uint16, 1),
	"bayer_bggr16":	(np.uint16, 1),
	"bayer_gbrg16":	(np.uint16, 1),
	"bayer_grbg16":	(np.uint16, 1),

    # OpenCV CvMat types
	"8UC1":    (np.uint8,   1),
	"8UC2":    (np.uint8,   2),
	"8UC3":    (np.uint8,   3),
	"8UC4":    (np.uint8,   4),
	"8SC1":    (np.int8,    1),
	"8SC2":    (np.int8,    2),
	"8SC3":    (np.int8,    3),
	"8SC4":    (np.int8,    4),
	"16UC1":   (np.int16,   1),
	"16UC2":   (np.int16,   2),
	"16UC3":   (np.int16,   3),
	"16UC4":   (np.int16,   4),
	"16SC1":   (np.uint16,  1),
	"16SC2":   (np.uint16,  2),
	"16SC3":   (np.uint16,  3),
	"16SC4":   (np.uint16,  4),
	"32SC1":   (np.int32,   1),
	"32SC2":   (np.int32,   2),
	"32SC3":   (np.int32,   3),
	"32SC4":   (np.int32,   4),
	"32FC1":   (np.float32, 1),
	"32FC2":   (np.float32, 2),
	"32FC3":   (np.float32, 3),
	"32FC4":   (np.float32, 4),
	"64FC1":   (np.float64, 1),
	"64FC2":   (np.float64, 2),
	"64FC3":   (np.float64, 3),
	"64FC4":   (np.float64, 4)
}



def getDepthImage(response_d):
	img_depth = np.array(response_d.image_data_float, dtype=np.float32)
	img_depth = img_depth.reshape(response_d.height, response_d.width)
	print(img_depth)
	return img_depth


def CreateDMessage( img_depth):
	bridge = CvBridge()
	msg_d=Image()
	msg_d.encoding = "32FC1"
	msg_d.height = IMAGE_HEIGHT
	msg_d.width = IMAGE_WIDTH
	msg_d.data = bridge.cv2_to_imgmsg(img_depth, "32FC1").data

	return msg_d

def get_depth_image_messages(client):
    # get camera images from the multirotor

    responses = client.simGetImages([airsim.ImageRequest('0', airsim.ImageType.DepthPerspective, True,False)])


    # convert depth float array to NumPy 2D array using
    depth_img = getDepthImage(responses[0])


    # Populate image message

    depth_msg = CreateDMessage(depth_img)

    return depth_msg

def airpub():
    ## Start ROS ---------------------------------------------------------------
    rospy.init_node('airsim_img_publisher', anonymous=False)
    loop_rate = rospy.get_param('~loop_rate', 1)
    rate = rospy.Rate(loop_rate)


    ## Publishers --------------------------------------------------------------
    # image publishers

    depth_pub = rospy.Publisher("airsim/depth", Image, queue_size=1)

    ## Main --------------------------------------------------------------------
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    while not rospy.is_shutdown():


        depth_msg = get_depth_image_messages(client)


        # publish message
        depth_pub.publish(depth_msg)
        print("duileduilehahaahahahahh")
        rate.sleep()

if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass