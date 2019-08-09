#!/usr/bin/env python
#coding=utf-8
# airsim
import setup_path
import airsim
# standard python
import math
import sys
import numpy as np
from numpy.lib.stride_tricks import as_strided
# ROS
import rospy
import tf2_ros
# ROS Image message
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from message_filters import ApproximateTimeSynchronizer
import message_filters

client = airsim.MultirotorClient()
yaw = 0
pi = 3.14159265483
vx = 0
vy = 0
driving = 0
help = False



# Source: https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/image.py
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
def depth_callback(ros_image):
	bridge = CvBridge()
    # 使用cv_bridge转换ROS图像至OpenCV格式# 深度图像是单通道32位图像
	depth_image = bridge.imgmsg_to_cv2(ros_image, "32FC1")

	#depth_array = np.array(depth_image, dtype=np.float32)

	# cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
	# depth_display_image = process_depth_image(depth_array)
	#cv2.imshow("Depth	Image", depth_image)
	#cv2.imwrite('depth1.jpg',depth_image)
	avoidance(depth_image)

def avoidance(image):
	while True:

			# slice the image so we only check what we are headed into (and not what is down on the ground below us).

			# top = np.vsplit(gray, 2)[0]#将gray矩阵分成两部分[0]表示第一部分top(72,256)
			top = np.vsplit(image, 2)[0]
			# Split an array into multiple sub-arrays vertically (row-wise).
			# now look at 4 horizontal bands (far left, left, right, far right) and see which is most open.
			# the depth map uses black for far away (0) and white for very close (255), so we invert that
			# to get an estimate of distance.
			# 白色是远黑色是近
			bands = np.hsplit(top, [50, 100, 150, 200]);  # 按列拆分，分成列50，100，150，200list分为五组
			# maxes = [np.max(x) for x in bands]
			# min = np.argmin(maxes)
			# distance = 255 - maxes[min]
			mines = [np.min(x) for x in bands]
			l2 = np.mean(np.hsplit(top, [50, 100, 150, 200])[0])
			l1 = np.mean(np.hsplit(top, [50, 100, 150, 200])[1])
			c = np.mean(np.hsplit(top, [50, 100, 150, 200])[2])
			r1 = np.mean(np.hsplit(top, [50, 100, 150, 200])[3])
			r2 = np.mean(np.hsplit(top, [50, 100, 150, 200])[4])
			dirction = [l2, l1, c, r1, r2]
			current = mines[2]

			if (current < 25):
				client.moveByVelocityZAsync(0, 0, -1.5, 1, airsim.DrivetrainType.ForwardOnly,
											airsim.YawMode(False, 0)).join()
				pitch, roll, yaw = airsim.to_eularian_angles(client.simGetVehiclePose().orientation)
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
				cv2.rectangle(image, (x, 50), (x + 50, 100), (0, 255, 0), 2)  # 46行png
				cv2.imshow("Top", image)

				if (vx == 0 and vy == 0):
					vx = 1 * math.cos(yaw);
					vy = 1 * math.sin(yaw);

				# client.moveByVelocityZAsync(vx, vy, -1.5, 1, airsim.DrivetrainType.ForwardOnly,airsim.YawMode(False, 0)).join()
				client.moveByVelocityZAsync(vx, vy, -1.5, 1, airsim.DrivetrainType.ForwardOnly,
											airsim.YawMode(True, 0)).join()
			else:
				change = 0
				yaw = (yaw + change)
				vx = 1 * math.cos(yaw);
				vy = 1 * math.sin(yaw);
				print("switching angle", math.degrees(yaw), "vx", vx, "vy", vy)

				if (vx == 0 and vy == 0):
					vx = 1 * math.cos(yaw);
					vy = 1 * math.sin(yaw);

				client.moveByVelocityZAsync(vx, vy, -1.5, 1, airsim.DrivetrainType.ForwardOnly,
											airsim.YawMode(False, 0)).join()

			key = cv2.waitKey(1) & 0xFF;
			if (key == 27 or key == ord('q') or key == ord('x')):
				break;

def process_depth_image(frame):
    # Just return the raw image	for	this demo
    return frame

def listener():
	rospy.init_node('listener', anonymous=True)

    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
	rospy.Subscriber('airsim/depth', Image, depth_callback)
	rospy.spin()

if __name__ == '__main__':
    listener()