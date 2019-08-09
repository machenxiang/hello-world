# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 17:45:50 2019

@author: 12438
"""

# use open cv to show new images from AirSim 


import airsim
import cv2
import time
import math
import sys
import numpy as np
from PIL import Image
import os

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
client.moveToPositionAsync(1, 2, -1.5, 1)
time.sleep(4)

# you must first press "1" in the AirSim view to turn on the depth capture

# get depth image
yaw = 0
pi = 3.14159265483
vx = 0
vy = 0
driving = 0
help = False

i = 1
while True:

    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)])
    if (responses == "\0"):
        if (not help):
            help = True
            print("Please press '1' in the AirSim view to enable the Depth camera view")
    # 图片大小144*256=36864
    else:
        img1d = np.array(responses[0].image_data_float, dtype=np.float)
        img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
        img2d = np.reshape(img1d, (responses[0].height, responses[0].width))

        image = np.invert(np.array(Image.fromarray(img2d.astype(np.uint8), mode='L')))

        factor = 10
        maxIntensity = 255.0  # depends on dtype of image data

        # Decrease intensity such that dark pixels become much darker, bright pixels become slightly dark
        newImage1 = (maxIntensity) * (image / maxIntensity) ** factor
        result = np.array(newImage1, dtype=np.uint8)
        cv2.imwrite(('depth %d.jpg')%i,newImage1)
        # slice the image so we only check what we are headed into (and not what is down on the ground below us).

        # top = np.vsplit(gray, 2)[0]#将gray矩阵分成两部分[0]表示第一部分top(72,256)
        top = np.vsplit(result, [50, 100])[1]
        # Split an array into multiple sub-arrays vertically (row-wise).
        # now look at 4 horizontal bands (far left, left, right, far right) and see which is most open.
        # the depth map uses black for far away (0) and white for very close (255), so we invert that
        # to get an estimate of distance.
        # 白色是远黑色是近
        bands = np.hsplit(top, [50, 100, 150, 200]);  # 按列拆分，分成列50，100，150，200list分为五组
        l2 = np.mean(np.hsplit(top, [50, 100, 150, 200])[0])
        l1 = np.mean(np.hsplit(top, [50, 100, 150, 200])[1])
        c = np.mean(np.hsplit(top, [50, 100, 150, 200])[2])
        r1 = np.mean(np.hsplit(top, [50, 100, 150, 200])[3])
        r2 = np.mean(np.hsplit(top, [50, 100, 150, 200])[4])
        # print('l2',l2)
        # print('l1', l1)
        # print('c',c)
        # print('r1',r1)
        # print('r2', r2)
        dirction = [l2, l1, c, r1, r2]
        if (min(dirction) < 30):
            print('we are about to crash')
            if (max(dirction) == l2):
                change = -2 * pi / 10
            elif (max(dirction) == l1):
                change = -pi / 10
            elif (max(dirction) == c):
                change = 0  # center strip, go straight
            elif (max(dirction) == r1):
                change = pi / 10
            else:
                change = 2 * pi / 10

            yaw = (yaw + change)
            vx = 3 * math.cos(yaw);
            vy = 3 * math.sin(yaw);
            print("switching angle", math.degrees(yaw), vx, vy)

            if (vx == 0 and vy == 0):
                vx = 3 * math.cos(yaw);
                vy = 3 * math.sin(yaw);

            client.moveByVelocityZAsync(vx, vy, -1.5, 1, airsim.DrivetrainType.ForwardOnly,
                                        airsim.YawMode(False, 0)).join()
        else:
            change = 0
            yaw = (yaw + change)
            vx = 3 * math.cos(yaw);
            vy = 3 * math.sin(yaw);
            print("switching angle", math.degrees(yaw), vx, vy)

            if (vx == 0 and vy == 0):
                vx = 3 * math.cos(yaw);
                vy = 3 * math.sin(yaw);

            client.moveByVelocityZAsync(vx, vy, -1.5, 1, airsim.DrivetrainType.ForwardOnly,
                                        airsim.YawMode(False, 0)).join()
        driving = dirction.index(max(dirction))
        x = int(driving * 50)
        cv2.rectangle(result, (x, 50), (x + 50, 100), (0, 255, 0), 2)  # 46行png
        # 第一个参数：png是原图
        # （x，y）是矩阵的左上点坐标
        # x+w，y+h）是矩阵的右下点坐标
        # （0,255,0）是画线对应的rgb颜色
        # 2是所画的线的宽度
        cv2.imshow("Top", result)

        key = cv2.waitKey(1) & 0xFF;
        if (key == 27 or key == ord('q') or key == ord('x')):
            break;
