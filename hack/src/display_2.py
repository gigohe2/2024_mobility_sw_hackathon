#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32
import cv2
from cv_bridge import CvBridge
import numpy as np


class Display():
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.top_sub = rospy.Subscriber('/camera_image', CompressedImage, self.top_cb)
        self.bot_sub = rospy.Subscriber('/camera_image_2', CompressedImage, self.bot_cb)
        self.face_sub = rospy.Subscriber('/face_y', Int32, self.face_cb)
        self.change_sub = rospy.Subscriber('/is_vision', Bool, self.change_cb)
        self.bridge = CvBridge()
        self.top_img, self.bot_img = None, None
        self.dp_img = None
        self.vision_exp = False
        self.stop_status = False
        self.cte = 0

        self.black_img = cv2.resize(cv2.imread('/home/jun/catkin_ws/src/hack/src/dp.jpg'), (640, 480))

    def face_cb(self, msg):
        self.cte = msg.data - 240
        
    def top_cb(self, msg):
        self.top_img = cv2.flip(self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8"), 1)
        
    def bot_cb(self, msg):
        self.bot_img = cv2.flip(self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8"), 1)
    
    def change_cb(self, msg):
        self.vision_exp = msg.data

    def display(self):
        if self.vision_exp:
            self.dp_img = np.vstack([self.top_img, self.bot_img])
        else:
            self.dp_img = np.vstack([self.top_img, self.black_img])

        # 960 x 480
        center_y = int(np.interp(self.cte, [-200, 200], [-50, 50])) + 480
        up, down = center_y-430, center_y+430
    
        new = self.dp_img[up:down, :, :]
        
        print(up, down, np.shape(new))
        
        cv2.imshow('display', self.dp_img)
        cv2.imshow('new', new)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('display')
    rospy.loginfo("init display")
    dp = Display()
    rospy.wait_for_message('/camera_image', CompressedImage)
    rospy.wait_for_message('/camera_image_2', CompressedImage)
    rospy.loginfo("start display")

    while not rospy.is_shutdown():
        dp.display()
        dp.rate.sleep()

        
    