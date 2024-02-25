#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32
import numpy as np

class Check:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, CompressedImage, self.callback)
        self.steer_sub = rospy.Subscriber('/steer', Float32, self.steer_cb)
        self.stop_sub = rospy.Subscriber("/stop_line", Bool, self.stop_cb)
        self.stopstat_sub = rospy.Subscriber("/stop_line/status", Int32, self.stopstat_cb)
        self.sonic_sub = rospy.Subscriber('/is_sonic', Bool, self.sonic_cb)
        self.vision_exp_pub = rospy.Publisher('/is_vision', Bool, queue_size=1)
        self.is_red_pub = rospy.Publisher('/is_red', Bool, queue_size=1)
        self.img, self.hsv_img, self.avg =None, None, None
        self.over_steer = False
        self.rate = rospy.Rate(10)
        self.debug = False
        self.is_stop, self.is_sonic, self.stop_status = False, False, False

    def stopstat_cb(self, msg):
        self.stop_status = True if msg.data==1 else False
        
    def sonic_cb(self, data):
        self.is_sonic = data.data

    def callback(self, data):
        self.img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.avg = np.mean(self.hsv_img[240, 300:340, :], axis=0)
        #print("medium :", (self.avg))

        
    def stop_cb(self, msg):
        self.is_stop = msg.data
        
    def steer_cb(self, data):
        self.over_steer = True if abs(data.data)>0.3 else False

    def check(self):
        lower_red_1 = (0, 70, 50)
        upper_red_1 = (10, 255, 255)

        lower_red_2 = (170, 70, 50)
        upper_red_2 = (180, 255, 255)

        in_range_1 = np.all((lower_red_1 <= self.avg) & (self.avg <= upper_red_1))
        in_range_2 = np.all((lower_red_2 <= self.avg) & (self.avg <= upper_red_2))
        in_range = in_range_1 or in_range_2

        if in_range:
            self.is_red_pub.publish(True)
        else:
            self.is_red_pub.publish(False)
        if self.over_steer or in_range or self.is_stop or self.is_sonic or self.stop_status:
            self.vision_exp_pub.publish(True)
        else:
            self.vision_exp_pub.publish(False)
        #print("str:{0}, red:{1}, stop:{2}, sonic:{3}".format(self.over_steer, in_range, self.is_stop, self.is_sonic))
        if self.debug:
            try:
                cv2.imshow('img', self.img)
                cv2.waitKey(1)
            except:
                print('none')

if __name__ == '__main__':
    rospy.init_node('check', anonymous=True)

    
    print("start")
    ch = Check('/camera_image_2')
    rospy.wait_for_message("/camera_image_2", CompressedImage)
    while not rospy.is_shutdown():
        ch.check()
