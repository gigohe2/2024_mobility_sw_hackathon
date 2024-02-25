#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Bool
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class StopLineDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.stopline_num = 0
        self.is_red = False
        self.Green = (0,255,0)
        self.stopline_detected_times = 0  # 추가된 코드: 조건이 성립하는 횟수를 세는 변수
        rospy.init_node('stopline_detector', anonymous=True)
        self.start_time = rospy.Time.now()
        rospy.Subscriber("/camera_image_2", CompressedImage, self.check_stopline)
        rospy.Subscriber('/is_red', Bool, self.red_cb)
        self.stopline_pub = rospy.Publisher("/stop_line", Bool, queue_size=1)
        rospy.logwarn('stopline_detector: wait for bottom image')
        rospy.wait_for_message("/camera_image_2", CompressedImage)
        rospy.logwarn('stopline_detector: begin')
        rospy.spin()

    def red_cb(self, data):
        self.is_red = data.data

    def check_stopline(self, data):
        # 이미지 변환
        image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

        # 원본 영상의 특정영역(ROI Area)을 잘라내기
        roi_img = image[100:200, 0:640]
       

        # HSV 포맷으로 변환하고 V채널에 대해 범위를 정해서 흑백이진화 이미지로 변환
        hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV) 
        sensitivity = 15
        upper_white = np.array([255, sensitivity, 255])
        lower_white = np.array([0, 0, 255-sensitivity])
        binary_img = cv2.inRange(hsv_image, lower_white, upper_white)

        # 흑백이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
        stopline_check_img = binary_img[0:100, 0:640] 

        # 흑백이진화 이미지를 칼라이미지로 바꾸고 정지선 체크용 이미지 영역을 녹색사각형으로 표시
        img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(img, (200,100),(440,120),self.Green,3)
        # cv2.imshow('Stopline Check', img)
        # cv2.imshow("ROI Image", roi_img)

        # cv2.imshow('check', stopline_check_img)

        # cv2.waitKey(1)

        # 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
        stopline_count = cv2.countNonZero(stopline_check_img)
       
        print(stopline_count, self.is_red, self.stopline_num)
        # 사각형 안의 흰색 점이 
        # 기준치 이상이면 정지선을 발견한 것으로 한다
        if (stopline_count > 20000) & (not self.is_red):
            #print("Stopline Found...! -", self.stopline_num)
            self.stopline_num = self.stopline_num + 1
            self.stopline_detected_times += 1
            elapsed_time = rospy.Time.now() - self.start_time
            if elapsed_time.to_sec() < 0.3:  # 1.0은 조건을 확인하는 시간 간격이며, 필요에 따라 조절 가능합니다.
                if self.stopline_detected_times > 3:  # 3은 조건이 성립해야 하는 횟수이며, 필요에 따라 조절 가능합니다.
                    self.stopline_pub.publish(True)
            else:
                self.start_time = rospy.Time.now()
                self.stopline_detected_times = 0
                self.stopline_pub.publish(False)
        else:
            self.stopline_pub.publish(False)

if __name__ == '__main__':
    try:
        detector = StopLineDetector()
    except rospy.ROSInterruptException:
        pass
