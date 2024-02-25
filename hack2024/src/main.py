#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from control import Control
from std_msgs.msg import Int32, Float32, Bool
import numpy as np
from Util import PID
import RPi.GPIO as GPIO
DRIVE_PATH = 0
DRIVE_PARK = 1
DRIVE_MANU = 2


class Auto_car():
    def __init__(self):
        self.rate = rospy.Rate(50)
        self.drive = Control()
        self.driving_mode = DRIVE_PATH
        self.path_steer, self.path_speed = 0, 15
        self.park_steer, self.park_speed = 0, 0
        self.manu_steer, self.manu_speed = 0, 0 
        self.stop_sub = rospy.Subscriber("/stop_line", Bool, self.stop_cb)
        self.lane_sub = rospy.Subscriber('/lane_center', Float32, self.lane_cb)
        self.sonic_sub = rospy.Subscriber('/is_sonic', Bool, self.sonic_cb)
        self.steer_pub = rospy.Publisher('/steer', Float32, queue_size=1)
        self.sonic_pub = rospy.Publisher('/is_sonic',Bool,queue_size=1)
        self.stop_pub = rospy.Publisher('/stop_line/status', Int32,queue_size=1)
        self.stop_flag, self.stop_count, self.is_sonic = False, 0, False 
        self.lane_steer = 0 
        self.sonic_counter = 0 
        self.PID_steer = PID(1, 0, 0.01, 0.02)
        rospy.logwarn('main driver : wait for msg')
        rospy.wait_for_message("/stop_line", Bool)
        rospy.wait_for_message("/lane_center", Float32)
        rospy.wait_for_message("/is_sonic", Bool)
        rospy.logwarn('main driver : start driving')

    def sonic_cb(self, msg):
        self.is_sonic = msg.data
        
    def lane_cb(self, msg):
        self.lane_steer = self.PID_steer.do((msg.data - 320) / 500)

    def stop_cb(self, msg):
        self.stop_flag = (msg.data)
        
    def main(self):
        if self.driving_mode == DRIVE_PATH:
            steer, speed = self.lane_steer, self.path_speed 
        elif self.driving_mode == DRIVE_PARK:
            steer, speed = self.park_steer, self.park_speed
        elif self.driving_mode == DRIVE_MANU:
            steer, speed = self.manu_steer, self.manu_speed
        
        if self.stop_flag and self.stop_count == 0:
            self.stop_pub.publish(1) # stop
            for i in range(0, 500):
                self.drive.run(0, 0)
                self.rate.sleep()
            self.stop_pub.publish(2) # end
            steer, speed = 0, 0
            self.stop_count += 1
            
            self.path_speed = 10
        
        if self.sonic_counter==0 and (not self.is_sonic):
           speed = 0
        elif self.is_sonic:
           speed = 0
           self.sonic_counter += 1
        # if self.sonic_counter>=50:
        #        self.is_sonic=False

        #print(self.sonic_counter, self.is_sonic, self.stop_count, self.stop_flag)
        print("steer:{0}, speed:{1}".format(steer, speed))
        #print("stop : ", self.stop_count)
        self.drive.run(steer, -speed)
        self.steer_pub.publish(steer)
        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drive_car')
    car = Auto_car()
    
    while not rospy.is_shutdown():
        car.main()
