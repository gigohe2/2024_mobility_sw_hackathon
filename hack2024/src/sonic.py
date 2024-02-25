#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Bool

class Sonic:
    def __init__(self):
        rospy.loginfo('set sonic')
        GPIO.setmode(GPIO.BCM)
        self.trig_pin = 11
        self.echo_pin = 8
        self.is_sonic_pub = rospy.Publisher("/is_sonic", Bool, queue_size=1)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        rospy.loginfo('sonci')
    
    def obstacle_detection(self):
        try:
            #print("akakakak")
            
            GPIO.output(self.trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, GPIO.LOW)
            count = time.time()
            while GPIO.input(self.echo_pin) == GPIO.LOW and time.time()-count<0.1:
                count = time.time()
                #print("LOW")
            pulse_start = time.time()
            count=time.time()
            while GPIO.input(self.echo_pin) == GPIO.HIGH and time.time()-count<0.1:
                count=time.time()
                #print("HIGH")
            pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
            
            if distance <= 20:
                self.is_sonic_pub.publish(True)
                print(distance)
            else:
                self.is_sonic_pub.publish(False)
                print(distance)
        except:
            print("err")
    


if __name__ == '__main__':
    rospy.init_node('obstacle')
    sonic = Sonic()
    rate = rospy.Rate(10)  # 10Hz의 루프 주기 설정

    while (1):
        sonic.obstacle_detection()
        rate.sleep()
