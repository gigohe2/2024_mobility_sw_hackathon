#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
import time  
import numpy as np 
import RPi.GPIO as GPIO
import Adafruit_PCA9685

# motor pin number (BCM)
Motor_A_EN = 4
Motor_B_EN = 17
Motor_A_Pin1 = 21  
Motor_A_Pin2 = 26
Motor_B_Pin1 = 18
Motor_B_Pin2 = 27

class Control():
    def __init__(self):
        self.Motor_A_EN=4
        self.Motor_B_EN=17
        self.Motor_A_Pin1=21
        self.Motor_A_Pin2=26
        self.Motor_B_Pin1=18
        self.Motor_B_Pin2=27
        self.trig_pin = 11
        self.echo_pin = 8
        self.servoMin=150
        self.servoMax=550
        self.robot_handle=Adafruit_PCA9685.PCA9685()
        self.is_sonic_pub = rospy.Publisher("/is_sonic", Bool, queue_size=1)
        self.setup()
    ## motor
    def setup(self):
        global pwm_A  
        global pwm_B
        GPIO.setwarnings(False)  
        GPIO.setmode(GPIO.BCM)  
        GPIO.setup(self.Motor_A_EN, GPIO.OUT)  
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)  
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)  
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)  
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)    
 
        self.motorStop()  
        try:  
            pwm_A = GPIO.PWM(self.Motor_A_EN, 1000)  
            pwm_B = GPIO.PWM(self.Motor_B_EN, 1000)
        except:  
            pass
            

    def motor_A(self,direction, speed):
        if direction == 1:
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == 0:
            GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        else:
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)

    def motor_B(self,direction, speed):
        if direction == 1:
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == 0:
            GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        else:
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)

    def destroy(self):
        self.motorStop()
        GPIO.cleanup()

    def m_F_B(self,vel):
        if vel>=0:
            self.motor_A(1,vel)
            self.motor_B(1,vel)
        elif vel<0:
            self.motor_A(0,abs(vel))
            self.motor_B(0,abs(vel))

    def motorStop(self):
        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_A_EN, GPIO.LOW)
        GPIO.output(self.Motor_B_EN, GPIO.LOW)
    

    ## steering
    def map(self,value, min_angle, max_angle, min_pulse, max_pulse):
	    angle_range = max_angle-min_angle
	    pulse_range = max_pulse-min_pulse
	    scale_factor=float(angle_range)/float(pulse_range)
	    return min_pulse+(value/scale_factor)
	
    def set_angle(self,angle):
        # servo : 30 ~ 150
        # -1 ~ 1

        angle = int(np.interp(angle, [-1, 1], [30, 150]))
        pulse = int(self.map(angle, 0, 180, self.servoMin, self.servoMax))
        self.robot_handle.set_pwm(0, 0, pulse)

    def run(self, steer, speed):
        self.m_F_B(speed)
        self.set_angle(steer)
        #("steer : {0}, speed : {1}".format(steer, speed))


            

    
if __name__=='__main__':
    my_robot=Control()
    while True:
        
        my_robot.run(0,100)
        my_robot.set_angle(0.1)
        
    