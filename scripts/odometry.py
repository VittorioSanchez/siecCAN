#!/usr/bin/env python

import time, threading
from threading import Thread, Lock
import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray
from math import *

class Pose:
    def __init__(self):
        self.x = 0  
        self.y = 0     
        self.theta = 0   
        self.MUT = Lock()
        
POSE=Pose()

class MS_ROS:
    def __init__(self):
        self.steering_angle = 0  #Bytes 0-1 / Steering Wheel Angle
        self.batt_level = 0      #Bytes 2-3 / Battery Level
        self.motor_speed_L = 0   #Bytes 4-5 / Left Motor Speed
        self.motor_speed_R = 0   #Bytes 6-7 / Right Motor Speed
        self.MUT = Lock()
        
MOTOR_SENSORS = MS_ROS()


########ATTENTION AUX CONVERSIONS#############
#sp en RPM 
#st en radians
PERIOD_ODOMETRY = 0.050 #en secondes
time_old = 0
def update_odometry () :
    global POSE
    global MOTOR_SENSORS
    global time_old 
    L = 0.58 
    
    time_new = time.clock()    
    delta_t = time_new - time_old
    time_old = time_new 
    
    POSE.MUT.acquire()
    position_x = POSE.x  
    position_y = POSE.y  
    position_theta = POSE.theta  
    POSE.MUT.release() 
     
    #basic velocity inputs
    MOTOR_SENSORS.MUT.acquire()
    speed = MOTOR_SENSORS.motor_speed_L*0.63/60.0                  #m/s from rear wheels
    steering_angle =  MOTOR_SENSORS.steering_angle*3.14/180.0        #radians
    MOTOR_SENSORS.MUT.release()

    #compute odometry values from joint angles
    #first get the theta update
    delta_theta = (speed/L)*sin(steering_angle)*delta_t
    
    #compute odometry update values
    delta_x = speed*cos(position_theta+steering_angle)*delta_t
    delta_y = speed*sin(position_theta+steering_angle)*delta_t

    #now update our pose estimate
    POSE.MUT.acquire()
    POSE.x = position_x + delta_x
    POSE.y = position_y + delta_y
    POSE.theta = position_theta + delta_theta
    POSE.MUT.release()
    threading.Timer(PERIOD_ODOMETRY, update_odometry).start()
    
def show_Pose ():
    global POSE
    POSE.MUT.acquire()
    print ('POSE : ')
    print ('x :', POSE.x)
    print ('y :', POSE.y)
    print ('theta :', POSE.theta)
    POSE.MUT.release()
    threading.Timer(1, show_Pose).start()
    
def show_Ultrasound ():
    global ULTRASONIC_SENSORS2
    ULTRASONIC_SENSORS2.MUT.acquire()
    print('Rear Ultrasound ', ULTRASONIC_SENSORS1.rearCentralUltr, ' cm')
    ULTRASONIC_SENSORS2.MUT.release()
    threading.Timer(1, show_Ultrasound).start()
    
def listener():
    rospy.init_node('odometry', anonymous=True)
    rospy.Subscriber('/motor_sensors', Float32MultiArray, callback_sensor_motor)
    rospy.Subscriber('/ultrasonic_sensors1', Float32MultiArray, callback_sensor_ultrasound1)
    rospy.Subscriber('/ultrasonic_sensors2', Float32MultiArray, callback_sensor_ultrasound2)
    
    
    
def callback_sensor_motor(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.linear.x)
    #print('I heard speed :', data.data[2], 'direction ', data.data[0])
    global MOTOR_SENSORS
    MOTOR_SENSORS.MUT.acquire()
    MOTOR_SENSORS.steering_angle = data.data[0]
    MOTOR_SENSORS.motor_speed_L = data.data[2]
    MOTOR_SENSORS.motor_speed_R = data.data[3]
    MOTOR_SENSORS.MUT.release()
    
class US1_ROS:
    def __init__(self):
        self.frontLeftUltr = 0  #Bytes 0-1 / Front Left Ultrasonic US_AVG
        self.frontRightUltr = 0      #Bytes 2-3 / Front Right Ultrasonic US_AVD
        self.rearCentralUltr = 0   #Bytes 4-5 / Central Rear Ultrasonic US_ARCa
        self.MUT = Lock()

class US2_ROS:
    def __init__(self):
        self.rearLeftUltr= 0  #Bytes 0-1 / Left Rear Ultrasonic US_ARG
        self.rearRightUltr = 0      #Bytes 2-3 / Right Rear Ultrasonic US_ARD
        self.frontCentralUltr = 0   #Bytes 4-5 / Central Front Ultrasonic US_AVC
        self.MUT = Lock()

ULTRASONIC_SENSORS1 = US1_ROS()
ULTRASONIC_SENSORS2 = US2_ROS()
def callback_sensor_ultrasound1(data):
    #OK
    global ULTRASONIC_SENSORS1
    ULTRASONIC_SENSORS1.MUT.acquire()
    ULTRASONIC_SENSORS1.frontLeftUltr = data.data[0]
    ULTRASONIC_SENSORS1.frontRightUltr = data.data[1]
    ULTRASONIC_SENSORS1.rearCentralUltr = data.data[2]
    ULTRASONIC_SENSORS1.MUT.release()
    
def callback_sensor_ultrasound2(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.linear.x)
    #print('I heard speed :', data.data[2], data.data[1], data.data[0])
    global ULTRASONIC_SENSORS2
    ULTRASONIC_SENSORS2.MUT.acquire()
    ULTRASONIC_SENSORS2.rearLeftUltr = data.data[0]
    ULTRASONIC_SENSORS2.rearRightUltr = data.data[1]
    ULTRASONIC_SENSORS2.frontCentralUltr = data.data[2]
    #print (data.data[0])
    ULTRASONIC_SENSORS2.MUT.release()

    
    
    
if __name__ == '__main__':
    listener()
    update_odometry()
    show_Pose()
    show_Ultrasound()
    
    
    
  
