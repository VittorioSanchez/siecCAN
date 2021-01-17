#!/usr/bin/env python

import time, threading
import os
from threading import Thread, Lock
import rospy
import tf
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import *


class Pose:
    def __init__(self):
        self.x = 0  
        self.y = 0     
        self.theta = 0   
        self.MUT = Lock()
        
POSE=Pose()

class Velocity:
    def __init__(self):
        self.vx = 0  
        self.vy = 0     
        self.vtheta = 0   
        self.MUT = Lock()
        
VELOCITY = Velocity()

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
PERIOD_ODOMETRY = 0.000050 #en secondes
time_old = 0
class update_odometry (Thread) :
    def __init__(self):
        Thread.__init__(self)
    
    def run(self):
        global POSE
        global MOTOR_SENSORS
        global time_old 
        global VELOCITY
        L = 0.58 
        while (True):
            time_new = time.clock()    
            delta_t = time_new - time_old
            #print (delta_t)
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
            
            #compute velocities
            v_x = delta_x / delta_t
            v_y = delta_y / delta_t
            v_theta = delta_theta / delta_t
            
            #affect the velocity to the VELOCITY class
            VELOCITY.MUT.acquire()
            VELOCITY.vx = v_x 
            VELOCITY.vy = v_y 
            VELOCITY.vtheta = v_theta 
            VELOCITY.MUT.release()
            
            #update our pose estimate
            position_x = position_x + delta_x
            position_y = position_y + delta_y
            position_theta = position_theta + delta_theta
            
            #affect the update to the POSE class
            POSE.MUT.acquire()
            POSE.x = position_x 
            POSE.y = position_y 
            POSE.theta = position_theta 
            POSE.MUT.release()
            #time.sleep (PERIOD_ODOMETRY)
            #threading.Timer(PERIOD_ODOMETRY, update_odometry).start()
            
        
    
def show_Pose ():
    global POSE
    POSE.MUT.acquire()
    print ('POSE : ')
    print ('x :', POSE.x)
    print ('y :', POSE.y)
    print ('theta :', POSE.theta)
    POSE.MUT.release()
    threading.Timer(1, show_Pose).start()
    
    
def listener():
    
    rospy.Subscriber('/motor_sensors', Float32MultiArray, callback_sensor_motor)

    
class odometry_publisher(Thread):
    def __init__(self):
        Thread.__init__(self)
    
    def run(self):
        global POSE
        global VELOCITY
        

        
        #odometry message & publisher
        odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        #odom_broadcaster = tf.TransformBroadcaster()
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            POSE.MUT.acquire()
            position_x = POSE.x  
            position_y = POSE.y  
            position_theta = POSE.theta  
            POSE.MUT.release() 
            
            VELOCITY.MUT.acquire()
            v_x = VELOCITY.vx 
            v_y = VELOCITY.vy 
            v_theta = VELOCITY.vtheta
            VELOCITY.MUT.release()
            #odom_quat = tf.transformations.quaternion_from_euler(0, 0, position_theta)
            #print (odom_quat)

            # first, we'll publish the transform over tf
            '''odom_broadcaster.sendTransform(
                (position_x, position_y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )'''

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            #odom.pose.pose = Pose(Point(position_x, position_y, 0.0), [0,0,0,0])

            
            odom.pose.pose.position.x = position_x
            odom.pose.pose.position.y = position_y
            odom.pose.pose.position.z = 0.0

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(v_x, v_y, 0), Vector3(0, 0, v_theta))

            # publish the message
            odom_pub.publish(odom)
            rate.sleep()

    
    
def callback_sensor_motor(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.linear.x)
    #print('I heard speed :', data.data[2], 'direction ', data.data[0])
    global MOTOR_SENSORS
    MOTOR_SENSORS.MUT.acquire()
    MOTOR_SENSORS.steering_angle = data.data[0]
    MOTOR_SENSORS.motor_speed_L = data.data[2]
    MOTOR_SENSORS.motor_speed_R = data.data[3]
    MOTOR_SENSORS.MUT.release()
 

    
if __name__ == '__main__':
    rospy.init_node('odometry_computation', anonymous=True)
    listener()
    newThread = update_odometry()
    newThread.start()
    newrostalker = odometry_publisher()
    newrostalker.start()
    show_Pose()
    
    
    # show_Ultrasound()
    
    
    
  
