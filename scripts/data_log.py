#!/usr/bin/env python
"""
Simple script to export ROS data and time to files in order to process it after with Excel (graphs...).
The created files are "data_file.txt" and "time.txt".
Change the topic you want to suscribe in listener().
Be careful to choose the right data in the callback function. It's only one value per data file.
If you want to record various type of data at the same time, you need to create a file for each type of data.
"""
import csv
import rospy
import time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist 
from std_msgs.msg import MultiArrayDimension
valeur=[]
temps=[]

data_file = open('data_file.txt', mode='w')
cmd_file = open('cmd_file.txt', mode='w')
time_file = open('time.txt', mode='w')

cmd = 0

def callback_cmd(data):

    #print('Time : ', 100*time.clock())
    global cmd
    cmd = data.angular.z
    
def callback(data):

    #print('Time : ', 100*time.clock())
    data_file.write("{}\n".format(data.data[0]))
    global cmd
    cmd_file.write("{}\n".format(cmd))
    time_file.write("{}\n".format(time.clock()))

     
    
def listener():
    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber('/motor_sensors', Float32MultiArray, callback)
    rospy.Subscriber('/speed_cmd', Twist, callback_cmd)
    
if __name__ == '__main__':    
    listener()
    rospy.spin()
    