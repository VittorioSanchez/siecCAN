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
from std_msgs.msg import MultiArrayDimension
valeur=[]
temps=[]
i=0
data_file = open('data_file.txt', mode='w')
time_file = open('time.txt', mode='w')
    
def callback(data):
    global i      # i is used to close properly the files after a the defined delay, here 10000 callbacks (it corresponds to something like 10s, it's not precise)
    if i<10000:
        i=i+1
        #print(i)
        #print('Time : ', 100*time.clock())
        data_file.write("{}\n".format(data.data[3]))
        time_file.write("{}\n".format(time.clock()))
    else:
        data_file.close()
        time_file.close()
     
    
def listener():
    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber('/motor_sensors', Float32MultiArray, callback)
    
if __name__ == '__main__':    
    listener()
    rospy.spin()
    