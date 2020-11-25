#!/usr/bin/env python
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
data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.linear.x)
    global i
    if i<10000:
        i=i+1
        print(i)
        print('Time : ', 100*time.clock())
        #valeur.append(data.data[3])
        #data_writer.writerow([data.data[3]])
        data_file.write("{}\n".format(data.data[3]))
        time_file.write("{}\n".format(time.clock()))
    else:
        data_file.close()
        time_file.close()
    

    
    
def listener():
    rospy.init_node('listener2', anonymous=True)

    rospy.Subscriber('/mot_sens', Float32MultiArray, callback)
    
if __name__ == '__main__':    
    listener()
    rospy.spin()
    