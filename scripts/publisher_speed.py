#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Twist 

def talker():
    pub = rospy.Publisher('/speed_cmd', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    move_cmd = Twist() #create a twist message
    #move_cmd.linear.x = 1.0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        #move_cmd = Twist()
        move_cmd.linear.x = input("Valeur  vitesse (float): ")
        #move_cmd.angular.z=-30
        #rospy.loginfo(hello_str)
        pub.publish(move_cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass