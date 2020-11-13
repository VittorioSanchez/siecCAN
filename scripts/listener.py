#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

from threading import Thread, Lock
import time
import can
import os
import struct
import codecs
from ctypes import *

MCM = 0x010
MS = 0x100
US1 = 0x000
US2 = 0x001
OM1 = 0x101
OM2 = 0x102



class MyReceive(Thread):
    def __init__(self, bus):
        Thread.__init__(self)
        #self.conn = conn
        self.bus  = can.interface.Bus(channel='can0', bustype='socketcan_native')

        self.speed_cmd = 0
        self.turn = 0
        self.enable_steering = 0
        self.enable = 0

    def run(self):
        self.speed_cmd = 0
        self.turn = 0
        self.enable_steering = 1
        self.enable_speed = 1

        while True :

            global mot_cons
            mot_cons.MUT.acquire()
            self.speed_cmd = mot_cons.sp
            self.turn = mot_cons.tr
            mot_cons.MUT.release()


            if self.enable_speed:
                cmd_mv = (50 + self.speed_cmd) | 0x80
            else:
                cmd_mv = (50 + self.speed_cmd) & ~0x80

            if self.enable_steering:
                cmd_turn = self.turn | 0x80
            else:
                cmd_turn = self.turn & 0x80

            print("mv:",(50 + self.speed_cmd),"turn:", self.turn)

            msg = can.Message(arbitration_id=MCM,data=[cmd_mv, cmd_mv, cmd_turn,0,0,0,0,0],extended_id=False)

            try:
                self.bus.send(msg)
                print("Message sent")
            except can.CanError:
                print("Message NOT sent")
            
            time.sleep(0.1)
            
class MySend(Thread):

    def __init__(self, bus):
        Thread.__init__(self)
        self.bus = bus
        
        self.bat = 0.0
        self.angle = 0.0
        self.speed_left = 0.0
        self.speed_right = 0.0

    def run(self):
        while True :
            msg = self.bus.recv()

            #print(msg.arbitration_id, msg.data)
            st = ""

            if msg.arbitration_id == MS:
                # position volant
                self.angle = int(codecs.encode(msg.data[0:2],'hex'), 16)
                # Niveau de la batterie
                
                self.bat = ((int(codecs.encode(msg.data[2:4],'hex'), 16)*(3.3/0.20408))/4095)
                # vitesse roue gauche
                self.speed_left = int(codecs.encode(msg.data[4:6],'hex'), 16)/100
                # vitesse roue droite
                # header : SWR payload : entier, *0.01rpm
                self.speed_right= int(codecs.encode(msg.data[6:8],'hex'), 16)/100
                
                print("angle: ",self.angle,";bat: ",self.bat,";sp-left: ",self.speed_left,"; sp-right: ",self.speed_right)
                
            global mot_sens
            mot_sens.MUT.acquire()
            mot_sens.Bat_mes = self.bat
            mot_sens.Vol_mes = self.angle
            mot_sens.VMG_mes = self.speed_left
            mot_sens.VMD_mes = self.speed_right
            mot_sens.MUT.release()
    


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.linear.x)
    print('I heard %d', data.linear.x)
    global mot_cons
    mot_cons.MUT.acquire()
    mot_cons.sp = int(data.linear.x)
    mot_cons.tr = int(data.angular.z)
    mot_cons.MUT.release()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, callback)
 
class MyTalker(Thread):

    def __init__(self):
        Thread.__init__(self)

    def run(self):
        pub = rospy.Publisher('/mot_sens', Float32MultiArray, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        vect = Float32MultiArray()
        vect.layout.dim.append(MultiArrayDimension())
        vect.layout.dim[0].label = "height"
        vect.layout.dim[0].size = 4
        vect.layout.dim[0].stride = 4
        
        while not rospy.is_shutdown():
            global mot_sens
            mot_sens.MUT.acquire()
            vect.data = [mot_sens.Vol_mes, mot_sens.Bat_mes, mot_sens.VMG_mes, mot_sens.VMD_mes]    
            mot_sens.MUT.release()
            pub.publish(vect)
            rate.sleep()



class MCM_ROS:
    def __init__(self):
        self.sp = 0
        self.tr = 50
        self.MUT = Lock()
mot_cons = MCM_ROS()


class MS_ROS:
    def __init__(self):
        self.Vol_mes = 0   #Bytes 0-1
        self.Bat_mes = 0   #Bytes 2-3
        self.VMG_mes = 0   #Bytes 4-5
        self.VMD_mes = 0   #Bytes 6-7
        self.MUT = Lock()

mot_sens = MS_ROS()


if __name__ == '__main__':
    
    
    listener()
    #ifconfig can0 txqueuelen 1000
    print('Bring up CAN0....')
    #os.system("sudo /sbin/ip link set can0 up type can bitrate 400000")
    time.sleep(0.1)

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
        print('Cannot find PiCAN board.')
        exit()

    newthread = MyReceive(bus)
    newthread.start()
    newsend = MySend(bus)
    newsend.start()
    
    newrostalker = MyTalker()
    newrostalker.start()
    #newthread.join()

    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
