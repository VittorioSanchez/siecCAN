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

from threading import Thread, Lock
import time
import can
import os
import struct

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 6666              # Arbitrary non-privileged port

MCM = 0x010
MS = 0x100
US1 = 0x000
US2 = 0x001
OM1 = 0x101
OM2 = 0x102


speed = 0
mov = 0
ori = 0
ena_prop = 0
ena_steer = 0

MUT_speed = Lock()
MUT_mov = Lock()
MUT_ori = Lock()
MUT_ena_prop = Lock()
MUT_ena_steer = Lock()


class MyReceive(Thread):
    def __init__(self, bus):
        Thread.__init__(self)
        #self.conn = conn
        self.bus  = can.interface.Bus(channel='can0', bustype='socketcan_native')

        self.speed_cmd = 0
        self.movement = 0
        self.turn = 0
        self.enable_steering = 0
        self.enable = 0

    def run(self):
        self.speed_cmd = 0
        self.movement = 1
        self.turn = 0
        self.enable_steering = 0
        self.enable_speed = 1

        while True :

	    MUT_speed.acquire()
    	    self.speed_cmd = speed
    	    MUT_speed.release()
            """data = conn.recv(1024)

            if not data: break

            #for b in data:
            #    print(b)

            header = data[0:3]
            payload = data[3:]
            print("header :", header, "payload:", str(payload))

            if (header == b'SPE'):  # speed
                self.speed_cmd = int(payload)
                print("speed is updated to ", self.speed_cmd)
            elif (header == b'STE'):  # steering
                if (payload == b'left'):
                    self.turn = 1
                    self.enable_steering = 1
                    print("send cmd turn left")
                elif (payload == b'right'):
                    self.turn = -1
                    self.enable_steering = 1
                    print("send cmd turn right")
                elif (payload == b'stop'):
                    self.turn = 0
                    self.enable_steering = 0
                    print("send cmd stop to turn")
            elif (header == b'MOV'):  # movement
                if (payload == b'stop'):
                    self.movement = 0
                    self.enable_speed = 0
                    print("send cmd movement stop")
                elif (payload == b'forward'):
                    print("send cmd movement forward")
                    self.movement = 1
                    self.enable_speed = 1
                elif (payload == b'backward'):
                    print("send cmd movement backward")
                    self.movement = -1
                    self.enable_speed = 1"""

            print(self.speed_cmd)
            print(self.movement)
            print(self.enable)
            print(self.turn)
            print(self.enable_steering)

            if self.enable_speed:
                cmd_mv = (50 + self.movement*self.speed_cmd) | 0x80
            else:
                cmd_mv = (50 + self.movement*self.speed_cmd) & ~0x80

            if self.enable_steering:
                cmd_turn = 50 + self.turn*30 | 0x80
            else:
                cmd_turn = 50 + self.turn*30 & 0x80

            print("mv:",cmd_mv,"turn:",cmd_turn)

            msg = can.Message(arbitration_id=MCM,data=[cmd_mv, cmd_mv, cmd_turn,0,0,0,0,0],extended_id=False)

            #msg = can.Message(arbitration_id=0x010,data=[0xBC,0xBC,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
            #msg = can.Message(arbitration_id=MCM,data=[0xBC,0xBC,0x00, 0x00, 0x00, 0x00,0x00, 0x00],extended_id=False)
            #print(msg)
            self.bus.send(msg)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.linear.x)
    MUT_speed.acquire()
    speed = data.linear.x
    MUT_speed.release()
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    print('Bring up CAN0....')
    os.system("sudo /sbin/ip link set can0 up type can bitrate 400000")
    time.sleep(0.1)

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    except OSError:
        print('Cannot find PiCAN board.')
        exit()

    newthread = MyReceive( bus)
    newthread.start()
    newthread.join()

    listener()

