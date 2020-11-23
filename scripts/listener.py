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
        self.cmd_MD = 0
        self.cmd_MG = 0

    def run(self):
        self.speed_cmd = 0
        self.turn = 0
        self.enable_steering = 1
        self.enable_speed = 1
        self.cmd_MD = 0
        self.cmd_MG = 0

        while True :

            global mot_cons
            mot_cons.MUT.acquire()
            self.speed_cmd = mot_cons.sp
            self.turn = mot_cons.tr
            mot_cons.MUT.release()
            self.cmd_MD, self.cmd_MG = asservissement(self.speed_cmd, self.speed_cmd)
            self.cmd_turn = asservissement_dir(self.turn)

            if self.enable_speed:
                cmd_mv_D = (50 + self.cmd_MD) | 0x80
                cmd_mv_G = (50 + self.cmd_MG) | 0x80
            else:
                cmd_mv_D = (50 + self.speed_cmd) & ~0x80
                cmd_mv_G = (50 + self.speed_cmd) & ~0x80
                 
            if self.enable_steering:
                cmd_turn = (50 + self.cmd_turn) | 0x80
            else:
                cmd_turn = (50 + self.turn) & 0x80

            print("mv:",(50 + self.speed_cmd),"turn:", cmd_turn)

            msg = can.Message(arbitration_id=MCM,data=[cmd_mv_D, cmd_mv_G, cmd_turn,0,0,0,0,0],extended_id=False)

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
                self.angle = (int(codecs.encode(msg.data[0:2],'hex'), 16)-1831)/19.45
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
    
#conversion de la vitesse (en rpm) vers la commande à envoyer
#Pour la marche avant
def RPM_to_PWM_AV(RPM): 
    a = 0.431
    b = 0
    PWM=a*RPM+b
    #fixation de seuils : 0<PWM<50 pour avancer
    #attention un +50 est appliqué dans run, pour coller aux spec du moteur 
    if PWM>50:
        PWM=50
    elif PWM<0:
        PWM=0
    return PWM

#conversion de la vitesse (en rpm) vers la commande à envoyer
#Pour la marche arrière
def RPM_to_PWM_AR(RPM):
    a = 0.431
    b = 0
    PWM=a*RPM+b
    #fixation de seuils : -50<PWM<0 pour reculer
    #attention un +50 est appliqué dans run, pour coller aux spec du moteur 
    if PWM>0:
        PWM=0
    elif PWM<-50:
        PWM=-50
    return PWM

kp=1         #Coefficient proportionnel
ki=0          #Coefficient integrateur
somme_erreurDroit = 0    # Somme des erreurs pour l'integrateur
somme_erreurGauche = 0   # Somme des erreurs pour l'integrateur
somme_erreurAngle = 0

kp_d=1      #Coefficient proportionnel direction


#fonction d'asservissement des moteurs roues arrières
#param refDroit et refGauche = ce qu'on veut en commande
#return cmdDroit et cmdGauche = ce que l'on envoit à l'actionneur
def asservissement(refDroit, refGauche):
    global mot_sens     #moteur sensor
    mot_sens.MUT.acquire()
    global kp
    global ki
    vitesseG = mot_sens.VMG_mes * 0.01   #RPM
    vitesseD = mot_sens.VMD_mes * 0.01   #RPM
    mot_sens.MUT.release()
    erreurDroit =refDroit-vitesseD
    print(vitesseD)
    erreurGauche =refGauche-vitesseG
    print(erreurDroit)
    global somme_erreurDroit
    somme_erreurDroit = somme_erreurDroit + erreurDroit
    global somme_erreurGauche
    somme_erreurGauche = somme_erreurGauche + erreurGauche
    
    #PI : calcul de la commande
    
    cmdDroit_RPM = kp*erreurDroit + ki*somme_erreurDroit
    cmdGauche_RPM = kp*erreurGauche + ki*somme_erreurGauche
    if (refDroit > 0):
        cmdDroit = int(RPM_to_PWM_AV(cmdDroit_RPM))
        cmdGauche = int(RPM_to_PWM_AV(cmdGauche_RPM))
    else:
        cmdDroit = int(RPM_to_PWM_AR(cmdDroit_RPM))
        cmdGauche = int(RPM_to_PWM_AR(cmdGauche_RPM))
    
    print(cmdDroit)
    #global cmdGauche

    
    #MUT_cmdGauche.release()
    return cmdDroit, cmdGauche 


#Pour la direction des roues avant
#à droite à fond => 30°=> 100 en PWM de base => 50 en PWM recentrée en 0
#à gauche à fond => -30°=> 0 en PWM de base => -50 en PWM recentrée en 0
#tout droit => 0° => 50 en PWM de base => 0 en PWM recentrée en 0
def Angle_to_PWM(Angle):
    a = 1.6667
    b = 0
    PWM=a*Angle+b
    if PWM>50:
        PWM=50
    elif PWM<-50:
        PWM=-50
    return PWM



#fonction d'asservissement des moteurs pour la direction des roues avant
#param refAngle = l'angle voulu, entre -30 et 30° (A VERIFIER)
#return cmdAngle = ce que l'on envoit à l'actionneur
def asservissement_dir(refAngle):
    global mot_sens
    mot_sens.MUT.acquire()
    global kp
    global ki
    angle = mot_sens.Vol_mes  #angle retourné par le capteur (en degré)
    mot_sens.MUT.release()
    erreurAngle =refAngle-angle
    print(erreurAngle)
    global somme_erreurAngle
    somme_erreurAngle = somme_erreurAngle + erreurAngle #erreur intégrale
    
    #PI : calcul de la commande
    
    cmdAngle_degre = kp_d*erreurAngle + ki*somme_erreurAngle
    #if (refAngle > 0):
    cmdAngle = int(Angle_to_PWM(cmdAngle_degre))
    #else:
    #    cmdAngle = int(Angle_to_PWM(cmdAngle_degre))
    
    print('cmdAngle',cmdAngle)
    
    return cmdAngle 




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
        
mot_cons = MCM_ROS() #mot_cons stands for "moteur consigne" (?)


class MS_ROS:
    def __init__(self):
        self.Vol_mes = 0   #Bytes 0-1 / Steering Wheel Angle
        self.Bat_mes = 0   #Bytes 2-3 / Battery Level
        self.VMG_mes = 0   #Bytes 4-5 / Left Motor Speed
        self.VMD_mes = 0   #Bytes 6-7 / Right Motor Speed
        self.MUT = Lock()

mot_sens = MS_ROS() #mot_sens stands for "motor sensor"


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
