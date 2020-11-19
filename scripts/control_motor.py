#!/usr/bin/env python


import rospy
from threading import Thread, Lock
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import struct



class MS_ROS:
    def __init__(self):
        self.Vol_mes = 0   #Bytes 0-1
        self.Bat_mes = 0   #Bytes 2-3
        self.VMG_mes = 0   #Bytes 4-5
        self.VMD_mes = 0   #Bytes 6-7
        


mot_sens = MS_ROS()
MUT_mot_sens = Lock()

def RPM_to_PWM(RPM):
    a = 0.431
    b = 0
    PWM=a*RPM+b
    if PWM>25:
        PWM=25
    elif PWM<-25:
        PWM=-25
    return PWM


def asservissement(refDroit, refGauche):
    MUT_mot_sens.acquire()
    global mot_sens
    global kp
    global ki
    vitesseG = mot_sens.VMG_mes * 0.01   #RPM
    vitesseD = mot_sens.VMD_mes * 0.01   #RPM
    MUT_mot_sens.release()
    erreurDroit =refDroit-vitesseD
    print(vitesseD)
    erreurGauche =refGauche-vitesseG
    print(erreurDroit)
    global somme_erreurDroit
    somme_erreurDroit = somme_erreurDroit + erreurDroit
    global somme_erreurGauche
    somme_erreurGauche = somme_erreurGauche + erreurGauche
    
    #PI : calcul de la commande
    
    MUT_cmdDroit.acquire()
    global cmdDroit
    cmdDroit_RPM = kp*erreurDroit + ki*somme_erreurDroit
    cmdDroit = int(RPM_to_PWM(cmdDroit_RPM))
    print(cmdDroit)
    MUT_cmdDroit.release()
    
    MUT_cmdGauche.acquire()
    global cmdGauche
    cmdGauche_RPM = kp*erreurGauche + ki*somme_erreurGauche
    cmdGauche = int(RPM_to_PWM(cmdGauche_RPM))
    MUT_cmdGauche.release()
    
def callback(data):
    print('I heard ', data.data[2])
    MUT_mot_sens.acquire()
    global mot_sens
    mot_sens.VMG_mes = int(data.data[2])
    mot_sens.VMD_mes = int(data.data[3])
    MUT_mot_sens.release()
    asservissement(10,10)
    
somme_erreurDroit = 0    # Somme des erreurs pour l'integrateur
somme_erreurGauche = 0   # Somme des erreurs pour l'integrateur

cmdDroit = 0
cmdGauche = 0
MUT_cmdDroit = Lock()
MUT_cmdGauche = Lock()
kp=2          #Coefficient proportionnel
ki=1           #Coefficient integrateur

    
    
    
class MyTalker(Thread):

    def __init__(self):
        Thread.__init__(self)

    def run(self):
        pub = rospy.Publisher('/vitesse_asservie', Float32MultiArray, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        vect = Float32MultiArray()
        vect.layout.dim.append(MultiArrayDimension())
        vect.layout.dim[0].label = "height"
        vect.layout.dim[0].size = 2
        vect.layout.dim[0].stride = 2
        
        while not rospy.is_shutdown():
            MUT_cmdDroit.acquire()
            MUT_cmdGauche.acquire()
            global cmdDroit
            global cmdGauche
            vect.data = [cmdDroit, cmdGauche]    
            MUT_cmdDroit.release()
            MUT_cmdGauche.release()
            pub.publish(vect)
            rate.sleep()


    
def listener():
    rospy.init_node('Asservissement_node', anonymous=True)
    rospy.Subscriber('/mot_sens',Float32MultiArray , callback)

if __name__ == '__main__':
    listener()    
    newrostalker = MyTalker()
    newrostalker.start()
    #newthread.join()
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()



