#!/usr/bin/env python
import sys
import rospy
from sinfonia_pepper_tools_interaction.srv import *

def callback_master(x):

    rospy.wait_for_service('srvSpeak')
    try:
        send_text = rospy.ServiceProxy('srvSpeak', speak)
        resp2 = send_text(x)
        print("hablar %s",x)
        return resp2.result
    except:
        print ("Service call failed: %s")

def callback_master2(x):

    rospy.wait_for_service('srvListen')
    try:
        send_text = rospy.ServiceProxy('srvListen', listen)
        resp2 = send_text(x)
        print("escuchar %s",x)
        return resp2.text_s2t
    except:
        print ("Service call failed: %s")

if __name__ == "__main__":
    print('elija 1 si desea reproducir o 2 si desea grabar ')
    opcion= raw_input()
    if opcion=='1':
        x="hola, que hace"
        print ("Requesting %s"%(x))
        callback_master(x)
        #print ("Returning [%s]"%(add_two_ints_client(x)))
    elif opcion=='2':
        x='Que quieres tomar'
        print ("Requesting %s"%(x))
        #callback_master1(x)
        print ("entendi [%s]"%(callback_master2(x)))
