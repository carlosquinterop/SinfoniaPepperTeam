#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sinfonia_pepper_tools_decisionmaking.msg import *
from sinfonia_pepper_tools_decisionmaking.srv import *
from sinfonia_pepper_tools_interaction.srv  import *
from std_msgs.msg import String
from sinfonia_pepper_tools_interaction.srv import FaceDetector
#from sinfonia_pepper_tools_interaction import TestFaceID
import os
import time
import tf
import sys
a=0
orderGlobal = ""
confAns = []
confResp = []
person_ok = False
def talkListen(askname):
    rospy.wait_for_service('srvListen')
    try:
        ask = rospy.ServiceProxy('srvListen', listen)
        resp1 = ask(askname)
        print(askname)
        return resp1.text_s2t
    except:
        print ("Service call failed: %s")


def analyzeName(name):
    rospy.wait_for_service('srv_classify_text')
    try:
        ask = rospy.ServiceProxy('srv_classify_text', classify_text)
        resp2 = ask(name)
        print(name)
        return resp2.affirmation,resp2.negation,resp2.clientName,resp2.clientOrder
    except:
        print ("Service call failed: %s")


def talk(x):

    rospy.wait_for_service('srvSpeak')
    try:
        send_text = rospy.ServiceProxy('srvSpeak', speak)
        resp2 = send_text(x)
        print("hablar %s",x)
        return resp2.result
    except:
        print ("Service call failed: %s")

def updateOrder(x, y):
    rospy.wait_for_service('srvUpdateOrder')
    try:
        add_two_ints = rospy.ServiceProxy('srvUpdateOrder', update_order,persistent=False,headers=None)
        resp1 = add_two_ints(x, y)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def Give_order(arg):
    global order
    rospy.wait_for_service('srvGiveOrder2Client')
    try:
        received_order = rospy.ServiceProxy('srvGiveOrder2Client', give_order2client)
        order = received_order(arg)
    except rospy.ServiceException:
        None


def detect_face(cvWind):
    try:
        detect_face_request = rospy.ServiceProxy('robot_face_detector',FaceDetector)
        is_face_in_Front = detect_face_request(cvWind)
        if is_face_in_Front.response:
            print("Si Hay cara")
            return True
        else:
            print("No hay cara")
            return False
    except rospy.ServiceException:
        print ("Error!! Make sure robot_face node is running ")


def memorize_face(name, cvWind):
    try:
        memorize_face_request = rospy.ServiceProxy('robot_face_memorize',FaceMemorize)
        azure_id = memorize_face_request(name, cvWind)
        print("azure_id = {}".format(azure_id))
    except rospy.ServiceException:
        print ("Error!! Make sure robot_face node is running ")


def recogniceFace():
    while 1:
        choice = 's'
        camera = 1
        if(choice==('s' or 'yes' or 'si' or 'S')):
            cvWindow = True
        else:
            cvWindow = False
        if_face = detect_face(cvWindow)
        if if_face:
            break

def entregarOpc():
    global confAns, confResp
    talk("Camilo, tu pedido no esta disponible, te puedo ofrecer las siguientes opciones")
    for i in range(len(order.additional_prod)):
        talk(order.additional_prod[i])
    respPedido = talkListen("¿Qué deseas ordenar")
    confResp = list(analyzeName(respPedido + "."))
    print(confResp)
    ans = talkListen("Entendí que quieres" + confResp[3][0] + "¿Es correcto?")
    confAns = analyzeName(ans + ".")

def callback(data):
    global orderGlobal, confResp, a
    photo = data.data
    print("hola hola hola",photo,a)
    while photo == "True" and a > 80:
        while True:
            Give_order("Bryan")
            if not order.order_state:
                while True:
                    entregarOpc()
                    if confAns[0]:
                        orderGlobal = confResp[3][0]
                        break
                    else:
                        confResp[3] = []
                updateOrder("Bryan",orderGlobal)
                photo = "False"
                a = 0
                pass
            else:
                talk("Tú pedido esta listo, puedes recogerlo en la barra")
                break
            break
        break
        a = 0
        print("asdfhjklñññññññ",photo,a)
    a = a + 1
    print("HELOSUI")

def sIAGiveOrdercalback():
    talk("Inicio Nodo Give Order")
    rospy.init_node('sIA_Give_order', anonymous=True)
    # tfListener = tf.TransformListener()
    rospy.Subscriber("person_detection", String, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        sIAGiveOrdercalback()
    except rospy.ROSInterruptException:
        pass
