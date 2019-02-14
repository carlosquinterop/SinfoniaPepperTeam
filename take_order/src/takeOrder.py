#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sinfonia_pepper_tools_decisionmaking.msg import *
from sinfonia_pepper_tools_decisionmaking.srv import *
from sinfonia_pepper_tools_interaction.srv  import *
from std_msgs.msg import String
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
confAns = []
confName = []
confOrder = []
confClient = []
nameGlobal = ""
orderGlobal = ""
features_person = []
personID = ""
a = 0


def talkListen(askname):
    rospy.wait_for_service('srvListen')
    try:
        ask = rospy.ServiceProxy('srvListen', listen)
        resp1 = ask(askname)
        print(askname)
        return resp1.text_s2t
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


def analyzeName(name):
    rospy.wait_for_service('srv_classify_text')
    try:
        ask = rospy.ServiceProxy('srv_classify_text', classify_text)
        resp2 = ask(name)
        print(name)
        return resp2.affirmation,resp2.negation,resp2.clientName,resp2.clientOrder
    except:
        print ("Service call failed: %s")


def addNewClient(x, y ,w ,z):
    rospy.wait_for_service('srvAddClient')
    try:
        add_two_ints = rospy.ServiceProxy('srvAddClient', add_client,persistent=False,headers=None)
        resp1 = add_two_ints(x, y, w, z)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def Give_order(arg):
    rospy.wait_for_service('srvGiveOrder2Client')
    try:
        add_two_ints3 = rospy.ServiceProxy('srvGiveOrder2Client', give_order2client)
        resp3 = add_two_ints3(arg)
        return resp3.order_state, resp3.additional_prod
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def Give_order_bar(str_order):
    vec_clients = []
    vec_orders = []
    rospy.wait_for_service('srvGiveOrder2Bar')
    try:
        add_two_ints4 = rospy.ServiceProxy('srvGiveOrder2Bar', give_order2bar)
        resp4 = add_two_ints4(str_order)
        vec_clients = resp4.msg_bar_order.name.split()
        vec_orders = resp4.msg_bar_order.order.split()
        print(vec_clients)
        print(vec_orders)
        return resp4.msg_bar_order
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def pedirNombre():
    global confAns, confName
    while True:
        askname = "¿Cuál es tú nombre?"
        name = talkListen(askname)
        confName = list(analyzeName(name + "."))
        if confName[2] != "":
            break
    ans = talkListen("Entendí que tú nombre es" + confName[2] + " , ¿Es correcto?")
    confAns = analyzeName(ans + ".")
    print(confName)


def pedirPedido():
    global confAns, confOrder, confName
    if confName[3] == []:
        askOrder = "¿Qué deseas ordenar?"
        order = talkListen(askOrder)
        confOrder = analyzeName(order + ".")
    else:
        confOrder = confName
    ans = talkListen("Entendi que quieres " + confOrder[3][0] + " , ¿Es correcto?")
    confAns = analyzeName(ans + ".")


def otroCliente():
    global confClient
    anotherClient = "¿Alguién más desea ordenar?"
    another = talkListen(anotherClient)
    confClient = analyzeName(another + ".")

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
    global features_person,personID
    try:
        memorize_face_request = rospy.ServiceProxy('robot_face_memorize',FaceMemorize)
        azure_id = memorize_face_request(name, cvWind)
        features_person = azure_id.features.split(",")
        personID = azure_id.personId
    except rospy.ServiceException:
        print ("Error!! Make sure robot_face node is running ")


if __name__ == "__main__":
    while True:
        if detect_face(False):
            while True:
                pedirNombre()
                print(confAns)
                if confAns[0] == True:
                    nameGlobal = confName[2]
                    memorize_face(nameGlobal, False)
                    print(features_person,personID)
                    break
            while True:
                pedirPedido()
                if confAns[0] == True:
                    orderGlobal = confOrder[3][0]
                    break
                else:
                    confName[3] = []
            addNewClient(nameGlobal, orderGlobal, personID, features_person)
            otroCliente()
            print(confClient[1])
            if confClient[1] == True:
                break
    str_order = "x"
    print(Give_order_bar(str_order))
    # print("nombre",nameGlobal)
    # print("orden",orderGlobal)
