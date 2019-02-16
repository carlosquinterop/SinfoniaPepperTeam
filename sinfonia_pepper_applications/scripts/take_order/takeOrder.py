#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sinfonia_pepper_tools_decisionmaking.msg import *
from sinfonia_pepper_tools_decisionmaking.srv import *
from sinfonia_pepper_tools_interaction.srv  import *
from std_msgs.msg import String
import threading
import time
import os
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

a = 0

class TakeOrder():
    def __init__(self):
        self.confAns = []
        self.confName = []
        self.confOrder = []
        self.confClient = []
        self.nameGlobal = ""
        self.orderGlobal = ""
        self.features_person = []
        self.personID = ""
        print('jajaaj')

    def start(self):
        while True:
            if self.detect_face(True):
                while True:
                    self.pedirNombre()
                    print(self.confAns)
                    if self.confAns[0] == True:
                        self.nameGlobal = self.confName[2]
                        memorize_thread= threading.Thread(target=self.memorize_face,args=[self.nameGlobal,False])
                        memorize_thread.start()
                        print("segui")
                        print(self.features_person,self.personID)
                        break
                while True:
                    self.pedirPedido()
                    if self.confAns[0] == True:
                        self.orderGlobal = self.confOrder[3][0]
                        break
                    else:
                        self.confName[3] = []
                memorize_thread.join()
                self.addNewClient(self.nameGlobal, self.orderGlobal, self.personID, self.features_person)
                self.otroCliente()
                print(self.confClient[1])
                if self.confClient[1] == True:
                    break
        str_order = "x"
        print(self.Give_order_bar(str_order))
        # print("nombre",self.nameGlobal)
        # print("orden",self.orderGlobal)

    def talkListen(self, askname):
        rospy.wait_for_service('srvListen')
        try:
            ask = rospy.ServiceProxy('srvListen', listen)
            resp1 = ask(askname)
            print(askname)
            return resp1.text_s2t
        except:
            print ("Service call failed: %s")


    def talk(self, x):
        rospy.wait_for_service('srvSpeak')
        try:
            send_text = rospy.ServiceProxy('srvSpeak', speak)
            resp2 = send_text(x)
            print("hablar %s",x)
            return resp2.result
        except:
            print ("Service call failed: %s")


    def analyzeName(self, name):
        rospy.wait_for_service('srv_classify_text')
        try:
            ask = rospy.ServiceProxy('srv_classify_text', classify_text)
            resp2 = ask(name)
            print(name)
            return resp2.affirmation,resp2.negation,resp2.clientName,resp2.clientOrder
        except:
            print ("Service call failed: %s")


    def addNewClient(self, x, y ,w ,z):
        rospy.wait_for_service('srvAddClient')
        try:
            add_two_ints = rospy.ServiceProxy('srvAddClient', add_client,persistent=False,headers=None)
            resp1 = add_two_ints(x, y, w, z)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def Give_order(self, arg):
        rospy.wait_for_service('srvGiveOrder2Client')
        try:
            add_two_ints3 = rospy.ServiceProxy('srvGiveOrder2Client', give_order2client)
            resp3 = add_two_ints3(arg)
            return resp3.order_state, resp3.additional_prod
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def Give_order_bar(self, str_order):
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


    def pedirNombre(self):
        while True:
            askname = "¿Cuál es tú nombre?"
            name = self.talkListen(askname)
            self.confName = list(self.analyzeName(name + "."))
            if self.confName[2] != "":
                break
        ans = self.talkListen("Entendí que tú nombre es" + self.confName[2] + " , ¿Es correcto?")
        self.confAns = self.analyzeName(ans + ".")
        print(self.confName)


    def pedirPedido(self):
        if self.confName[3] == []:
            askOrder = "¿Qué deseas ordenar?"
            order = self.talkListen(askOrder)
            self.confOrder = self.analyzeName(order + ".")
        else:
            self.confOrder = self.confName
        ans = self.talkListen("Entendi que quieres " + self.confOrder[3][0] + " , ¿Es correcto?")
        self.confAns = self.analyzeName(ans + ".")


    def otroCliente(self):

        anotherClient = "¿Alguién más desea ordenar?"
        another = self.talkListen(anotherClient)
        self.confClient = self.analyzeName(another + ".")

    def detect_face(self, cvWind):
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

    def memorize_face(self, name, cvWind):
        print("entre al hilo")
        try:
            memorize_face_request = rospy.ServiceProxy('robot_face_memorize',FaceMemorize)
            azure_id = memorize_face_request(name, cvWind)
            self.features_person = azure_id.features.split(",")
            self.personID = azure_id.personId
        except rospy.ServiceException:
            print ("Error!! Make sure robot_face node is running ")
        print("termine el hilo")
