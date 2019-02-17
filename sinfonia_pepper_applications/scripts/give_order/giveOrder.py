#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sinfonia_pepper_tools_decisionmaking.msg import *
from sinfonia_pepper_tools_decisionmaking.srv import *
from sinfonia_pepper_tools_interaction.srv  import *
from std_msgs.msg import String
from rospy_message_converter import message_converter
import ast
from sinfonia_pepper_tools_interaction.srv import FaceDetector
#from sinfonia_pepper_tools_interaction import TestFaceID
import os
import time
import tf
import sys
a = 0
class GiveOrder():


    def __init__(self):
        self.orderGlobal = ""
        self.confAns = []
        self.confResp = []
        self.person_ok = False
        # self.a = 0
        self.order = None
        self.attributes_dict = None
        self.sizeClients = 3

    def talkListen(self, askname):
        rospy.wait_for_service('srvListen')
        try:
            ask = rospy.ServiceProxy('srvListen', listen)
            resp1 = ask(askname)
            print(askname)
            return resp1.text_s2t
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


    def verifySizeClients(self,h):
        rospy.wait_for_service('srvVerifyClients')
        try:
            ask = rospy.ServiceProxy('srvVerifyClients', verify_clients)
            respclients = ask(h)
            return respclients
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

    def updateOrder(self, x, y):
        rospy.wait_for_service('srvUpdateOrder')
        try:
            add_two_ints = rospy.ServiceProxy('srvUpdateOrder', update_order,persistent=False,headers=None)
            resp1 = add_two_ints(x, y)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def Give_order(self, arg):
        rospy.wait_for_service('srvGiveOrder2Client')
        try:
            received_order = rospy.ServiceProxy('srvGiveOrder2Client', give_order2client)
            self.order = received_order(arg)
        except rospy.ServiceException:
            None


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
        try:
            memorize_face_request = rospy.ServiceProxy('robot_face_memorize',FaceMemorize)
            azure_id = self.memorize_face_request(name, cvWind)
            print("azure_id = {}".format(azure_id))
        except rospy.ServiceException:
            print ("Error!! Make sure robot_face node is running ")


    def recogniceFace(self):
        while True:
            choice = 's'
            camera = 1
            if(choice==('s' or 'yes' or 'si' or 'S')):
                cvWindow = True
            else:
                cvWindow = False
            if_face = self.detect_face(cvWindow)
            if if_face:
                break

    def entregarOpc(self):
        self.talk(self.attributes_dict['name']+", tu pedido no esta disponible, te puedo ofrecer las siguientes opciones")
        for i in range(len(self.order.additional_prod)):
            self.talk(self.order.additional_prod[i])
        respPedido = self.talkListen("¿Qué deseas ordenar")
        self.confResp = list(self.analyzeName(respPedido + "."))
        print(self.confResp)
        try:
            ans = self.talkListen("Entendí que quieres" + self.confResp[3][0] + "¿Es correcto?")
        except:
            self.talk("No entendí lo que dijíste")
            self.confResp[0] = False
        self.confAns = self.analyzeName(ans + ".")
    def recognize_face(self, cvWind):
            try:
                recognize_face_request = rospy.ServiceProxy('robot_face_recognize',FaceRecognize)
                person = recognize_face_request(cvWind)
                #print('RESPONSE', person)
                attributes = message_converter.convert_ros_message_to_dictionary(person)
                self.attributes_dict = ast.literal_eval(attributes['features'])
                if self.attributes_dict:
                    print('RESPONSE', self.attributes_dict['name'])
                else:
                    print('No hay cara')
            except rospy.ServiceException:
                print ("Error!! Make sure robot_face node is running ")

    def start(self):
        global a
        #photo = True
        #print("photo",a,data)
        while self.detect_face(True):
            while True:
                self.recognize_face(False)
                if self.attributes_dict['name'] == "":
                    pass
                else:
                    self.Give_order(self.attributes_dict['name'])
                    if not self.order.order_state:
                        while True:
                            self.entregarOpc()
                            if self.confAns[0]:
                                self.orderGlobal = self.confResp[3][0]
                                teOrder(self.attributes_dict['name'], self.orderGlobal)
                                # photo =
                                time.sleep(3)
                                break
                            else:
                                self.confResp[3] = []
                        self.upda"False"
                        #a = 0
                        pass
                    else:
                        self.talk(self.attributes_dict['name']+", Tú pedido esta listo, puedes recogerlo en la barra")
                        time.sleep(3)
                        break
        self.sizeClients = self.verifySizeClients("h")
        print("Clientes Restantes: ", self.sizeClients)
        if self.sizeClients == 0:
            break
        #a = 0
        # print("asdfhjklñññññññ",photo,a)
    #a = a + 1
