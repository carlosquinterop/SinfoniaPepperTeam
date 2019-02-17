#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sinfonia_pepper_tools_decisionmaking.msg import *
from sinfonia_pepper_tools_decisionmaking.srv import *
from sinfonia_pepper_tools_interaction.srv  import *
from std_msgs.msg import String
import sys
reload(sys)
sys.setdefaultencoding('utf-8')


class MakeOrder():
    def __init__(self):
        self.vec_clients = []
        self.vec_orders = []
        self.vec_orders2 = []
        self.vec_orders1 = []
        self.vec_ids = []
        self.vec_features = []
        self.orderMissing = []
        self.orderOptions = []
        self.confOrderOpt =[]

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


    def analyzeTxt(self, txt):
        rospy.wait_for_service('srv_classify_text')
        try:
            ask = rospy.ServiceProxy('srv_classify_text', classify_text)
            resp2 = ask(txt)
            print(txt)
            return resp2.affirmation,resp2.negation,resp2.clientName,resp2.clientOrder
        except:
            print ("Service call failed: %s")


    def Give_order_bar(self, str_order):
        rospy.wait_for_service('srvGiveOrder2Bar')
        try:
            add_two_ints4 = rospy.ServiceProxy('srvGiveOrder2Bar', give_order2bar)
            resp4 = add_two_ints4(str_order)
            self.vec_clients = resp4.msg_bar_order.name.split()
            self.vec_orders = resp4.msg_bar_order.order.split()
            self.vec_ids = resp4.msg_bar_order.personid.split()
            self.vec_features = resp4.msg_bar_order.features
            self.respAnsOpt = []
            print(self.vec_clients)
            print(self.vec_orders)
            print(self.vec_ids)
            print(self.vec_features)
            return resp4.msg_bar_order
        except rospy.ServiceException, e:
            print "Service call failedcccccccc: %s"%e


    def Make_order(self, str_objects):
        rospy.wait_for_service('srvValidateOrder')
        try:
            add_two_ints2 = rospy.ServiceProxy('srvValidateOrder', validate_order,persistent=False,headers=None)
            resp2 = add_two_ints2(str_objects)
            self.orderMissing = resp2.missingOrder
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def updateOrder(self, objs):
        rospy.wait_for_service('srvUpdateStates')
        try:
            add_two_ints2 = rospy.ServiceProxy('srvUpdateStates', update_states,persistent=False,headers=None)
            resp2 = add_two_ints2(objs)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def Additionals_order(self, order):
        rospy.wait_for_service('srvAddAdditionalProducts')
        try:
            add_two_ints = rospy.ServiceProxy('srvAddAdditionalProducts', additional_products,persistent=False,headers=None)
            resp5 = add_two_ints(order)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def detectObjects(self, object):
        rospy.wait_for_service('srvDetectObjects')
        try:
            add_two_ints = rospy.ServiceProxy('srvDetectObjects', detect_objects)
            resp5 = add_two_ints(object)
            self.vec_orders1 = resp5.objects
            return resp5.objects
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def talkWait(self):
        self.talk("Hola barman a continuación te diré los pedidos")
        for i in range(len(self.vec_clients)):
            self.vec_features2 = self.vec_features[i].split()
            if self.vec_features2[1] == "no_gafas":
                self.vec_features2[1] = "quien no tiene gafas "
            elif self.vec_features2[1] == "gafas":
                self.vec_features2[1] = "quien tiene gafas "
            elif self.vec_features2[1] == "gafas_sol":
                self.vec_features2[1] = "quien tiene gafas de sol "
            self.talk("El cliente " + self.vec_clients[i] + " quien tiene cabello " + self.vec_features2[0] + " " + self.vec_features2[1] + " y es " + self.vec_features2[2] + " desea " + self.vec_orders[i])


    def confMissing(self):
        if self.orderMissing == []:
            self.talk("Gracias Barman")
            return True
        else:
            for i in range(len(self.orderMissing)):
                resp = self.talkListen("Veo que falta el pedido de " + self.orderMissing[i] + ", ¿Es correcto?")
                confResp = self.analyzeTxt(resp+".")
                if confResp[1]:
                    print("uno")
                    self.updateOrder(True)
                    return False
                else:
                    print("DOS")
                    return True


    def asking4Options(self):
        self.orderOptions = self.talkListen("¿Que opciones le puedo ofrecer al cliente?")
        self.confOrderOpt = self.analyzeTxt(self.orderOptions)
        self.talk('Entendí que las opciones son:')
        for i in range(len(self.confOrderOpt[3])):
            self.talk(self.confOrderOpt[3][i])
        ansOpt = self.talkListen('¿Es correcto?')
        self.respAnsOpt = self.analyzeTxt(ansOpt)

    def start(self):
        self.Give_order_bar("x")
        self.talkWait()
        while True:
            Aff = self.talkListen("Esperaré a que esté listo")
            confAff = self.analyzeTxt(Aff + ".")
            if confAff[0]:
                break
        while True:
            print(self.detectObjects("x"))
            self.Make_order(self.vec_orders1)
            if self.confMissing():
                break
        while True:
            self.asking4Options()
            if self.respAnsOpt[0]:
                print(True)
                self.Additionals_order(self.confOrderOpt[3])
                break
            else:
                print(False)
