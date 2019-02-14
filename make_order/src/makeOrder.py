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
vec_clients = []
vec_orders = []
vec_orders2 = []
vec_orders1 = []
vec_ids = []
vec_features = []
orderMissing = []
orderOptions= []
confOrderOpt=[]

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


def analyzeTxt(txt):
    rospy.wait_for_service('srv_classify_text')
    try:
        ask = rospy.ServiceProxy('srv_classify_text', classify_text)
        resp2 = ask(txt)
        print(txt)
        return resp2.affirmation,resp2.negation,resp2.clientName,resp2.clientOrder
    except:
        print ("Service call failed: %s")


def Give_order_bar(str_order):
    global vec_clients, vec_orders, vec_ids, vec_features
    rospy.wait_for_service('srvGiveOrder2Bar')
    try:
        add_two_ints4 = rospy.ServiceProxy('srvGiveOrder2Bar', give_order2bar)
        resp4 = add_two_ints4(str_order)
        vec_clients = resp4.msg_bar_order.name.split()
        vec_orders = resp4.msg_bar_order.order.split()
        vec_ids = resp4.msg_bar_order.personid.split()
        vec_features = resp4.msg_bar_order.features
        print(vec_clients)
        print(vec_orders)
        print(vec_ids)
        print(vec_features)
        return resp4.msg_bar_order
    except rospy.ServiceException, e:
        print "Service call failedcccccccc: %s"%e

def Make_order(str_objects):
    global orderMissing
    rospy.wait_for_service('srvValidateOrder')
    try:
        add_two_ints2 = rospy.ServiceProxy('srvValidateOrder', validate_order,persistent=False,headers=None)
        resp2 = add_two_ints2(str_objects)
        orderMissing = resp2.missingOrder
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def updateOrder(objs):
    global orderMissing
    rospy.wait_for_service('srvUpdateStates')
    try:
        add_two_ints2 = rospy.ServiceProxy('srvUpdateStates', update_states,persistent=False,headers=None)
        resp2 = add_two_ints2(objs)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def Additionals_order(order):
    rospy.wait_for_service('srvAddAdditionalProducts')
    try:
        add_two_ints = rospy.ServiceProxy('srvAddAdditionalProducts', additional_products,persistent=False,headers=None)
        resp5 = add_two_ints(order)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def detectObjects(object):
    global vec_orders1,resp5
    rospy.wait_for_service('srvDetectObjects')
    try:
        add_two_ints = rospy.ServiceProxy('srvDetectObjects', detect_objects)
        resp5 = add_two_ints(object)
        vec_orders1 = resp5.objects
        return resp5.objects

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def talkWait():
    talk("Hola barman a continuación te diré los pedidos")
    for i in range(len(vec_clients)):
        vec_features2 = vec_features[i].split()
        if vec_features2[1] == "no_gafas":
            vec_features2[1] = "quien no tiene gafas "
        elif vec_features2[1] == "gafas":
            vec_features2[1] = "quien tiene gafas "
        elif vec_features2[1] == "gafas_sol":
            vec_features2[1] = "quien tiene gafas de sol "
        talk("El cliente " + vec_clients[i] + " quien tiene cabello " + vec_features2[0] + " " + vec_features2[1] + " y es " + vec_features2[2] + " desea " + vec_orders[i])


def confMissing():
    if orderMissing == []:
        talk("Gracias Barman")
        return True
    else:
        for i in range(len(orderMissing)):
            resp = talkListen("Veo que falta el pedido de " + orderMissing[i] + ", ¿Es correcto?")
            confResp = analyzeTxt(resp+".")
            if confResp[1]:
                print("uno")
                updateOrder(True)
                return False
            else:
                print("DOS")
                return True


def asking4Options():
    global confOrderOpt, respAnsOpt
    orderOptions = talkListen("¿Que opciones le puedo ofrecer al cliente?")
    confOrderOpt=analyzeTxt(orderOptions)
    talk('Entendí que las opciones son:')
    for i in range(len(confOrderOpt[3])):
        talk(confOrderOpt[3][i])
    ansOpt = talkListen('¿Es correcto?')
    respAnsOpt = analyzeTxt(ansOpt)



if __name__ == "__main__":
    Give_order_bar("x")
    talkWait()
    while True:
        Aff = talkListen("Esperaré a que esté listo")
        confAff = analyzeTxt(Aff + ".")
        if confAff[0]:
            break
    while True:
        print(detectObjects("x"))
        Make_order(vec_orders1)
        if confMissing():
            break
    while True:
        asking4Options()
        if respAnsOpt[0]:
            print(True)
            Additionals_order(confOrderOpt[3])
            break
        else:
            print(False)
