#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sinfonia_pepper_tools_decisionmaking.msg import *
from sinfonia_pepper_tools_decisionmaking.srv import *
import rospy
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

# vecClients=[{'name':"Bryan",'order':"gaseosa",'order_state':False, 'personId':"fdjwekejdj", 'features':["black","noglasses","male","21"]},### Vector de prueba
#         {'name':"Andrea",'order':"café",'order_state':False, 'personId':"iuytrdcvbnmjuytg", 'features':["black","eyesglasses","female","20"]},
#         {'name':"Camilo",'order':"vino",'order_state':False, 'personId':"iuytrfvjhgf", 'features':["brown","noglasses","male","19"]}]

dic_drinks = {'bottle':["gaseosa", "cerveza"] , \
              'vase':["agua"] , \
              'cup':["cafe","café","tinto","te","té"] , \
              'wine glass':["copa de vino","vino"]}
dic_features = {'black':"negro" , \
              'brown':"cafe" , \
              'red':"rojo" , \
              'noglasses':"no_gafas" , \
              'male':"hombre" , \
              'female':"mujer" , \
              'blond':"rubio" , \
              'sunglasses':"gafas_sol" , \
              'eyesglasses':"gafas"}
vecClients = []
vecObjects = []
orderMissing = []
str_additional_order = ["cerveza", "gaseosa"]
global  str_order
state = False


def addClient(reqNameProduct):
    "Anadir nombre del cliente, perdido y almacenarlo."
    global vecClients
    dict = {'name':reqNameProduct.name, 'order':reqNameProduct.order, 'order_state':False,
            'personId':reqNameProduct.personid, 'features':reqNameProduct.features}
    vecClients.append(dict)
    print ("\n",vecClients )


def translate():
    for a in vecClients:
        for b in range(len(a['features'])-1):
            for c in dic_features:
                if a['features'][b] == c:
                    a['features'][b] = dic_features[c]


def giveOrder2Bar(reqOrders):
    "Entregar nombre del cliente y pedido al bar"
    translate()
    if reqOrders.ret_orders == "x":
        name  = ""
        order = ""
        personid = ""
        features = len(vecClients)*['']
        a = 0
        for h in vecClients:
            name  = name  + h['name']  + " "
            order = order + h['order'] + " "
            personid = personid + h['personId'] + " "
            for i in range(len(h['features'])):
                features[a] = features[a] + h['features'][i] + " "
            a = a + 1
        bar_order.name  = name
        bar_order.order = order
        bar_order.personid = personid
        bar_order.features = features
        print(features)
        return give_order2barResponse(bar_order)


def validateOrder(reqProducts):
    "Validacion de objetos entregados por el barman"
    global vecClients, vecObjects, orderMissing
    orderMissing = []
    vecObjects = reqProducts.objects
    verifyOrders()
    for i in range(len(vecClients)):
        if vecClients[i]['order_state'] == False:
            orderMissing.append(vecClients[i]['name'])
    print(orderMissing)
    return validate_orderResponse(orderMissing)


def updateStates(reqnames):
    "Validacion de objetos entregados por el barman"
    global vecClients, vecObjects2
    for j in range(len(vecObjects)):
        vecClients[j]['order_state'] = False
    return update_statesResponse(True)


def addAdditionalProducts(recAdditionalOrder):
    "Anadir productos adicionales"
    global str_additional_order
    str_additional_order= recAdditionalOrder.add_orders
    print(str_additional_order)

def updateOrder(reqNameProd):
    "Entregar pedido al cliente"
    global vecClients, state
    x = 0
    for ii in vecClients:
        if ii['name'] == reqNameProd.name:
            ii['order'] = reqNameProd.product
    print(vecClients)
    return update_orderResponse(True)

def giveOrder2Client(reqName):
    "Entregar pedido al cliente"
    global vecClients, state
    x = 0
    for ii in vecClients:
        if ii['name'] == reqName.name:
            state = ii['order_state']
            if state == True:
                z = x
        x = x + 1
    if state == True:
        del vecClients[z]
    print(vecClients)
    return give_order2clientResponse(state, str_additional_order)

def verifyOrders():
    global vecClients, vecObjects2
    vecObjects2 = len(vecClients)*['']
    for d in range(len(vecClients)):
        vecObjects2[d] = findDict(vecClients[d]['order'])
        for j in vecObjects:
            if vecObjects2[d] == j:
                vecClients[d]['order_state'] = True


def findDict(toFind):
    for object in dic_drinks:
        for drink in dic_drinks[object]:
            if drink==toFind:
                return object
    return ''


def callServices():
    rospy.init_node('sIA_order_management')
    a = rospy.Service('srvAddClient', add_client, addClient)
    b = rospy.Service('srvGiveOrder2Bar', give_order2bar, giveOrder2Bar)
    c = rospy.Service('srvValidateOrder', validate_order, validateOrder)
    d = rospy.Service('srvGiveOrder2Client', give_order2client, giveOrder2Client)
    e = rospy.Service('srvAddAdditionalProducts', additional_products, addAdditionalProducts)
    f = rospy.Service('srvUpdateOrder', update_order, updateOrder)
    g = rospy.Service('srvUpdateStates', update_states, updateStates)
    rospy.spin()


if __name__ == "__main__":
    callServices()
