#!/usr/bin/env python

import sys
import rospy
from sinfonia_pepper_tools_decisionmaking.srv import *
from sinfonia_pepper_tools_decisionmaking.msg import *


def Take_order(x, y):
    rospy.wait_for_service('srvAddClient')
    try:
        add_two_ints = rospy.ServiceProxy('srvAddClient', add_client,persistent=False,headers=None)
        resp1 = add_two_ints(x, y)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def Make_order(a, b, c):
    rospy.wait_for_service('srvValidateOrder')
    try:
        add_two_ints2 = rospy.ServiceProxy('srvValidateOrder', validate_order,persistent=False,headers=None)
        resp2 = add_two_ints2(a, b, c)

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

def Additionals_order(order):
    rospy.wait_for_service('srvAddAdditionalProducts')
    try:
        add_two_ints = rospy.ServiceProxy('srvAddAdditionalProducts', additional_products,persistent=False,headers=None)
        resp5 = add_two_ints(order)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    # x="jaime"
    # y="vino"
    # x="daniela"
    # y="gaseosa"
    # x="bryan"
    # y="cafe"
    # a="gaseosa"
    # b="vino"
    # c="Null"
    arg="Camilo"
    # str_order="x"
    # order="te cerveza"
    # Take_order(x, y)
    # Make_order(a, b, c)
    print(Give_order(arg))
    # Give_order_bar(str_order)
    # Additionals_order(order)
