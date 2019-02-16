#!/usr/bin/env python
# -*- coding: utf-8 -*-
from take_order.takeOrder import TakeOrder
from make_order.makeOrder import MakeOrder
from give_order.giveOrder import GiveOrder
from std_msgs.msg import String
import rospy


def callback():
    gv = GiveOrder()## objeto para give_order
    gv.start() ## ejecucion de give_order, funcion con argumento deteccion de persona


def GiveOrdercalback():
    rospy.init_node('sIA_test_order')
    rospy.Subscriber("person_detection", String, callback)
    rospy.spin()
if __name__ == "__main__":
    # tk = TakeOrder() ## Objeto para take_order
    # tk.start() #ejecución take_order
    # mk = MakeOrder() ## objeto para make_order
    # mk.start() ##ejecucion de make_order
    try:
        callback()
    except rospy.ROSInterruptException:
        pass
