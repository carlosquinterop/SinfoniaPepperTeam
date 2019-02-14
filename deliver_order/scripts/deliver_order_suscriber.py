#!/usr/bin/env python
import rospy
from std_msgs.msg import String
orders = None
def callback(data):
    global orders
    orders = data.data
def suscribe_topic():
    rospy.Subscriber("person_detection", String, callback)
