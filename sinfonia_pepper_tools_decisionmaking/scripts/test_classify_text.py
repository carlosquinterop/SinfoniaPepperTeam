#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from sinfonia_pepper_tools_decisionmaking.srv import *
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

def callback_master(x):

    rospy.wait_for_service('srv_classify_text')
    try:
        send_text = rospy.ServiceProxy('srv_classify_text', classify_text)
        resp2 = send_text(x)
        print("hablar:",x)
        return resp2.affirmation,resp2.negation,resp2.clientName,resp2.clientOrder
    except:
        print ("Service call failed: %s")

if __name__ == "__main__":
    x = 'Mi nombre es Tatiana y quiero una gaseosa.'
    print(x)
    print(callback_master(x))
