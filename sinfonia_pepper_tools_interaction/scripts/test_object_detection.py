#!/usr/bin/env python

import sys
import rospy
from sinfonia_pepper_tools_interaction.srv import *


def detectObjects(object):
    rospy.wait_for_service('srvDetectObjects')
    try:
        add_two_ints = rospy.ServiceProxy('srvDetectObjects', detect_objects)
        resp5 = add_two_ints(object)
        return resp5.objects,resp5.distances

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    object = "gas"
    print(detectObjects(object))
