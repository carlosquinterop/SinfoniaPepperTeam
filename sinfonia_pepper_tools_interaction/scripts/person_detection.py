#!/usr/bin/env python3
import rospy

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv2
from darkflow import person_detect as pd
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/home/roboticacuda/pepper_sinfonia_ws/devel/lib/python2.7/dist-packages")


from std_msgs.msg import String

cam = cv2.VideoCapture(0)

def main():
    pub = rospy.Publisher('person_detection', String, queue_size=10)
    rospy.init_node('sIA_person_detection', anonymous=True)
    print("Nodo creado con exito")
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        ret_val, img = cam.read()
        if pd.detect_person(img):
            print(pd.detect_person(img))
            pub.publish("True")
        else:
            pub.publish("False")
        r.sleep()

if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
