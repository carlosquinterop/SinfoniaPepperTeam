#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import threading
#from person import Person

class CV_Image():
    def __init__(self):
        self.bridge = CvBridge()
        #self.persons = Person()
        self.imageFlag = False
        self.cv_image = None
        self.imageSub = rospy.Subscriber('/faceImage', Image, self.imageCallback)
        
    def imageCallback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.imageFlag = True
        except CvBridgeError as e:
            print(e)
    def visualize(self):
        rate = rospy.Rate(1000) # 10hz
        while not rospy.is_shutdown():
            if self.imageFlag:
                plt.figure(1)
                image.imageFlag = False;
                plt.imshow(image.cv_image, cmap = 'gray', interpolation = 'bicubic')
                plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
                plt.draw()
            rate.sleep()  
    
if __name__ == '__main__':
    try:
        rospy.init_node('visualize_face_node', anonymous=True)
        rospy.loginfo("Nodo visualize_face_image Iniciado")
        image = CV_Image();
        hilo2 = threading.Thread(target = image.visualize)
        hilo2.daemon = True
        hilo2.start()
        plt.figure(1)
        plt.show()      
    except rospy.ROSInterruptException:
        pass