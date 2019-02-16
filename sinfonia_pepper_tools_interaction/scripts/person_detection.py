#!/usr/bin/env python3
import rospy

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv2
from darkflow import person_detect as pd
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/home/roboticacuda/pepper_sinfonia_ws/devel/lib/python2.7/dist-packages")
import time
from PIL import Image as im
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from std_msgs.msg import String
bridge = None
t0 = 0
_rate = 0
x = 0
#cam = cv2.VideoCapture(0)
frame = None
source = '2'
def VideoCallback(image):
    global bridge, t0, frame
    framerate = int(1/(time.time()-t0))
    t0 = time.time()
    frame = bridge.imgmsg_to_cv2(image, "rgb8")
    print("Framerate: " + str(framerate) + " fps")
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.namedWindow("Video")
    cv2.imshow("Video", frame)
    cv2.waitKey(10)


def main():
    global pub1, frame, bridge, t0, x
    pub = rospy.Publisher('person_detection', String, queue_size=10)
    rospy.init_node('sIA_person_detection', anonymous=True)
    _rate = rospy.Rate(10)
    pub1 = rospy.Publisher("sIA_stream_from", String, queue_size=10)
    rospy.Subscriber("sIA_video_stream", Image, VideoCallback)
    print("Nodo creado con exito")
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        if source == '1':
            ret_val, img = cam.read()
            if pd.detect_person(img):
                print(pd.detect_person(img))
                pub.publish("True")
            else:
                pub.publish("False")
            r.sleep()
        else:
            bridge = CvBridge()
            pub1.publish("sIA_video_stream.0.1.11.30.ON")
            t0 = time.time()
            x=x+1
            if pd.detect_person(frame) & x>30:
                pub1.publish("sIA_video_stream.0.1.11.30.OFF")
                pub.publish("True")
            else:
                pub.publish("False")
            _rate.sleep()
if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
