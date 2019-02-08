#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2 as cv2
from Class.characterization import Characterization
from sinfonia_pepper_tools_interaction.srv import FaceDetector
from sinfonia_pepper_tools_interaction.srv import FaceMemorize
from sinfonia_pepper_tools_interaction.srv import FaceRecognize
from sinfonia_pepper_robot_toolkit.srv import TakePicture
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class FaceID():
    def __init__(self):
        self.imagePub = rospy.Publisher('/faceImage', Image, queue_size=10)
        self.bridge = CvBridge()
        self.person = Characterization()
    def recognizeFace(self, cvWindow):
        personId = self.person.indentify_person(cvWindow)
        return personId
    
    def memorizeFace(self, req):
        images = {}
        print('req:',req)
        for i in range(20):
            frame = self.take_picture_source(1)
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            images[i] = frame
        personId, person= self.person.add_person(req.name, images)
        if req.cvWindow and person.image is not None:
            image = self.add_features_to_image(person, req.name)
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        personId = personId
        features = str(person.hairColor +","+ str(person.glasses) +","+ 
                              person.gender + "," + str(person.age))
        return personId, features
    
    def detectFace(self, req):
        frame = self.take_picture_source(1)
        isInFront, frame = self.person.detect_person( frame )
        if req.cvWindow:
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        return isInFront
    
    def take_picture_source(self,source):
        if source == 1:
            cap = cv2.VideoCapture(0)
            ret, frame = cap.read()
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            cap.release()
        else:
            rospy.wait_for_service("sIA_takePicture")
            takePicture = rospy.ServiceProxy("sIA_takePicture", TakePicture)
            imageRos = takePicture("Take Picture", [0, 2,11,30])
            frame = self.bridge.imgmsg_to_cv2(imageRos, "rgb8")
        return frame
    def add_features_to_image(self,person, name):
        pi = (person.bb['left'], person.bb['top'])
        pf = (person.bb['left'] + person.bb['width'],
              person.bb['top'] + person.bb['height'])
        cv2.rectangle(person.image, pi, pf, (0, 255, 0), 3)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(person.image, name, pi, font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        return person.image
        
if __name__ == '__main__':
    try:
        rospy.init_node('robot_face_node')
        face = FaceID()
        rospy.Service('robot_face_detector',FaceDetector,face.detectFace)
        rospy.Service('robot_face_memorize',FaceMemorize,face.memorizeFace)
        rospy.Service('robot_face_recognize',FaceRecognize,face.recognizeFace)
        print("robot face node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
