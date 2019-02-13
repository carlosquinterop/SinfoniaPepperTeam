#!/usr/bin/env python
# license removed for brevity
"""
//======================================================================//
//  This software is free: you can redistribute it and/or modify        //
//  it under the terms of the GNU General Public License Version 3,     //
//  as published by the Free Software Foundation.                       //
//  This software is distributed in the hope that it will be useful,    //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
//  GNU General Public License for more details.                        //
//  You should have received a copy of the GNU General Public License   //
//  Version 3 in the file COPYING that came with this distribution.     //
//  If not, see <http://www.gnu.org/licenses/>                          //
//======================================================================//
//                                                                      //
//      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //
//      Sinfonia - Colombia                                             //
//      https://sinfoniateam.github.io/sinfonia/index.html              //
//                                                                      //
//======================================================================//
"""
import pprint
import rospy
import sys
import cv2 as cv2
import os
from Class.characterization import Characterization
from Class.utils import Utils
from sinfonia_pepper_tools_interaction.srv import FaceDetector
from sinfonia_pepper_tools_interaction.srv import FaceMemorize
from sinfonia_pepper_tools_interaction.srv import FaceRecognize
from sinfonia_pepper_robot_toolkit.srv import TakePicture
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rospy_message_converter import message_converter
from std_msgs.msg import String

class FaceID():
    def __init__(self):
        self.imagePub = rospy.Publisher('/faceImage', Image, queue_size=10)
        self.bridge = CvBridge()
        self.person = Characterization()
        self.source = None
        self.utils = Utils()

    def detectFace(self, req):
            frame = self.take_picture_source()
            people = self.person.detect_person(frame)
            if req.cvWindow:
                res = self.add_features_to_image(frame,people)
                self.imagePub.publish(self.bridge.cv2_to_imgmsg(res["frame"], "bgr8"))
            return res["isInFront"]

    def recognizeFace(self, req):
        frame = self.take_picture_source()
        people = self.person.indentify_person(frame)
        if req.cvWindow:
            res = self.add_features_to_image(frame,[people])
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(res['frame'], "bgr8"))
        features = str(people)
        print(features)
        return features
    
    def memorizeFace(self, req):
        images = {}
        for i in range(20):
            frame = self.take_picture_source()
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            images[i] = frame
        personId, person= self.person.add_person(req.name, images)
        if req.cvWindow and person.image is not None:
            image = self.add_features_to_image(person, req.name)
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        features = str(person.hairColor +","+ str(person.glasses) +","+ person.gender + "," + str(person.age))
        return personId, features
      
    def take_picture_source(self):
        source = self.source
        if source == 1:
            cap = cv2.VideoCapture(0)
            ret, frame = cap.read()           
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            cap.release()
        elif source == 2:
            ROOT_PATH = os.path.dirname(sys.modules['__main__'].__file__)
            frame =  cv2.imread(ROOT_PATH+"/gente2.jpg")
        else:
            rospy.wait_for_service("sIA_take_picture")
            takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)
            imageRos = takePicture("Take Picture", [0, 2,11,30]).response
            frame = self.bridge.imgmsg_to_cv2(imageRos, "rgb8")
        return frame

    def add_features_to_image(self,frame, people):
        frame_size = frame.shape[0]*frame.shape[1]
        props = self.utils.setProps(people)
        for prop in props: 
            cv2.rectangle(frame, prop['pi'], prop['pf'], (0, 255, 0), 3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            if 'name' in people[0]:
                cv2.putText(frame, str(people[0]['name']), prop['pi'], font, 1, (255, 150, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, str(round(prop['prop']*100/float(frame_size),4))+'%', prop['pi'], font, 1, (255, 150, 0), 2, cv2.LINE_AA)
        cv2.rectangle(frame, props[0]['pi'], props[0]['pf'], (0, 0, 255), 5) #Remarca la cara mayor
        isInFront = False
        if (max(props))['prop']>3:
            isInFront = True
        response = {"frame":frame, "isInFront":isInFront}
        return response

if __name__ == '__main__':
    try:
        rospy.init_node('robot_face_node')
        face = FaceID()
        face.source = 3 #Toma la camara de pepper por defecto
        if(len(sys.argv)>1):
            face.source = int(sys.argv[1])

        rospy.Service('robot_face_detector',FaceDetector,face.detectFace)
        rospy.Service('robot_face_memorize',FaceMemorize,face.memorizeFace)
        rospy.Service('robot_face_recognize',FaceRecognize,face.recognizeFace)
        print("robot face node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
