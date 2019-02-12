#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sinfonia_pepper_tools_interaction.srv import FaceDetector
from sinfonia_pepper_tools_interaction.srv import FaceMemorize
    
class TestFaceID():

    def detect_face(self, cvWind):
        try:
            detect_face_request = rospy.ServiceProxy('robot_face_detector',FaceDetector)
            is_face_in_Front = detect_face_request(cvWind)
            if is_face_in_Front.response:
                print("Si Hay cara")
            else:
                print("No hay cara")
        except rospy.ServiceException:
            print ("Error!! Make sure robot_face node is running ")    
            
    def memorize_face(self, name, cvWind):
        try:
            memorize_face_request = rospy.ServiceProxy('robot_face_memorize',FaceMemorize)
            azure_id = memorize_face_request(name, cvWind)
            print("azure_id = {}".format(azure_id))
        except rospy.ServiceException:
            print ("Error!! Make sure robot_face node is running ")
            
         
if __name__ == '__main__':
    try:
        rospy.init_node('test_face_node', anonymous=True)
        rospy.loginfo("Nodo Test Face Iniciado")
        test = TestFaceID()
        while 1: 
            option = int(raw_input('** Welcome to Robot Face Test Node ** \n What do you want to test\n 1. Face recognition service \n 2. Face detector service\n 3. Face memorize service \n'))            
            try:
                options = ['\n',1,2,3]
                if(options.index(int(option))):
                    choise = raw_input('you want the captures to be displayed? (S/n) ')
                    camera = int(raw_input('What camera do you want to use: \n 1. WebCam \n 2. Pepper\n'))
                    if(choise==('s' or 'yes' or 'si' or 'S')):
                        cvWindow = True
                    else:
                        cvWindow = False
                    if(option==1): 
                        print('-Recognize service:')
                        test.recognizeFace(cvWindow)
                    elif(option==2):
                        print('-Detector service:')
                        test.detect_face(cvWindow)
                    else:
                        print('-Memorize service:')
                        name  = raw_input('Ingrese el nombre de la persona que desea registrar:\n ')
                        test.memorize_face(name,cvWindow)
            except:
                print("""\n\n
                         .d88b. 888d888888d888 .d88b. 888d888 
                        d8P  Y8b888P"  888P"  d88""88b888P"   
                        88888888888    888    888  888888     
                        Y8b.    888    888    Y88..88P888     
                         "Y8888 888    888     "Y88P" 888   \n""")
                print('                         [Wrong instruction, try again]\n\n')
    except rospy.ROSInterruptException:
        pass