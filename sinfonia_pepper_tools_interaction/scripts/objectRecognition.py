# Clase para la identificacion de objetos a partir de una imagen
# Desarrollado por Sinfonia Pepper Team
# Universidad Santo Tomas
# Bogota, Colombia
# Enero de 2019

#------------------------
#	CONFIGS
#------------------------


import time
import rospy
from PIL import Image
# Direccion de salida de las imagenes
img_file_path = "./src/sinfonia_pepper_tools_interaction/scripts/Resources/Objects.jpg"
import os
import time

import rospy
import argparse
import numpy as np
from PIL import Image
# from cv_bridge import CvBridge
from std_msgs.msg import String, Float64MultiArray
# from sinfonia_pepper_robot_toolkit.srv import TakePicture
# from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector, Wav, T2S, File
# CV2 librery used for image capture and processing.
import cv2
# Lightnet library used for image processing and object recognition
import lightnet
import os
# Puerto de la camara a usar por parte del programa.
camera_port = 0

class ObjectRecognition():
	def __init__(self):
		self.txt_line =  ""
		self.cap = cv2.VideoCapture( camera_port )
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		self.cap.set(cv2.CAP_PROP_FPS, 30)
		# self.bridge = CvBridge()
		self.model = lightnet.load('yolo')
		self.objects = []
		self.es_dict = { 	'bottle' : 'botella' , \
							'keyboard':'teclado' , \
							'diningtable':'mesa' , \
							'cup':'taza' , \
							'vase':'vaso' , \
							'laptop':'portatil' , \
							'wine glass': 'copa' }


	def takePhoto(self, source):
		self.txt_line = ""
		self.objects = []
        #"Toma una foto y la guarda en un archivo .jpg"
		if source == 1:
			for i in range(1):
				self.cap.read()
			ret, frame = self.cap.read()
			cv2.imwrite( img_file_path , frame )
		else:
			rospy.wait_for_service("sIA_take_picture")
			takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)
			response = takePicture("Take Picture", [1, 2, 11, 30]).response
			# frame = self.bridge.imgmsg_to_cv2(response, "bgr8")
			cv2.imwrite( img_file_path , frame )


	def processPhoto(self):
		#"Obtiene el nombre de los objetos de la foto tomada"
		image = lightnet.Image.from_bytes( open( img_file_path , 'rb' ).read() )
		boxes = self.model( image )
		#print(boxes)
		for obj in boxes:
			self.objects.append( obj[1] )


	def speakObjects(self):
		#"Imprime los objetos detectados"

		for obj in self.objects:
			self.txt_line += obj
			self.txt_line += ","
			#print("speakObjects = %s", self.txt_line)
