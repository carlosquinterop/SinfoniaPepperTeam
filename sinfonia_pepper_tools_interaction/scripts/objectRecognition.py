# Clase para la identificacion de objetos a partir de una imagen
# Desarrollado por Sinfonia Pepper Team
# Universidad Santo Tomas
# Bogota, Colombia
# Enero de 2019

#------------------------
#    CONFIGS
#------------------------


import time
import rospy
from PIL import Image
# Direccion de salida de las imagenes
img_file_path = "./src/sinfonia_pepper_tools_interaction/scripts/Resources/Objects.jpg"
img_file_path2 = "./src/sinfonia_pepper_tools_interaction/scripts/Resources/Objects_depth.jpg"
import os
import time
import struct

import rospy
import argparse
import numpy as np
from PIL import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64MultiArray
from sinfonia_pepper_robot_toolkit.srv import TakePicture
from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector, Wav, T2S, File
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
		self.bridge = CvBridge()
		self.model = lightnet.load('yolo')
		self.objects = []
		self.es_dict = {     'bottle' : 'botella' , \
		'keyboard':'teclado' , \
		'diningtable':'mesa' , \
		'cup':'taza' , \
		'vase':'vaso' , \
		'laptop':'portatil' , \
		'wine glass': 'copa' }
		self.frame = None
		rospy.wait_for_service("sIA_take_picture")
		self.takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)
		self.boxes = []
		self.dists = []

	def takePhoto(self, source):
		self.txt_line = ""
		self.objects = []
		#"Toma una foto y la guarda en un archivo .jpg"
		if source == 1:
			for i in range(1):
				self.cap.read()
			ret, self.frame = self.cap.read()
			cv2.imwrite( img_file_path , self.frame )
		else:
			response = self.takePicture("Take Picture", [0, 2, 11, 30]).response
			self.frame = self.bridge.imgmsg_to_cv2(response, "bgr8")
			cv2.imwrite( img_file_path , self.frame )

	def takeDepthPhoto(self):
		# Functionality test
		distances = []
		response = self.takePicture("Take Picture", [2, 1, 17, 1]).response
		for i in range(0, len(response.data) - 1, 2):
			distances.append(struct.unpack('<H', response.data[i:i + 2])[0])

		distArray = np.array(distances, dtype=float).reshape(240, 320)
		newDistArray = cv2.resize(distArray,(self.frame.shape[0], self.frame.shape[1]))
		cv2.imwrite( img_file_path2 , self.frame )
		self.dists = len(self.boxes)*[0.0]

		for i in range(len(self.boxes)):
			x1 = int(self.boxes[i][3][0])
			x2 = x1 + int(self.boxes[i][3][2])
			y1 = int(self.boxes[i][3][1])
			y2 = y1 + int(self.boxes[i][3][3])
			dist = newDistArray[y1:y2, x1:x2]
			dist = dist.mean()
			self.dists[i] = dist
		print(self.dists / 100 + "m")
		print(self.dists / 10  + "cm")

		print(self.boxes)

	def processPhoto(self):
		#"Obtiene el nombre de los objetos de la foto tomada"
		image = lightnet.Image.from_bytes( open( img_file_path , 'rb' ).read() )
		self.boxes = self.model( image )
		#print(self.boxes)
		for obj in self.boxes:
			self.objects.append( obj[1] )



	def speakObjects(self):
		#"Imprime los objetos detectados"

		for obj in self.objects:
			self.txt_line += obj
			self.txt_line += ","
			#print("speakObjects = %s", self.txt_line)
