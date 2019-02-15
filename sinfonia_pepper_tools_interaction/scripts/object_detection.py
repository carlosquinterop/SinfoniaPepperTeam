#! /usr/bin/env python
# Clase para la identificacion de objetos a partir de una imagen
# Desarrollado por Sinfonia Pepper Team
# Universidad Santo Tomas
# Bogota, Colombia
# Enero de 2019


import rospy
from sinfonia_pepper_tools_interaction.srv import *
from std_msgs.msg import String
# Importing class 'objectRecognition'
from objectRecognition import *
objectRecognition = ObjectRecognition()
# State variable used on the state machine
state = 'awaitingInput'
cameraObjects = []

def ObjectRec():
	global txt_line
	objectRecognition.takePhoto(1)
	print("take photo")
	objectRecognition.processPhoto()
	print("process photo")
	objectRecognition.speakObjects()
	state = 'awaitingInput3'
	print("speak Objects")


def talker():
	rospy.init_node('sIA_objetct_detection')
	print ("Nodo creado con exito")
	a = rospy.Service('srvDetectObjects', detect_objects, detectObjects)
	rospy.spin()
def testCamera():
	rospy.wait_for_service("sIA_take_picture")
	takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)

	# Functionality test
	response = takePicture("Take Picture", [0, 2, 11, 30]).response
	image = Image.frombytes("RGB", (response.width, response.height), str(bytearray(response.data)))
	image.show()
	response = takePicture("Take Picture", [1, 2, 11, 30]).response
	image = Image.frombytes("RGB", (response.width, response.height), str(bytearray(response.data)))
	image.show()


def detectObjects(data):
		global cameraObjects
		ObjectRec()
		cameraObjects=""
		cameraObjects = objectRecognition.txt_line.split(",")
		cameraObjects.remove('')
		return detect_objectsResponse(cameraObjects)


if __name__== '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
