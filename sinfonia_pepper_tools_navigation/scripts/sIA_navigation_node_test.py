#!/usr/bin/env python
# license removed for brevity
import time
#import utils
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from sinfonia_pepper_tools_navigation.msg import doReactNav
from sinfonia_pepper_tools_navigation.msg import location
from sinfonia_pepper_tools_navigation.srv import GetMap
from sinfonia_pepper_tools_navigation.srv import GetPlaceLocation
 
global TESTTOPIC
# Variable de prueba - EDITE EL VALOR DE TESTTOPIC POR LA FUNCIONALIDAD QUE DESEA PROBAR!
TESTTOPIC = "sIA_test_RRT"

# --- Calbacks ---
def navigationTestNodeCallback(msg):
	pass

# --- Funcion principal ---
def navigationTestNode():
    rospy.init_node('navigation_test_node', anonymous=True)
    rate = rospy.Rate(10)
    
    
    if TESTTOPIC == "sIA_test_react_navigation":
		testDoReactNavigation(rate)

    if TESTTOPIC == "sIA_test_move_to_point":
		testMoveToPoint(rate)

    if TESTTOPIC == "sIA_test_get_map":
		testGetMap(rate)
    
    if TESTTOPIC == "sIA_test_set_place_location":
		testSetPlacesLocation(rate)
    
    if TESTTOPIC == "sIA_test_get_place_location":
 #       testSetPlacesLocation(rate)
   		testGetPlaceLocation(rate)

    if TESTTOPIC == "sIA_test_RRT":
        testRRT(rate)

# --- Funciones de pruebas ---
def testDoReactNavigation(rate):
	pub= rospy.Publisher('sIA_do_react_navigation', doReactNav, queue_size = 5)
	msg = doReactNav()
	msg.doReactNav = False
	msg.updateMap = True
	while not rospy.is_shutdown():
		print(msg)
		pub.publish(msg)
        rate.sleep()

def testMoveToPoint(rate):
	pub = rospy.Publisher('sIA_move_to_point',Quaternion, queue_size = 5)
	msg = Quaternion()
	msg.x = 2.0
	msg.y = 0.0
	msg.z = 0.0
	msg.w = 1.0
	while not rospy.is_shutdown():
		print(msg)
		pub.publish()
		rate.sleep()

def testGetMap(rate):
    rospy.wait_for_service('sIA_get_map')
    getMap = rospy.ServiceProxy("sIA_get_map", GetMap)
    response = getMap("Get Map", True).response
    print(response)
    
def testSetPlacesLocation(rate):
    pub= rospy.Publisher('sIA_set_places_location', location, queue_size = 5)
    msg = location()
    msg.name = "Living Room"
    msg.x = 2 
    msg.y = 2
    msg.z = 0
    msg.w = 2
    while not rospy.is_shutdown():
        print(msg)
        pub.publish(msg)
        rate.sleep()
    
def testGetPlaceLocation(rate):
    rospy.wait_for_service('sIA_get_place_location')
    placeLocation = rospy.ServiceProxy("sIA_get_places_location", GetPlaceLocation)
    response = placeLocation("Get Place Location", "Living room").response
    print(response)

def testRRT(rate):
    pub = rospy.Publisher('sIA_target',location, queue_size = 5)
    msg = location()
    msg.x = 3
    msg.y = 0
    msg.w = 0.5
    while not rospy.is_shutdown():
        print(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ =='__main__':
	try:
		navigationTestNode()
	except rospy.ROSInterruptException:
		pass
