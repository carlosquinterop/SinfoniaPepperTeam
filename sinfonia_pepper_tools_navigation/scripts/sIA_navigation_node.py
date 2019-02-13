#!/usr/bin/env python
import rospy
import numpy as np 
import time
import roslaunch
import tf
import csv
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from sinfonia_pepper_tools_navigation.msg import doReactNav
from sinfonia_pepper_tools_navigation.msg import location
from sinfonia_pepper_robot_toolkit.msg import MoveToVector
from sinfonia_pepper_robot_toolkit.msg import MoveTowardVector

# Variables auxiliares
globalMap = None
oldName = ""

def mapCallback(data):
    global globalMap
    globalMap = data
    print(globalMap)
    pass

def getMap(self):
    return globalMap

# Functional Callbacks 
def doReactNavigationCallback(data):
    doReactNavigation = data.doReactNav
    updateMap = data.updateMap

    rate = rospy.Rate(10) # 10hz    
    while not rospy.is_shutdown():
        # Print 
        print("updateMap:" + str(updateMap))
        print("doReactNavigation:" + str(doReactNavigation))
        # Get the map through activating the lasers
        pubLaser = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        if (updateMap == True):
            pubLaser.publish("sIA_laser_srdf.laser_scan.ON")
            pubLaser.publish("sIA_laser_srdl.laser_scan.ON")
            pubLaser.publish("sIA_laser_srdr.laser_scan.ON")

        else:
            pubLaser.publish("sIA_laser_srdf.laser_scan.OFF")
            pubLaser.publish("sIA_laser_srdl.laser_scan.OFF")
            pubLaser.publish("sIA_laser_srdr.laser_scan.OFF")

        # Move Pepper in a square
        pub = rospy.Publisher("sIA_move_toward", MoveTowardVector, queue_size=10)
        while pub.get_num_connections() == 0:
            rate.sleep()
        pub2 = rospy.Publisher("sIA_stop_move", String, queue_size=10)
        while pub2.get_num_connections() == 0:
            rate.sleep()
    
        # Functionality test
        square = [[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [-1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0]]
    
        for vertex in square:
            msg = utils.fillVector(vertex, "mtw")
            pub.publish(msg)
            time.sleep(3)
    
        stopStr = "Stop"
        pub2.publish(stopStr)
        time.sleep(3)
    
        # Error test
        msg = utils.fillVector([0.0, -2.0, 0.0], "mtw")
        pub.publish(msg)
        time.sleep(3)
        stopStr = "Stap"
        pub2.publish(stopStr)

def moveToPointCallback():
    rate = rospy.Rate(10)
    pub = rospy.Publisher("sIA_move_to", MoveToVector, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()
    
        # Move Pepper in another square
        square = [[1.0, 0.0, 0.0, 6.0],
                  [0.0, 1.0, 0.0, 6.0],
                  [-1.0, 0.0, 0.0, 6.0],
                  [0.0, -1.0, 0.0, 6.0]]
    
        for vertex in square:
            msg = utils.fillVector(vertex, "mt")
            pub.publish(msg)
            time.sleep(8)
            
def setPlacesLocationCallback(data):
    global oldName

    name = data.name
    x = data.x
    y = data.y
    w = data.w
    z = data.z
    
    if oldName != name:
        with open('file.csv', mode='w') as locations_file:
            locations_file = csv.writer(locations_file)
            locations_file.writerow(['Juan', x, y, w])
       
            
        print("Place succesfully saved")
        oldName = name
    
    
def getPlaceLocationCallback(data):
    name = data.name
    
    with open('file.csv', 'rb') as locations_file:
        locations_reader = csv.reader(locations_file, delimiter=',', quotechar='|')
        for i in locations_reader:
            response = location()
            if name == i[0]:
                response.x = i[1] 
                response.y = i[2]
                response.z = 0
                response.w = i[3]
            else:
                response.x = 0
                response.y = 0
                response.z = 0
                response.w = 0
        return response


# --------------- Main control function ---------------
def sIANavigationNode():
    # Init main navigation node
    rospy.init_node('sIA_navigation_node', anonymous=True)

    # Launch NavigationTools nodes
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/juan/pepper_sinfonia_ws/src/sinfonia_pepper_tools_navigation/launch/sIA_NavigationTools.launch"])
    launch.start()

    # Suscribe to permanent topics
    tfListener = tf.TransformListener() 
    rospy.Subscriber('map',OccupancyGrid, mapCallback)

    # Suscribe topics for functionalities 
    rospy.Subscriber('sIA_do_react_navigation', doReactNav , doReactNavigationCallback)
    rospy.Subscriber('sIA_move_to_point', Quaternion, moveToPointCallback)
    rospy.Subscriber('sIA_set_places_location', location, setPlacesLocationCallback)
    rospy.spin()

# Init main control
if __name__ == '__main__':
    try:
        sIANavigationNode()
    except rospy.ROSInterruptException:
        pass


