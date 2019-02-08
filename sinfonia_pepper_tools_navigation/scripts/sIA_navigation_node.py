#!/usr/bin/env python
import rospy
import numpy as np 
import time
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from sinfonia_pepper_tools_navigation.msg import doReactNav

doReactNavigation = False
updateMap = False

def doReactNavigationCallback(msg):
    global doReactNavivagion
    doReactNavigation = msg.doReactNav.doReactNav
    global updateMap
    updateMap = msg.doReactNav.updateMap


def sIANavigationNode():
    # Init main navigation node
    rospy.init_node('sIA_navigation_node', anonymous=True)
    # Launch NavigationTools nodes
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nicolas-rocha/pepper_sinfonia_ws/src/sinfonia_pepper_tools_navigation/launch/sIA_NavigationTools.launch"])
    launch.start()
    # Publishing topics
    pubMoveTo = rospy.Publisher("sIA_moveTo", Quaternion, queue_size=10)
    pubLaser = rospy.Publisher("sIA_stream_from", String, queue_size=10)
    # Suscribed topics
    rospy.Subscriber('sIA_do_react_navigation', doReactNav , doReactNavigationCallback)
    #rospy.Subscriber('sIA_do_react_navigation', doReactNav, doReactNavigationCallback)
    rospy.spin()
    
    try:
        
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            
            # Get the map through activating the lasers
            if (updateMap == True):
                pubLaser.publish("sIA_laser_srdf.laser_scan.ON")
                pubLaser.publish("sIA_laser_srdl.laser_scan.ON")
                pubLaser.publish("sIA_laser_srdr.laser_scan.ON")

            else:
                pubLaser.publish("sIA_laser_srdf.laser_scan.OFF")
                pubLaser.publish("sIA_laser_srdl.laser_scan.OFF")
                pubLaser.publish("sIA_laser_srdr.laser_scan.OFF")

            # Move Pepper in a square
            if (doReactNavigation == True):

                while pubMoveTo.get_num_connections() == 0:
                    rate.sleep()

                square = [[1.0, 0.0, 0.0, 6.0],
                          [0.0, 1.0, 0.0, 6.0],
                          [-1.0, 0.0, 0.0, 6.0],
                          [0.0, -1.0, 0.0, 6.0]]

                for vertex in square:
                    msg = utils.fillVector(vertex, 'q')
                    pubMoveTo.publish(msg)
                    time.sleep(8)

                msg = utils.fillVector([0.0, -1.0, 4.0, 6.0], 'q')
                pubMoveTo.publish(msg)

    except rospy.ServiceException as e:
        print ("Error!! ")
        pass

    
if __name__ == '__main__':
    try:
        sIANavigationNode()
    except rospy.ROSInterruptException:
        pass


