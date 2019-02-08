#!/usr/bin/env python
# license removed for brevity
import time
#import utils
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from sinfonia_pepper_tools_navigation.msg import doReactNav
 
global TESTTOPIC
TESTTOPIC = "sIA_navigation"

def navigationTestNodeCallback(msg):
	pass

def navigationTestNode():
	rospy.init_node('navigation_test_node', anonymous=True)
	rate = rospy.Rate(10)
	if TESTTOPIC== "sIA_navigation":
		testNavigation(rate)

def testNavigation(rate):
	pub= rospy.Publisher('sIA_start_navigation',doReactNav,queue_size=2)
	rospy.Subscriber('map',OccupancyGrid, navigationTestNodeCallback)
	msg = doReactNav()
	msg.doReactNav = [True,True]
	while pub.get_num_connections()==0:
		rate.sleep()

	pub.publish(msg.doReactNav)
	#pub.publish([True,True])
	rate.sleep()

if __name__ =='__main__':
	try:
		navigationTestNode()
	except rospy.ROSInterruptException:
		pass