#!/usr/bin/env python

#========================================================================================
#			TRACEBACK
#
#	14-02-2019:	
#
#========================================================================================

#========================================================================================
#			IMPORTS
#========================================================================================

# Import rospy to communicate with other ROS nodes.
import rospy

# Importing the requiered messages to be sended over an action.
from keyboard import Keyreader
from sinfonia_pepper_tools_navigation.msg import location


#========================================================================================
#			AUXILIARY FUNCTIONS
#========================================================================================

def nodeCallback():
	# Retrieve number from keyboard input.
	num = key.getNumber()

	# Define the movement array.
	delta = movement[num]
	
	# Calculate new Pepper position
	pos[0] = pos[0] + delta[0]
	pos[1] = pos[1] + delta[1]
	pos[2] = pos[2] + delta[2]

	# Create the new message
	message = location()
	message.name = 'Pepper'
	message.x = pos[0] 
	message.y = pos[1]
	message.z = pos[2]

	# Publish the new position into topic
	pose_pub.publish(message)

	# Preventing Pepper from getting crazy
	crazy = True
	while crazy:
		if key.getNumber() != num:
			crazy = False


#========================================================================================
#			MAIN NODE FUNCTION
#========================================================================================

#----------------------------------------------------------------------------------------
# Main node function declaration.
# Parameters:	None
#----------------------------------------------------------------------------------------
def moveToPointTest():

	# Publishers declaration. Must declare every publisher as a globa variable.
	global pose_pub

	# Then, create the Publisher object for each publisher.
	pose_pub = rospy.Publisher('/sIA_move_to_point', location, queue_size=10)

	# Node initialization
	rospy.init_node('sIA_move_to_point_test', anonymous=True)

	# Rate definition
	rate = rospy.Rate(100)

	# Dictionary that will tell how to move Pepper
	global movement
	movement = {
		0:[0.15,	0,		0],			# + 15cm on X
		1:[-0.15,	0,		0],			# - 15cm on X
		2:[0,		-0.15,	0],			# + 15cm on Y
		3:[0,		0.15,	0],			# - 15cm on Y
		4:[0,		0,		-0.0872],	# + 10deg rotation on Z
		5:[0,		0,		0.0872],	# - 10deg rotation on Z
		6:[0,		0,		0],			# Do nothing
	}

	# Global position variable.
	global pos
	pos = [0, 0, 0]

	# Keyreader object declaration
	global key
	key = Keyreader()
	key.start()

	# Node running
	while not rospy.is_shutdown():
		nodeCallback()
		rate.sleep()





#========================================================================================
#			NODE START
# This is the decent way to start a ROS node in Python
#========================================================================================
if __name__ == "__main__":
	try:
		moveToPointTest()
	except rospy.ROSInterruptException:
		print('An error has ocurred over sIA_move_to_point_test node')
		pass
