#!/usr/bin/env python

#========================================================================================
#			TRACEBACK
#
#	13-02-2019:	Move to a specific point. X and Y values achieved.
#	14-02-2019: Added movement to a specific theta value.
#	14-02-2019: Added subscriber to read Location messages.
#
#========================================================================================


#========================================================================================
#			IMPORTS
#========================================================================================

# Import rospy to communicate with other ROS nodes.
import rospy

# Importing tf to compute rotation transforms
import tf

# Importing the requiered messages to be sended over an action.
from geometry_msgs.msg import PoseStamped
from sinfonia_pepper_tools_navigation.msg import location


#========================================================================================
#			AUXILIARY FUNCTIONS
#========================================================================================

#----------------------------------------------------------------------------------------
# This function sends the message which will move Pepper to a specific point.
# Parameters:	x		-> X value of intended pose.
#				y		-> Y value of intended pose.
#				theta	-> Angular pose of Pepper.
#----------------------------------------------------------------------------------------
def sendPointToMove(x, y, theta):
	# Instance of the message
	goal = PoseStamped()

	# The rotation quaternion is calculated								
	quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

	# Adding the required fields to goal
	goal.header.stamp = rospy.Time.now()		# Header values
	goal.header.frame_id = "/map"
	goal.pose.position.x = x					# Pose values
	goal.pose.position.y = y
	goal.pose.position.z = 0.0
	goal.pose.orientation.x = quaternion[0]		# Orientation values
	goal.pose.orientation.x = quaternion[1]
	goal.pose.orientation.x = quaternion[2]
	goal.pose.orientation.x = quaternion[3]

	# Publish on topic
	pose_pub.publish(goal)

#----------------------------------------------------------------------------------------
# This function gets the message data and calls the function to move Pepper with the
# received arguments.
# Parameters:	data	-> Variable containing the received message.
#----------------------------------------------------------------------------------------
def receivedMessageCallback(data):
	# Retreiving data from received message
	x = data.x
	y = data.y
	theta = data.w

	# Calling the publishing function
	sendPointToMove(x, y, theta)

#========================================================================================
#			MAIN NODE FUNCTION
#========================================================================================

#----------------------------------------------------------------------------------------
# Main node function declaration.
# Parameters:	None
#----------------------------------------------------------------------------------------
def moveToPoint():

	# Publishers declaration. Must declare every publisher as a globa variable.
	global pose_pub

	# Then, create the Publisher object for each publisher.
	pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

	# Creating the subscription to goal topic
	rospy.Subscriber( '/sIA_move_to_point', location, receivedMessageCallback )

	# Node declaration
	rospy.init_node('sIA_move_to_point', anonymous=True)

	# Keeping the node alive is a good idea.
	rospy.spin()


#========================================================================================
#			NODE START
# This is the decent way to start a ROS node in Python
#========================================================================================
if __name__ == "__main__":
	try:
		moveToPoint()
	except rospy.ROSInterruptException:
		print('An error has ocurred over sIA_move_to_point node')
		pass
