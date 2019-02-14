#!/usr/bin/env python

#######################################################################################################################
#			IMPORTS																									  #
#######################################################################################################################

# Import rospy to communicate with other ROS nodes.
import rospy

# Importing the requiered messages to be sended over an action.
from geometry_msgs.msg import PoseStamped

#######################################################################################################################
#			AUXILIARY FUNCTIONS																						  #
#######################################################################################################################


def sendPointToMove(x, y, theta):
	#--------- Instance of the message ----------------------------------------
	goal = PoseStamped()

	#--------- Adding the required fields to goal -----------------------------
	#
	#					Header values
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "/map"

	#					Pose values
	goal.pose.position.x = x
	goal.pose.position.y = y
	goal.pose.position.z = 0.0
	goal.pose.orientation.w = 0.5

	#--------- Publish on topic -----------------------------------------------
	pose_pub.publish(goal)


#######################################################################################################################
#			MAIN NODE FUNCTION																						  #
#######################################################################################################################
def moveToPoint():

	#------------------------ Publishers declaration ----------------------------------------------
	# Must declare every publisher as global variables.
	global pose_pub

	# Then...create the Publisher object.
	pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

	#------------------------ Node declaration ----------------------------------------------------
	rospy.init_node('sIA_move_to_point', anonymous=True)
	rate = rospy.Rate(1)

	# Run the ROS node.
	while not rospy.is_shutdown():

		# Callback to sendPointToMove
		sendPointToMove(0,0,0)
		rate.sleep()



#######################################################################################################################
#			NODE START																								  #
# Running the decent way to start a ROS node in python																  #
#######################################################################################################################
if __name__ == "__main__":
	try:
		moveToPoint()
	except rospy.ROSInterruptException:
		print('An error has ocurred over sIA_move_to_point node')
		pass
