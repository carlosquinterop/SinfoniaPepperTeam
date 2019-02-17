#!/usr/bin/env python

#========================================================================================
#			TRACEBACK
#
#	13-02-2019:	Basic RRT path planning working.
#
#========================================================================================

#========================================================================================
#			IMPORTS
#========================================================================================

# Importing numpy to manage arrays
import numpy as np
# Importing math to perform complex math operations
import math as m
import rospy
import random as rnd
# Importing the requiered messages to be sended over an action.
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sinfonia_pepper_tools_navigation.msg import location
from Rapidly_random_trees import RapidlyRandomTrees

#========================================================================================
#			AUXILIARY FUNCTIONS
#========================================================================================

#global target
#global currentPosition
#global point_pub
#global size, obstacleList, newTarget, newMap, newGoal, RRTobj

#----------------------------------------------------------------------------------------
# Map message received callback function.
# Parameters:	data	-> Received message.
#----------------------------------------------------------------------------------------
def mapCallback(msg):
	#Obstacle size
	global newMap, finishedMapCallback
	if (newMap):
		newMap = False
		size = msg.info.resolution 
		print("entro al callback")
		#Retreiving map data info and reshaping into array.
		for c in range(0, len(msg.data)-1):
			if msg.data[c] > 50:
				#Hay un obstaculo en msg.data[c]
				y = m.floor(c/msg.info.width)
				x = (c - msg.info.width * (y))

				coordx = msg.info.origin.position.x + size*x
				coordy = msg.info.origin.position.y + size*y

				obs = [coordx, coordy, 3*size]
				obstacleList.append(obs)
		finishedMapCallback = True	
	pass


#----------------------------------------------------------------------------------------
# Map message received callback function.
# Parameters:	data	-> Received message.
#----------------------------------------------------------------------------------------
def odomCallback(data):
	# TODO leer la odometria.
	global currentPosition
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	currentPosition = [x, y]
	#print(currentPosition)
	pass


#----------------------------------------------------------------------------------------
# Map message received callback function.
# Parameters:	data	-> Received message.
#----------------------------------------------------------------------------------------
def targetCallback(data):
	# TODO leer el objetivo.
	global target
	target = [data.x, data.y]
	RRTobj.end = target
	pass


#========================================================================================
#			MAIN NODE FUNCTION
#========================================================================================

#----------------------------------------------------------------------------------------
# Main node function declaration.
# Parameters:	None
#----------------------------------------------------------------------------------------
def sIA_RRT_Node():
	# ROS node initialization.
	rospy.init_node('sIA_RRT', anonymous=True)

	# Publishers declaration. Must declare every publisher as a global variable.
	global point_pub, target, size, obstacleList, newTarget, newMap, newGoal, finishedMapCallback

	newTarget = False
	newMap = True
	newGoal = False
	finishedMapCallback = False
	# Then, create the Publisher object for each publisher.
	point_pub = rospy.Publisher('sIA_move_to_point', location, queue_size=10)

	# Creating the requiered subscriptions to /map, /odom, /sIA_target topics.
	obstacleList = []									
	rospy.Subscriber( '/map', OccupancyGrid, mapCallback )
	rospy.Subscriber( '/odom', Odometry, odomCallback )

	#rospy.Subscriber( '/sIA_target', location, targetCallback )

	# size = 0.5
	# for i in range(0, 10):
	# 	x = rnd.random()*10
	# 	y = rnd.random()*10
	# 	obs = [x,y,size]
	# 	obstacleList.append(obs)


	
	
	rate = rospy.Rate(10)

	# The RRT object declaration.
	RRTobj = RapidlyRandomTrees([0, 0], [4, 0], obstacleList, [-2, 15] )
	# While loop
	while not rospy.is_shutdown():
		#if newTarget or newMap:
			
			#print("finishedMapCallback " + str(finishedMapCallback))
			#print("dentro del if")
			#newTarget = False
			#newMap = False
			
			#print("new map"+str(newMap))

			#RRTobj.end = target
			#RRTobj.start = currentPosition

		if finishedMapCallback:
			RRTobj.obstacleList = obstacleList
			path = RRTobj.Planning(animation = False)
			print(path)
			finishedMapCallback = False

			# Trying to iterate over path points.
			point_num = 0
			while point_num < len(path):
			goal = path[len(path)-2]
			goalLocation = location()
			goalLocation.x = goal[0]
			goalLocation.y = goal[1]
			goalLocation.z = 0
			goalLocation.w = 0

			point_pub.publish(goalLocation)

		#if newGoal:
			#discretizamos el path
			#newGoal = False
			#print(path)
		#	goal = 

		#if m.sqrt((currentPosition[0]-goal[0])** + (currentPosition[1]-goal[1])**) < 0.10:
		#	newGoal = True

        rate.sleep()
    #rospy.spin()


#========================================================================================
#			NODE START
# This is the decent way to start a ROS node in Python
#========================================================================================

if __name__ == "__main__":
	try:
		sIA_RRT_Node()
	except rospy.ROSInterruptException:
		print('An error has ocurred over sIA_RRT node')
		pass