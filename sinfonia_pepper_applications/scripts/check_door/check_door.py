#!/usr/bin/env python
# license removed for brevity

"""
//======================================================================//
//  This software is free: you can redistribute it and/or modify        //
//  it under the terms of the GNU General Public License Version 3,     //
//  as published by the Free Software Foundation.                       //
//  This software is distributed in the hope that it will be useful,    //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
//  GNU General Public License for more details.                        //
//  You should have received a copy of the GNU General Public License   //
//  Version 3 in the file COPYING that came with this distribution.     //
//  If not, see <http://www.gnu.org/licenses/>                          //
//======================================================================//
//                                                                      //
//      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //
//      Sinfonia - Colombia                                             //
//      https://sinfoniateam.github.io/sinfonia/index.html              //
//                                                                      //
//======================================================================//
"""

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String


class CheckDoor:

    def __init__(self):

        self._rate = rospy.Rate(10)
        self.isDoorOpen = False
        self.patience = 3
        self.counts = 0
        self.pub = None
        self.distThreshold = 0.7


    def subscribeTopics(self):
        rospy.Subscriber("sIA_sonar_front", Range, self.callback)

    def initPublishers(self):
        self.pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        while self.pub.get_num_connections() == 0:
            self._rate.sleep()

    def startSonar(self):
        self.pub.publish("sIA_sonar_front.ON")

    def stopSonar(self):
        self.pub.publish("sIA_sonar_front.OFF")


    def callback(self, data):

        if data.range > self.distThreshold:
            self.counts += 1
        if self.counts >= self.patience:
            self.isDoorOpen = True

    def checkDoor(self):

        return self.isDoorOpen
