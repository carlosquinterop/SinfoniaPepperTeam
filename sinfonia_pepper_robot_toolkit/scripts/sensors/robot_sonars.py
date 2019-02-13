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
from naoqi import ALProxy
from std_msgs.msg import String
from sensor_msgs.msg import Range


class RobotSonars:

    def __init__(self, ip):
        self.ip = ip

        self.sonarFreq = 10
        self.sonarMinRange = 0.25
        self.sonarMaxRange = 2.55
        self.sonarFov = 1.05

        self.sonarRateParameter = "~sonar_rate"
        self.pepperSonarRate = 1.05

        self.sonarRate = rospy.Rate(rospy.get_param(self.sonarRateParameter, self.pepperSonarRate))

        self.sonarProxy = None
        self.memProxy = None
        self.connectNaoQi()

        self.frontSonarMessage = None
        self.backSonarMessage = None
        self.createMessages()

        self.frontSonarMemKey = "Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value"
        self.backSonarMemKey = "Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value"

        self.frontSonarPublisher = None
        self.backSonarPublisher = None

        self._sonars = {"front": False, "back": False}

        self._sonarsStates = {"ON": True, "OFF": False}

        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

    def subscribeTopics(self):
        rospy.Subscriber("sIA_stream_from", String, self.callback)

    def connectNaoQi(self):

        self.sonarProxy = ALProxy("ALSonar", self.ip, 9559)
        self.memProxy = ALProxy("ALMemory", self.ip, 9559)
        self.sonarProxy.subscribe('ros_sonar_subscription')

    def createMessages(self):
        self.frontSonarMessage = Range()
        self.backSonarMessage = Range()

        self.frontSonarMessage.header.frame_id = "sonar_front_frame"
        self.frontSonarMessage.min_range = self.sonarMinRange
        self.frontSonarMessage.max_range = self.sonarMaxRange
        self.frontSonarMessage.field_of_view = self.sonarFov
        self.frontSonarMessage.radiation_type = Range.ULTRASOUND

        self.backSonarMessage.header.frame_id = "sonar_back_frame"
        self.backSonarMessage.min_range = self.sonarMinRange
        self.backSonarMessage.max_range = self.sonarMaxRange
        self.backSonarMessage.field_of_view = self.sonarFov
        self.backSonarMessage.radiation_type = Range.ULTRASOUND

    def createPublishers(self):

        self.frontSonarPublisher = rospy.Publisher("sIA_sonar_front", Range, queue_size=5)
        self.backSonarPublisher = rospy.Publisher("sIA_sonar_back", Range, queue_size=5)

    def callback(self, data):

        if "sonar" in data.data:
            try:
                sonar = data.data.split('.')[0].split('_')[-1]
                state = data.data.split('.')[-1]

                if (sonar in self._sonars.keys()) and (state in self._sonarsStates.keys()):
                    self._sonars[sonar] = self._sonarsStates[state]
                else:
                    self._errorPub.publish("Error 0x01: Wrong message [sonars]")
            except:
                self._errorPub.publish("Error 0x01: Wrong message [sonars]")
                return

    def checkOn(self):

        return any(self._sonars.values())

    def getSonarData(self):

        if self._sonars["front"]:
            self.frontSonarMessage.header.stamp = rospy.Time.now()
            self.frontSonarMessage.range = self.memProxy.getData(self.frontSonarMemKey)
            self.frontSonarPublisher.publish(self.frontSonarMessage)

        if self._sonars["back"]:
            self.backSonarMessage.header.stamp = rospy.Time.now()
            self.backSonarMessage.range = self.memProxy.getData(self.backSonarMemKey)
            self.backSonarPublisher.publish(self.backSonarMessage)

        self.sonarRate.sleep()
