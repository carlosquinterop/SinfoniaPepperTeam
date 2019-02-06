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

import qi
import rospy
from std_msgs.msg import String
from robot_sensors import RobotSensors
from robot_interaction import RobotInteraction
from sinfonia_pepper_robot_toolkit.srv import TakePicture


IP = "192.168.0.101"


class RobotToolkitStream:

    def __init__(self):
        rospy.init_node('robot_toolkit_streaming_node', anonymous=True)
        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)
        self._rate = rospy.Rate(10)

        self._robotSensors = RobotSensors(IP)

        connectionUrl = "tcp://" + IP + ":9559"
        app = qi.Application(["RobotMic", "--qi-url=" + connectionUrl])
        self._robotInteraction = RobotInteraction(IP, app)

        rospy.Service("sIA_takePicture", TakePicture, self._robotInteraction.robotCamera.handleTakePicture)

        self._laserTypes = {"pointCloud": [True, False],
                            "laserScan": [False, True]}
        self._laserStates = {"ON": True,
                             "OFF": False}

        self.micFlag = False

    def robotToolkitStreamingNode(self):
        rospy.Subscriber("sIA_stream_from", String, self.callback)

        while not rospy.is_shutdown():
            if self._robotSensors.robotLaser.checkOn():
                self._robotSensors.robotLaser.getLaserData()
            if self.micFlag:
                self._robotInteraction.robotMic.do_nothing()
            self._rate.sleep()

    def callback(self, data):
        if "laser" in data.data:
            laser = data.data.split('.')[0].split('_')[-1]
            type = data.data.split('.')[-2]
            state = data.data.split('.')[-1]
            self._robotSensors.robotLaser.setType(self._laserTypes[type][0], self._laserTypes[type][1])
            self._robotSensors.robotLaser.setLaser(laser, self._laserStates[state])
        elif "mic" in data.data:
            channel = data.data.split('.')[-2]
            state = data.data.split('.')[-1]
            if state == "ON":
                self._robotInteraction.robotMic.startProcessing(channel)
                self.micFlag = True
            elif state == "OFF":
                self._robotInteraction.robotMic.stopProcessing()


if __name__ == '__main__':
    try:
        node = RobotToolkitStream()
        node.robotToolkitStreamingNode()
    except rospy.ROSInterruptException:
        pass
