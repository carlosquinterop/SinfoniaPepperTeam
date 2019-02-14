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
import argparse
from robot_sensors import RobotSensors
from robot_interaction import RobotInteraction


class RobotToolkitStreamNode:

    def __init__(self, ip):
        rospy.init_node('robot_toolkit_streaming_node', anonymous=True)
        self._rate = rospy.Rate(10)

        self._robotSensors = RobotSensors(ip)

        self._robotSensors.initLasers()
        self._robotSensors.robotLaser.subscribeTopics()

        self._robotSensors.initSonars()
        self._robotSensors.robotSonar.subscribeTopics()

        self._robotInteraction = RobotInteraction(ip)

        self._robotInteraction.initCamera()
        self._robotInteraction.robotCamera.subscribeTopics()
        self._robotInteraction.robotCamera.createPublishers()

        app = qi.Application(["RobotMic", "--qi-url=tcp://" + ip + ":9559"])
        self._robotInteraction.initMic(app)
        self._robotInteraction.robotMic.subscribeTopics()

    def robotToolkitStreamingNode(self):
        while not rospy.is_shutdown():
            if self._robotSensors.robotLaser.checkOn():
                self._robotSensors.robotLaser.getLaserData()
            if self._robotSensors.robotSonar.checkOn():
                self._robotSensors.robotSonar.getSonarData()
            if self._robotInteraction.robotMic.micFlag:
                self._robotInteraction.robotMic.do_nothing()
            if self._robotInteraction.robotCamera.isStreaming:
                self._robotInteraction.robotCamera.streamVideo()
            self._rate.sleep()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--pepper_ip", type=str, default="127.0.0.1",
                            help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
        parser.add_argument("__name", type=str, default="",
                            help="Name of the node.")
        parser.add_argument("__log", type=str, default="",
                            help="Auto generated log file path.")
        args = parser.parse_args()

        node = RobotToolkitStreamNode(args.pepper_ip)
        node.robotToolkitStreamingNode()
    except rospy.ROSInterruptException:
        pass
