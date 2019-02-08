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
from robot_sensors import RobotSensors
from robot_interaction import RobotInteraction


IP = "10.25.205.82"


class RobotToolkitStreamNode:

    def __init__(self):
        rospy.init_node('robot_toolkit_streaming_node', anonymous=True)
        self._rate = rospy.Rate(10)

        self._robotSensors = RobotSensors(IP)

        self._robotSensors.initLasers()
        self._robotSensors.robotLaser.subscribeTopics()

        self._robotInteraction = RobotInteraction(IP)

        self._robotInteraction.initCamera()

        app = qi.Application(["RobotMic", "--qi-url=tcp://" + IP + ":9559"])
        self._robotInteraction.initMic(app)
        self._robotInteraction.robotMic.subscribeTopics()

    def robotToolkitStreamingNode(self):
        while not rospy.is_shutdown():
            if self._robotSensors.robotLaser.checkOn():
                self._robotSensors.robotLaser.getLaserData()
            if self._robotInteraction.robotMic.micFlag:
                self._robotInteraction.robotMic.do_nothing()
            self._rate.sleep()


if __name__ == '__main__':
    try:
        node = RobotToolkitStreamNode()
        node.robotToolkitStreamingNode()
    except rospy.ROSInterruptException:
        pass
