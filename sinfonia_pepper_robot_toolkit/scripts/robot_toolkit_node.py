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
import argparse
from robot_control import RobotControl
from robot_interaction import RobotInteraction


class RobotToolkitNode:

    def __init__(self, ip):
        rospy.init_node('robot_toolkit_node', anonymous=True)

        self._robotControl = RobotControl(ip)
        self._robotControl.subscribeTopics()

        self._robotInteraction = RobotInteraction(ip)
        self._robotInteraction.initSpeakers()
        self._robotInteraction.robotSpeakers.subscribeTopics()

        self._robotInteraction.initT2S()
        self._robotInteraction.robotT2S.subscribeTopics()

        self._robotInteraction.initLEDs()
        self._robotInteraction.robotLEDs.subscribeTopics()

    def robotToolkitNode(self):
        rospy.spin()


if __name__ == '__main__':

    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--pepper_ip", type=str, default="127.0.0.1",
                            help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
        parser.add_argument("__name", type=str, default="",
                            help="Name of the node")
        parser.add_argument("__log", type=str, default="",
                            help="Auto generated log file path.")
        args = parser.parse_args()

        node = RobotToolkitNode(args.pepper_ip)
        node.robotToolkitNode()
    except rospy.ROSInterruptException:
        pass
