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
from robot_control import RobotControl
from robot_interaction import RobotInteraction


IP = "10.25.205.82"


class RobotToolkitNode:

    def __init__(self):
        rospy.init_node('robot_toolkit_node', anonymous=True)

        self._robotControl = RobotControl(IP)
        self._robotControl.subscribeTopics()

        self._robotInteraction = RobotInteraction(IP)
        self._robotInteraction.initSpeakers()
        self._robotInteraction.robotSpeakers.subscribeTopics()

    def robotToolkitNode(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = RobotToolkitNode()
        node.robotToolkitNode()
    except rospy.ROSInterruptException:
        pass
