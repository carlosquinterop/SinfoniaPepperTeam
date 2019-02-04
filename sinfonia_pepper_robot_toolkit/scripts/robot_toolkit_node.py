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
from std_msgs.msg import String
from robot_control import RobotControl

IP = "192.168.0.100"


def robotToolkitNode():
    rospy.init_node('robot_toolkit_node', anonymous=True)
    rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

    robotControl = RobotControl(IP)
    robotControl.initTopics()
    robotControl.subscribeTopics()

    rospy.spin()


if __name__ == '__main__':
    try:
        robotToolkitNode()
    except rospy.ROSInterruptException:
        pass
