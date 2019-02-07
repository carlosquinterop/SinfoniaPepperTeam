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

import utils
import rospy
from naoqi import ALProxy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion


class RobotControl:

    def __init__(self, ip):
        self._traction = ALProxy("ALMotion", ip, 9559)
        self._traction.moveInit()

        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

    def subscribeTopics(self):
        rospy.Subscriber("sIA_moveToward", Vector3, self.moveTowardCallback)
        rospy.Subscriber("sIA_stopMove", String, self.stopMoveCallback)
        rospy.Subscriber("sIA_moveTo", Quaternion, self.moveToCallback)

    def moveTowardCallback(self, data):
        values = [data.x, data.y, data.z]
        criteria = [[-1, 1], [-1, 1], [-1, 1]]

        if utils.areInRange(values, criteria):
            [vx, vy, theta] = values
            self._traction.post.moveToward(vx, vy, theta)
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        else:
            self._errorPub.publish("Error 0x00: Value out of range")

    def stopMoveCallback(self, data):
        if data.data != "Stop":
            self._errorPub.publish("Error 0x01: Wrong message")
        else:
            self._traction.post.stopMove()

    def moveToCallback(self, data):
        values = [data.x, data.y, data.z, data.w]
        criteria = [[-3.14159, 3.14159]]

        if utils.areInRange([values[2]], criteria):
            [vx, vy, theta, secs] = values
            self._traction.post.moveTo(vx, vy, theta, secs)
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        else:
            self._errorPub.publish("Error 0x00: Value out of range")
