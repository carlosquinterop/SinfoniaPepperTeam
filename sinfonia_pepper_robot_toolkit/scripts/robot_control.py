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
from sinfonia_pepper_robot_toolkit.srv import ReadJoint, ReadJointResponse
from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector


class RobotControl:

    def __init__(self, ip):
        self._motion = ALProxy("ALMotion", ip, 9559)
        self._motion.moveInit()

        rospy.Service("sIA_read_joint", ReadJoint, self.handleReadJoint)
        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

        self._joints = {"Head": ["HeadPitch", "HeadYaw"],
                        "RShoulder": ["RShoulderRoll", "RShoulderPitch"],
                        "RElbow": ["RElbowRoll", "RElbowYaw"],
                        "RWrist": ["RWristYaw"],
                        "LShoulder": ["LShoulderRoll", "LShoulderPitch"],
                        "LElbow": ["LElbowRoll", "LElbowYaw"],
                        "LWrist": ["LWristYaw"],
                        "Hip": ["HipRoll", "HipPitch"],
                        "Knee": ["KneePitch"]
                        }
        self._motion.setOrthogonalSecurityDistance(0.0)
        self._motion.setTangentialSecurityDistance(0.0)

    def subscribeTopics(self):
        rospy.Subscriber("sIA_move_toward", MoveTowardVector, self.moveTowardCallback)
        rospy.Subscriber("sIA_stop_move", String, self.stopMoveCallback)
        rospy.Subscriber("sIA_move_to", MoveToVector, self.moveToCallback)
        rospy.Subscriber("sIA_set_posture", String, self.setPostureCallback)

    def moveTowardCallback(self, data):
        values = [data.vx, data.vy, data.omega]
        criteria = [[-1, 1], [-1, 1], [-1, 1]]

        if utils.areInRange(values, criteria):
            [vx, vy, omega] = values
            self._motion.post.moveToward(vx, vy, omega)
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        else:
            self._errorPub.publish("Error 0x00: Value out of range [control]")

    def stopMoveCallback(self, data):
        if data.data != "Stop":
            self._errorPub.publish("Error 0x01: Wrong message [control]")
        else:
            self._motion.post.stopMove()

    def moveToCallback(self, data):
        values = [data.x, data.y, data.alpha, data.t]
        criteria = [[-3.14159, 3.14159]]

        if utils.areInRange([values[2]], criteria):
            [x, y, alpha, t] = values
            self._motion.post.moveTo(x, y, alpha, t)
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        else:
            self._errorPub.publish("Error 0x00: Value out of range [control]")

    def handleReadJoint(self, req):
        if req.joint_name in self._joints.keys():
            angles = self._motion.getAngles(self._joints[req.joint_name], False)
            roll = 0.0
            pitch = 0.0
            yaw = 0.0

            for i, angle in enumerate(angles):
                if "Roll" in self._joints[req.joint_name][i]:
                    roll = angle
                if "Pitch" in self._joints[req.joint_name][i]:
                    pitch = angle
                if "Yaw" in self._joints[req.joint_name][i]:
                    yaw = angle

            return ReadJointResponse(roll, pitch, yaw)
        else:
            self._errorPub.publish("Error 0x01: Wrong message [control]")

    def setPostureCallback(self, data):
        posture = data.data

        self._posture.goToPosture(posture, 1.0)