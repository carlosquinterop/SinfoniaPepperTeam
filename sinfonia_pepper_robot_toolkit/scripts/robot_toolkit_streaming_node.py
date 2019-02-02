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
from robot_sensors import RobotSensors

IP = "192.168.0.101"


class RobotToolkitStream:

    def __init__(self):
        rospy.init_node('robot_toolkit_streaming_node', anonymous=True)
        self._pub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)
        self._rate = rospy.Rate(10)
        self._flag = ""
        self._robotSensors = RobotSensors(IP)

        self._laserTypes = {"pointCloud": [True, False],
                            "laserScan": [False, True]}
        self._laserStates = {"ON": True,
                             "OFF": False}

    def robotToolkitStreamingNode(self):
        rospy.Subscriber("sIA_stream_from", String, self.testCallback)

        while not rospy.is_shutdown():
            if self._robotSensors.robotLaser.checkOn():
                self._robotSensors.robotLaser.getLaserData()
            self._rate.sleep()

    def testCallback(self, data):
        laser = data.data.split('.')[0].split('_')[-1]
        type = data.data.split('.')[-2]
        state = data.data.split('.')[-1]
        self._robotSensors.robotLaser.setType(self._laserTypes[type][0], self._laserTypes[type][1])
        self._robotSensors.robotLaser.setLaser(laser, self._laserStates[state])


if __name__ == '__main__':
    try:
        s = RobotToolkitStream()
        s.robotToolkitStreamingNode()
    except rospy.ROSInterruptException:
        pass
