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
from sinfonia_pepper_robot_toolkit.msg import LEDs


class RobotLEDs:

    def __init__(self, ip):
        self._leds = ALProxy("ALLeds", ip, 9559)

        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

    def subscribeTopics(self):
        rospy.Subscriber("sIA_leds", LEDs, self.callback)

    def callback(self, data):
        if "Ear" in data.name:
            self._leds.fadeRGB(data.name, 0, 0, data.b / 255, data.t)
        else:
            self._leds.fadeRGB(data.name, float(data.r) / 255.0, float(data.g) / 255.0, float(data.b) / 255.0, data.t)
