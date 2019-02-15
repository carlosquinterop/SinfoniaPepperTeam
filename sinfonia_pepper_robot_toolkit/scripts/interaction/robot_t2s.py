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
from sinfonia_pepper_robot_toolkit.msg import T2S


class RobotT2S:

    def __init__(self, ip):
        self._t2s = ALProxy("ALTextToSpeech", ip, 9559)

        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

    def subscribeTopics(self):
        rospy.Subscriber("sIA_say_something", T2S, self.saySomething)

    def setLanguage(self, lang):
        self._t2s.setLanguage(lang)

    def saySomething(self, data):

        if data.language == "English":
            self.setLanguage(data.language)
            self._t2s.say(data.text)
        else:
            self._errorPub.publish("Error 0x04: Language not supported [t2s]")
