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

from interaction.robot_t2s import RobotT2S
from interaction.robot_mic import RobotMic
from interaction.robot_camera import RobotCamera
from interaction.robot_speakers import RobotSpeakers
from interaction.robot_leds import RobotLEDs


class RobotInteraction:

    def __init__(self, ip):
        self._ip = ip

        self.robotCamera = None
        self.robotMic = None
        self.robotSpeakers = None
        self.robotT2S = None
        self.robotLEDs = None

    def initCamera(self):
        self.robotCamera = RobotCamera(ip=self._ip)

    def initMic(self, app):
        self.robotMic = RobotMic(app=app)
        self.robotMic.session.registerService("RobotMic", self.robotMic)

    def initSpeakers(self):
        self.robotSpeakers = RobotSpeakers(ip=self._ip)

    def initT2S(self):
        self.robotT2S = RobotT2S(ip=self._ip)

    def initLEDs(self):
        self.robotLEDs = RobotLEDs(ip=self._ip)
