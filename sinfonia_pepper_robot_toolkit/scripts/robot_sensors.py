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

from sensors.robot_lasers import RobotLasers
from sensors.robot_sonars import RobotSonars


class RobotSensors:

    def __init__(self, ip):
        self._ip = ip

        self.robotLaser = None
        self.robotSonar = None

    def initLasers(self):
        self.robotLaser = RobotLasers(ip=self._ip)
        self.robotLaser.createMessages()
        self.robotLaser.createPublishers()

    def initSonars(self):
        self.robotSonar = RobotSonars(ip=self._ip)
        self.robotSonar.createMessages()
        self.robotSonar.createPublishers()
