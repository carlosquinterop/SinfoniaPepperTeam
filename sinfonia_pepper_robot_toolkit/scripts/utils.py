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

from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector


def areInRange(values, criteria):
    boolean = True
    for value, criterion in zip(values, criteria):
        if not (criterion[0] <= value <= criterion[1]):
            boolean = False

    return boolean


def fillVector(values, dataType):
    msg = None

    if dataType == "mtw":
        msg = MoveTowardVector()
        msg.vx = values[0]
        msg.vy = values[1]
        msg.omega = values[2]
    elif dataType == "mt":
        msg = MoveToVector()
        msg.x = values[0]
        msg.y = values[1]
        msg.alpha = values[2]
        msg.t = values[3]

    return msg


def checkCameraSettings(params):
    if areInRange([params[1]], [[0, 2]]):
        criteria = [[0, 1], [0, 4], [0, 16], [1, 30]]
    elif areInRange([params[1]], [[3, 4]]):
        criteria = [[0, 1], [0, 4], [0, 16], [1, 1]]

    return areInRange(params, criteria)
