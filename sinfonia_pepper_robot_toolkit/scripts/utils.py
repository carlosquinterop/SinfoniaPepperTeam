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

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


def areInRange(values, criteria):
    boolean = True
    for value, criterion in zip(values, criteria):
        if not (criterion[0] <= value <= criterion[1]):
            boolean = False

    return boolean


def fillVector(values, dataType):
    msg = None

    if dataType == "v3":
        msg = Vector3()
    elif dataType == 'q':
        msg = Quaternion()
        msg.w = values[3]

    msg.x = values[0]
    msg.y = values[1]
    msg.z = values[2]

    return msg