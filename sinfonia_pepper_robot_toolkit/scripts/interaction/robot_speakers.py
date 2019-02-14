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

import os
import rospy
import paramiko
from naoqi import ALProxy
from std_msgs.msg import String
from sinfonia_pepper_robot_toolkit.msg import File


class RobotSpeakers:

    def __init__(self, ip):
        self._ip = ip
        self._robotPath = "/home/nao/sound.wav"
        self._audioPlayer = ALProxy("ALAudioPlayer", self._ip, 9559)

        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

        self._transport = None
        self._sftp = None

    def subscribeTopics(self):
        rospy.Subscriber("sIA_play_audio", File, self.playAudio)

    def playAudio(self, data):

        if data.extension == "wav":
            path = os.path.dirname(os.path.abspath(__file__)) + "/../.temp/temp." + data.extension
        else:
            self._errorPub.publish("Error 0x03: Unsupported data format [speakers]")
            exit(1)

        with open(path, "wb") as f:
            f.write(data.data)

        self._transport = paramiko.Transport((self._ip, 22))
        self._transport.connect(username="nao", password="nao")
        self._sftp = paramiko.SFTPClient.from_transport(self._transport)

        self._sftp.put(path, self._robotPath)

        self._sftp.close()
        self._transport.close()

        self._audioPlayer.playFile(self._robotPath)
