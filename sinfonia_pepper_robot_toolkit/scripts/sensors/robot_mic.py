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

import time
import utils
import rospy
from std_msgs.msg import Float64MultiArray, String


class RobotMic(object):

    def __init__(self, app):
        super(RobotMic, self).__init__()
        self._app = app
        self._app.start()
        self.session = self._app.session

        self._mic = self.session.service("ALAudioDevice")
        self.micData = []
        self._moduleName = "RobotMic"

        self._pub = rospy.Publisher("sIA_mic_raw", Float64MultiArray, queue_size=1)
        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

    def startProcessing(self, channel):
        channel = int(channel)
        if utils.areInRange([channel], [[1, 4]]):
            self._mic.setClientPreferences(self._moduleName, 16000, channel, 0)
            self._mic.subscribe(self._moduleName)
        else:
            self._errorPub.publish("Error 0x00: Value out of range")

    def stopProcessing(self):
        self._mic.unsubscribe(self._moduleName)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        audioMsg = Float64MultiArray()
        self.micData = self.convertStr2SignedInt(inputBuffer)

        audioMsg.data = self.micData
        self._pub.publish(audioMsg)

    def do_nothing(self):
        time.sleep(0.05)

    def convertStr2SignedInt(self, data):
        signedData = []
        ind = 0
        for i in range(0, len(data)/2):
            signedData.append(data[ind]+data[ind+1]*256)
            ind = ind+2

        for i in range(0, len(signedData)):
            if signedData[i] >= 32768:
                signedData[i] = signedData[i]-65536

        for i in range(0, len(signedData)):
            signedData[i] = signedData[i]/32767.0

        return signedData
