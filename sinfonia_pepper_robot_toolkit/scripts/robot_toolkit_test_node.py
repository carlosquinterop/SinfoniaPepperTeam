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
import time
import utils
import rospy
import numpy as np
from PIL import Image
import sounddevice as sd
import scipy.io.wavfile as wavf
from std_msgs.msg import String, Float64MultiArray
from sinfonia_pepper_robot_toolkit.srv import TakePicture
from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector, Wav, T2S


TESTTOPIC = "sIA_t2s"


def robotToolkitTestNode():
    rospy.init_node('robot_toolkit_test_node', anonymous=True)
    rate = rospy.Rate(10)

    if TESTTOPIC == "sIA_move_toward":
        testMoveToward(rate)
    elif TESTTOPIC == "sIA_move_to":
        testMoveTo(rate)
    elif TESTTOPIC == "sIA_laser":
        testLaser(rate)
    elif TESTTOPIC == "sIA_camera":
        testCamera()
    elif TESTTOPIC == "sIA_mic":
        testMic(rate)
    elif TESTTOPIC == "sIA_speakers":
        testSpeakers(rate)
    elif TESTTOPIC == "sIA_t2s":
        testT2S(rate)


def testMoveToward(rate):
    pub = rospy.Publisher("sIA_move_toward", MoveTowardVector, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()
    pub2 = rospy.Publisher("sIA_stop_move", String, queue_size=10)
    while pub2.get_num_connections() == 0:
        rate.sleep()

    # Functionality test
    square = [[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [-1.0, 0.0, 0.0],
              [0.0, -1.0, 0.0]]

    for vertex in square:
        msg = utils.fillVector(vertex, "mtw")
        pub.publish(msg)
        time.sleep(3)

    stopStr = "Stop"
    pub2.publish(stopStr)
    time.sleep(3)

    # Error test
    msg = utils.fillVector([0.0, -2.0, 0.0], "mtw")
    pub.publish(msg)
    time.sleep(3)
    stopStr = "Stap"
    pub2.publish(stopStr)


def testMoveTo(rate):
    pub = rospy.Publisher("sIA_move_to", MoveToVector, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()

    # Functionality test
    square = [[1.0, 0.0, 0.0, 6.0],
              [0.0, 1.0, 0.0, 6.0],
              [-1.0, 0.0, 0.0, 6.0],
              [0.0, -1.0, 0.0, 6.0]]

    for vertex in square:
        msg = utils.fillVector(vertex, "mt")
        pub.publish(msg)
        time.sleep(8)

    # Error test
    msg = utils.fillVector([0.0, -1.0, 4.0, 6.0], "mt")
    pub.publish(msg)


def testLaser(rate):
    pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()

    # Functionality test
    pub.publish("sIA_laser_gl.laser_scan.ON")
    time.sleep(5)
    pub.publish("sIA_laser_gr.laser_scan.ON")
    time.sleep(5)
    pub.publish("sIA_laser_gl.laser_scan.OFF")
    time.sleep(5)
    pub.publish("sIA_laser_gr.laser_scan.OFF")
    time.sleep(5)

    # Error test
    pub.publish("sIA_laser_grlaser_scanOFF")
    time.sleep(1)
    pub.publish("sIA_laser_gr.laserscan.OFF")
    time.sleep(1)


def testCamera():
    rospy.wait_for_service("sIA_take_picture")
    takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)

    # Functionality test
    response = takePicture("Take Picture", [0, 2, 11, 30]).response
    image = Image.frombytes("RGB", (response.width, response.height), str(bytearray(response.data)))
    image.show()
    response = takePicture("Take Picture", [1, 2, 11, 30]).response
    image = Image.frombytes("RGB", (response.width, response.height), str(bytearray(response.data)))
    image.show()

    # Error test
    try:
        takePicture("Take Picture", [0, 2, 18, 30])
    except rospy.service.ServiceException:
        pass
    time.sleep(5)
    try:
        takePicture("Takepicture", [0, 2, 11, 30])
    except rospy.service.ServiceException:
        pass
    time.sleep(5)
    try:
        takePicture("Take Picture", [0, 2, 11])
    except rospy.service.ServiceException:
        pass


def testMic(rate):
    global micData

    micData = []
    pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()

    # Functionality test
    rospy.Subscriber("sIA_mic_raw", Float64MultiArray, testMicCallback)
    pub.publish("sIA_mic_raw.1.ON")
    time.sleep(5)
    pub.publish("sIA_mic_raw.1.OFF")
    sd.play(np.array(micData), 16000, mapping=1, blocking=True)

    # Error test
    time.sleep(1)
    pub.publish("sIA_mic_raw.0.ON")
    time.sleep(1)
    pub.publish("sIA_mic_raw.1.O")


def testMicCallback(data):
    global micData
    micData += data.data


def testSpeakers(rate):
    pub = rospy.Publisher("sIA_play_audio", Wav, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()

    # Functionality test
    msg = Wav()
    path = os.path.dirname(os.path.abspath(__file__)) + "/test_data/demo.wav"
    msg.fs, data = wavf.read(path)
    if len(data.shape) > 1:
        msg.chl = list(data[:, 0])
        msg.chr = list(data[:, 1])
    elif len(data.shape) == 1:
        msg.chl = list(data)

    pub.publish(msg)

    # Error test
    time.sleep(5)
    msg.chl = list(data[:, 0])
    msg.chr = list(data[:-10, 1])
    pub.publish(msg)


def testT2S(rate):
    pub = rospy.Publisher("sIA_say_something", T2S, queue_size=10)
    while pub.get_num_connections() == 0:
        rate.sleep()

    # Functionality test
    msg = T2S()
    msg.text = "Hi, I'm Freezer. I'm part of sinfonIA team for the RoboCup 2019"
    msg.language = "English"
    pub.publish(msg)

    # Functionality test
    time.sleep(1)
    msg.language = "Spanish"
    pub.publish(msg)


if __name__ == '__main__':
    try:
        robotToolkitTestNode()
    except rospy.ROSInterruptException:
        pass
