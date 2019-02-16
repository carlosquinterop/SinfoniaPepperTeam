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
import cv2
import time
import utils
import rospy
import struct
import argparse
import numpy as np
import sounddevice as sd
from PIL import Image as im
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from sinfonia_pepper_robot_toolkit.srv import TakePicture, ReadJoint
from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector, Wav, T2S, File


class RobotToolkitTestNode:

    def __init__(self, testTopic):
        rospy.init_node('robot_toolkit_test_node', anonymous=True)

        self._rate = rospy.Rate(10)
        self._testTopic = testTopic

        self.micData = []
        self.bridge = None
        self.t0 = 0

    def robotToolkitTestNode(self):

        if self._testTopic == "sIA_move_toward":
            self.testMoveToward()
        elif self._testTopic == "sIA_move_to":
            self.testMoveTo()
        elif self._testTopic == "sIA_laser":
            self.testLaser()
        elif self._testTopic == "sIA_camera":
            self.testCamera()
        elif self._testTopic == "sIA_mic":
            self.testMic()
        elif self._testTopic == "sIA_speakers":
            self.testSpeakers()
        elif self._testTopic == "sIA_t2s":
            self.testT2S()
        elif self._testTopic == "sIA_video":
            self.testVideo()
        elif self._testTopic == "sIA_depth_camera":
            self.testDepthCamera()
        elif self._testTopic == "sIA_sonars":
            self.testSonars()
        elif self._testTopic == "sIA_merge":
            self.testMerge()
        elif self._testTopic == "sIA_read_joint":
            self.testReadJoint()

    def testMoveToward(self):
        pub = rospy.Publisher("sIA_move_toward", MoveTowardVector, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()
        pub2 = rospy.Publisher("sIA_stop_move", String, queue_size=10)
        while pub2.get_num_connections() == 0:
            self._rate.sleep()

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

    def testMoveTo(self):
        pub = rospy.Publisher("sIA_move_to", MoveToVector, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

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

    def testLaser(self):
        pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

        # Functionality test
        pub.publish("sIA_laser_gl.laser_scan.ON")
        time.sleep(5)
        pub.publish("sIA_laser_gr.laser_scan.ON")
        time.sleep(5)
        pub.publish("sIA_laser_gl.laser_scan.OFF")
        time.sleep(5)
        pub.publish("sIA_laser_gr.laser_scan.OFF")
        time.sleep(5)

        # # Error test
        pub.publish("sIA_laser_grlaser_scanOFF")
        time.sleep(1)
        pub.publish("sIA_laser_gr.laserscan.OFF")
        time.sleep(1)

    def testCamera(self):
        rospy.wait_for_service("sIA_take_picture")
        takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)

        # Functionality test
        response = takePicture("Take Picture", [0, 2, 11, 15]).response
        image = im.frombytes("RGB", (response.width, response.height), str(bytearray(response.data)))
        image.show()
        response = takePicture("Take Picture", [1, 2, 11, 15]).response
        image = im.frombytes("RGB", (response.width, response.height), str(bytearray(response.data)))
        image.show()

        # Error test
        try:
            takePicture("Take Picture", [0, 2, 18, 30])
        except rospy.service.ServiceException:
            pass
        time.sleep(1)
        try:
            takePicture("Takepicture", [0, 2, 11, 30])
        except rospy.service.ServiceException:
            pass
        time.sleep(1)
        try:
            takePicture("Take Picture", [0, 2, 11])
        except rospy.service.ServiceException:
            pass

    def testMic(self):
        pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

        # Functionality test
        rospy.Subscriber("sIA_mic_raw", Float64MultiArray, self.testMicCallback)
        pub.publish("sIA_mic_raw.1.ON")
        time.sleep(5)
        pub.publish("sIA_mic_raw.1.OFF")
        sd.play(np.array(self.micData), 16000, mapping=1, blocking=True)

        # Error test
        time.sleep(1)
        pub.publish("sIA_mic_raw.0.ON")
        time.sleep(1)
        pub.publish("sIA_mic_raw.1.O")

    def testMicCallback(self, data):
        self.micData += data.data

    def testSpeakers(self):
        pub = rospy.Publisher("sIA_play_audio", File, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

        # Functionality test
        msg = File()
        path = os.path.dirname(os.path.abspath(__file__)) + "/test_data/demo.wav"
        with open(path, "rb") as f:
            msg.data = f.read()

        msg.extension = path.split('.')[-1]
        pub.publish(msg)

        # Error test
        time.sleep(1)
        msg.extension = "mp3"
        pub.publish(msg)

    def testT2S(self):
        pub = rospy.Publisher("sIA_say_something", T2S, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

        # Functionality test
        msg = T2S()
        msg.text = "Hi, I'm Freezer. I'm part of sinfonIA team for the RoboCup 2019"
        msg.language = "English"
        pub.publish(msg)

        # Error test
        time.sleep(1)
        msg.language = "Spanish"
        pub.publish(msg)

    def testVideo(self):
        self.bridge = CvBridge()

        pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()
        rospy.Subscriber("sIA_video_stream", Image, self.testVideoCallback)

        # Functionality test
        pub.publish("sIA_video_stream.0.1.11.30.ON")
        self.t0 = time.time()
        time.sleep(5)
        pub.publish("sIA_video_stream.0.1.11.30.OFF")

        # Error test
        time.sleep(1)
        pub.publish("sIA_video_stream.0.1.11.ON")
        time.sleep(1)
        pub.publish("sIA_video_stream.3.1.11.30.ON")
        time.sleep(1)
        pub.publish("sIA_video_stream.0.1.11.30.O")
        time.sleep(1)
        pub.publish("sIA_video_stream.0.a.11.30.ON")

    def testVideoCallback(self, image):
        framerate = int(1/(time.time()-self.t0))
        self.t0 = time.time()
        frame = self.bridge.imgmsg_to_cv2(image, "rgb8")
        print("Framerate: " + str(framerate) + " fps")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.namedWindow("Video")
        cv2.imshow("Video", frame)
        cv2.waitKey(10)

    def testDepthCamera(self):
        rospy.wait_for_service("sIA_take_picture")
        takePicture = rospy.ServiceProxy("sIA_take_picture", TakePicture)

        # Functionality test
        distances = []
        response = takePicture("Take Picture", [2, 1, 17, 1]).response
        for i in range(0, len(response.data) - 1, 2):
            distances.append(struct.unpack('<H', response.data[i:i + 2])[0])

        distArray = np.array(distances, dtype=float).reshape(240, 320)
        normDistArray = (distArray * 255) / np.max(distArray)
        normDistArray = cv2.convertScaleAbs(normDistArray)
        image = im.fromarray(255 - normDistArray)
        image.show()

        # Error test
        try:
            takePicture("Take Picture", [2, 1, 17, 30])
        except rospy.service.ServiceException:
            pass

    def testSonars(self):
        pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

        pub.publish("sIA_sonar_front.ON")
        pub.publish("sIA_sonar_back.ON")
        time.sleep(30)
        pub.publish("sIA_sonar_front.OFF")
        pub.publish("sIA_sonar_back.OFF")

        # Error test
        pub.publish("sIA_sonar_font.OFF")
        pub.publish("sIA_sonar_back.ONN")

    def testMerge(self):
        pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
        while pub.get_num_connections() == 0:
            self._rate.sleep()

        pub.publish("sIA_laser_merge.ON")
        time.sleep(30)
        pub.publish("sIA_laser_merge.OFF")

    def testReadJoint(self):
        rospy.wait_for_service("sIA_read_joint")
        readJoint = rospy.ServiceProxy("sIA_read_joint", ReadJoint)

        # Functionality test
        joints = ["Head", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist", "Hip", "Knee"]

        for joint in joints:
            response = readJoint(joint)
            print(joint + ":\n\tRoll: " + str(response.roll) + "\n\tPitch: " + str(response.pitch) + "\n\tYaw: "
                  + str(response.yaw) + "\n")


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--test_topic", type=str, default="",
                            help="Topic to test.")
        parser.add_argument("__name", type=str, default="",
                            help="Name of the node.")
        parser.add_argument("__log", type=str, default="",
                            help="Auto generated log file path.")
        args = parser.parse_args()

        node = RobotToolkitTestNode(args.test_topic)
        node.robotToolkitTestNode()
    except rospy.ROSInterruptException:
pass
