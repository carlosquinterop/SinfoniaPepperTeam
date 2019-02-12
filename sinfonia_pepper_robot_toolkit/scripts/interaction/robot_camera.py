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

import utils
import rospy
import random
import numpy as np
from naoqi import ALProxy
from PIL import Image as im
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sinfonia_pepper_robot_toolkit.srv import TakePicture, TakePictureResponse


class RobotCamera:

    def __init__(self, ip):
        self._camera = ALProxy("ALVideoDevice", ip, 9559)
        self._ip = ip
        self._bridge = CvBridge()

        rospy.Service("sIA_take_picture", TakePicture, self.handleTakePicture)
        self._errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)

        self.name = None
        self.cameraParams = []
        self.isStreaming = False

        self.videoPublisher = None

    def subscribeTopics(self):
        rospy.Subscriber("sIA_stream_from", String, self.handleStreamVideo)

    def createPublishers(self):
        self.videoPublisher = rospy.Publisher("sIA_video_stream", Image, queue_size=1)

    def subscribeCamera(self):
        self.name = str(self._ip + "_" + str(random.randint(0, 99999)))
        self.name = self._camera.subscribeCamera(self.name, self.cameraParams[0], self.cameraParams[1],
                                                 self.cameraParams[2], self.cameraParams[3])

    def unsubscribeCamera(self):
        self._camera.unsubscribe(self.name)
        self.name = None

    def takePicture(self):
        self.subscribeCamera()
        image = self._camera.getImageRemote(self.name)
        image = im.frombytes("RGB", (int(image[0]), int(image[1])), image[6])
        self.unsubscribeCamera()

        return image

    def streamVideo(self):

        try:
            if self.name is None:
                self.subscribeCamera()
            image = self._camera.getImageRemote(self.name)
            image = im.frombytes("RGB", (int(image[0]), int(image[1])), image[6])
            image = self._bridge.cv2_to_imgmsg(np.array(image, 'uint8'), "rgb8")
            self.videoPublisher.publish(image)
        except TypeError:
            self._errorPub.publish("Error 0x06: Camera closed beforehand")
            return

    def takeDepthMap(self):
        self.subscribeCamera()
        image = self._camera.getImageRemote(self.name)
        self.unsubscribeCamera()

        img = Image()
        img.header.stamp = rospy.Time(image[4] + image[5] * 1e-6)
        img.header.frame_id = "2"
        img.height = image[1]
        img.width = image[0]
        nbLayers = image[2]
        img.encoding = "16UC1"
        img.step = img.width * nbLayers
        img.data = image[6]

        return img

    def handleTakePicture(self, req):

        if req.command == "Take Picture":
            if self.name is None:
                if len(req.params) != 4:
                    self._errorPub.publish("Error 0x02: Wrong number of params")
                else:
                    params = list(req.params)
                    if utils.checkCameraSettings(params, "camera"):
                        self.cameraParams = params
                        if utils.areInRange([params[0]], [[0, 1]]):
                            image = self.takePicture()
                            return TakePictureResponse(self._bridge.cv2_to_imgmsg(np.array(image, 'uint8'), "rgb8"))
                        elif params[0] == 2:
                            msg = self.takeDepthMap()
                            return TakePictureResponse(msg)
                    else:
                        self._errorPub.publish("Error 0x00: Value out of range")
            else:
                self._errorPub.publish("Error 0x05: Resource is already in use")
        else:
            self._errorPub.publish("Error 0x01: Wrong message")

    def handleStreamVideo(self, data):

        if "sIA_video_stream" in data.data:
            request = data.data.split(".")[1:]
            if len(request) == 5:
                state = request[-1]

                try:
                    params = map(int, request[:-1])
                except ValueError:
                    self._errorPub.publish("Error 0x01: Wrong message")
                    return
                if state == "ON":
                    if utils.checkCameraSettings(params, "video"):
                        self.cameraParams = params
                        self.isStreaming = True
                    else:
                        self._errorPub.publish("Error 0x00: Value out of range")
                elif state == "OFF":
                    self.unsubscribeCamera()
                    self.isStreaming = False
                else:
                    self._errorPub.publish("Error 0x01: Wrong message")
            else:
                self._errorPub.publish("Error 0x02: Wrong number of params")
        else:
            self._errorPub.publish("Error 0x01: Wrong message")
