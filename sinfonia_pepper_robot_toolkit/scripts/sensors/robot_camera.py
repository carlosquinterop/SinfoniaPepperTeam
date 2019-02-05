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

import random
import numpy as np
from PIL import Image
from naoqi import ALProxy
from sinfonia_pepper_robot_toolkit.srv import TakePicture, TakePictureResponse
from cv_bridge import CvBridge


class RobotCamera:

    def __init__(self, ip):
        self._camera = ALProxy("ALVideoDevice", ip, 9559)
        self._ip = ip
        self._bridge = CvBridge()

    def takePicture(self):
        name = str(self._ip + "_" + str(random.randint(0, 99999)))
        name = self._camera.subscribeCamera(name, 0, 2, 11, 30)
        self._camera.openCamera(0)
        self._camera.startCamera(0)

        image = self._camera.getImageRemote(name)
        image = Image.frombytes("RGB", (int(image[0]), int(image[1])), image[6])
        self._camera.unsubscribe(name)
        return image

    def handleTakePicture(self, req):
        if req.command == "Take Picture":
            image = self.takePicture()
            return TakePictureResponse(self._bridge.cv2_to_imgmsg(np.array(image, 'uint8'), "rgb8"))
