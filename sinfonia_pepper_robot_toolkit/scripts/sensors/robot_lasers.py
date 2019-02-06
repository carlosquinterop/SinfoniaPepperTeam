#!/usr/bin/env python
# license removed for brevity

# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import math
import rospy
import struct
from naoqi import ALProxy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


class RobotLasers:

    # PEPPER laser specs
    # see https://community.aldebaran.com/doc/2-1/family/juliette_technical/laser_juliette.html#juliette-laser
    PEPPER_LASER_FREQ = 6                       # [hZ] --> check on that
    PEPPER_LASER_MIN_ANGLE = -0.523598776          # [rad]
    PEPPER_LASER_MAX_ANGLE = 0.523598776           # [radi]
    PEPPER_LASER_FOV = math.fabs(PEPPER_LASER_MIN_ANGLE) + math.fabs(PEPPER_LASER_MAX_ANGLE)

    PEPPER_LASER_MIN_RANGE = 0.1                   # [m] --> no spec given here
    PEPPER_LASER_MAX_RANGE = 5.0                   # [m] --> same here, 5m as quality guess

    # FRONT GROUND LASERS
    PEPPER_LASER_GROUND_SHOVEL_POINTS = 3
    PEPPER_LASER_GROUND_LEFT_POINTS = 1
    PEPPER_LASER_GROUND_RIGHT_POINTS = 1
    # SURROUNDING LASER
    PEPPER_LASER_SRD_POINTS = 15

    # memory key to fetch laser readings from
    # see memory key listing https://community.aldebaran.com/doc/2-1/family/juliette_technical/juliette_dcm/actuator_sensor_names.html#lasers
    # iterate over all segments e.g./SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg01/X/Value
    PEPPER_MEM_KEY_GROUND_SHOVEL = 'Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/'
    PEPPER_MEM_KEY_GROUND_LEFT = 'Device/SubDeviceList/Platform/LaserSensor/Front/Vertical/Left/'
    PEPPER_MEM_KEY_GROUND_RIGHT = 'Device/SubDeviceList/Platform/LaserSensor/Front/Vertical/Right/'
    PEPPER_MEM_KEY_SRD_FRONT = 'Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/'
    PEPPER_MEM_KEY_SRD_LEFT = 'Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/'
    PEPPER_MEM_KEY_SRD_RIGHT = 'Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/'

    # ROS params to check
    # acc. to spec: 40 kHz
    PARAM_LASER_RATE = '~laser_rate'
    PARAM_LASER_RATE_DEFAULT = PEPPER_LASER_FREQ

    # frame id to publish
    PARAM_LASER_SHOVEL_FRAME = '~laser_shovel_frame_id'
    PARAM_LASER_SHOVEL_FRAME_DEFAULT = 'ShovelLaser_frame'

    PARAM_LASER_GROUND_LEFT_FRAME = '~laser_ground_left_frame_id'
    PARAM_LASER_GROUND_LEFT_FRAME_DEFAULT = 'VerticalLeftLaser_frame'

    PARAM_LASER_GROUND_RIGHT_FRAME = '~laser_ground_right_frame_id'
    PARAM_LASER_GROUND_RIGHT_FRAME_DEFAULT = 'VerticalRightLaser_frame'

    PARAM_LASER_SRD_FRONT_FRAME = '~laser_srd_front_frame_id'
    PARAM_LASER_SRD_FRONT_FRAME_DEFAULT = 'SurroundingFrontLaser_frame'

    PARAM_LASER_SRD_LEFT_FRAME = '~laser_srd_left_frame_id'
    PARAM_LASER_SRD_LEFT_FRAME_DEFAULT = 'SurroundingLeftLaser_frame'

    PARAM_LASER_SRD_RIGHT_FRAME = '~laser_srd_right_frame_id'
    PARAM_LASER_SRD_RIGHT_FRAME_DEFAULT = 'SurroundingRightLaser_frame'

    TOPIC_LASER_SHOVEL = '~/sIA_laser_shovel/'
    TOPIC_LASER_GROUND_LEFT = '~/sIA_laser_gl/'
    TOPIC_LASER_GROUND_RIGHT = '~/sIA_laser_gr/'
    TOPIC_LASER_SRD_FRONT = '~/sIA_laser_srdf/'
    TOPIC_LASER_SRD_LEFT = '~/sIA_laser_srdl/'
    TOPIC_LASER_SRD_RIGHT = '~/sIA_laser_srdr/'

    PEPPER_LASER_SUB_NAME = 'pepper_ros_laser'

    def __init__(self, ip):
        self.pointcloud = None
        self.laserscan = None

        self.laserProxy = None
        self.memProxy = None

        self.connectNaoQi(ip)

        # default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
        self.laserRate = rospy.Rate(rospy.get_param(
                            self.PARAM_LASER_RATE,
                            self.PARAM_LASER_RATE_DEFAULT))

        self.laserShovelFrame = rospy.get_param(
                                    self.PARAM_LASER_SHOVEL_FRAME,
                                    self.PARAM_LASER_SHOVEL_FRAME_DEFAULT)
        self.laserGroundLeftFrame = rospy.get_param(
                                        self.PARAM_LASER_GROUND_LEFT_FRAME,
                                        self.PARAM_LASER_GROUND_LEFT_FRAME_DEFAULT)
        self.laserGroundRightFrame = rospy.get_param(
                                        self.PARAM_LASER_GROUND_RIGHT_FRAME,
                                        self.PARAM_LASER_GROUND_RIGHT_FRAME_DEFAULT)
        self.laserSRDFrontFrame = rospy.get_param(
                                        self.PARAM_LASER_SRD_FRONT_FRAME,
                                        self.PARAM_LASER_SRD_FRONT_FRAME_DEFAULT)
        self.laserSRDLeftFrame = rospy.get_param(
                                        self.PARAM_LASER_SRD_LEFT_FRAME,
                                        self.PARAM_LASER_SRD_LEFT_FRAME_DEFAULT)
        self.laserSRDRightFrame = rospy.get_param(
                                        self.PARAM_LASER_SRD_RIGHT_FRAME,
                                        self.PARAM_LASER_SRD_RIGHT_FRAME_DEFAULT)

        self.isShovel = False
        self.isGroundLeft = False
        self.isGroundRight = False
        self.isSRDFront = False
        self.isSRDLeft = False
        self.isSRDRight = False

        self.pcShovelPublisher = None
        self.pcGroundLeftPublisher = None
        self.pcGroundRightPublisher = None
        self.pcSRDFrontPublisher = None
        self.pcSRDLeftPublisher = None
        self.pcSRDRightPublisher = None
        self.laserShovelPublisher = None
        self.laserGroundLeftPublisher = None
        self.laserGroundRightPublisher = None
        self.laserSRDFrontPublisher = None
        self.laserSRDLeftPublisher = None
        self.laserSRDRightPublisher = None
        self.shovelPC = None
        self.groundLeftPC = None
        self.groundRightPC = None
        self.srdFrontPC = None
        self.srdLeftPC = None
        self.srdRightPC = None
        self.shovelScan = None
        self.groundLeftScan = None
        self.groundRightScan = None
        self.srdFrontScan = None
        self.srdLeftScan = None
        self.srdRightScan = None

        self.errorPub = rospy.Publisher("sIA_rt_error_msgs", String, queue_size=10)
        self.laserSRDFrontPublisher_test = rospy.Publisher("~/pepper_navigation/front", LaserScan, queue_size=1)

    def setLaser(self, laser, state):
        if laser == "shovel":
            self.isShovel = state
        elif laser == "gl":
            self.isGroundLeft = state
        elif laser == "gr":
            self.isGroundRight = state
        elif laser == "srdf":
            self.isSRDFront = state
        elif laser == "srdl":
            self.isSRDLeft = state
        elif laser == "srdr":
            self.isSRDRight = state

    def setType(self, pointcloud=False, laserscan=True):
        self.pointcloud = pointcloud
        self.laserscan = laserscan

    def checkOn(self):
        return self.isShovel or self.isGroundLeft or self.isGroundRight or self.isSRDFront or self.isSRDLeft or self.isSRDRight

    def connectNaoQi(self, ip):
        self.laserProxy = ALProxy("ALLaser", ip, 9559)
        self.memProxy = ALProxy("ALMemory", ip, 9559)

        if self.laserProxy is None or self.memProxy is None:
            self.errorPub.publish("Error 0x02: Could not start either ALLaser or ALMemory Proxy")
            exit(1)

    def fetchLaserValues(self, keyPrefix, scanNum):
        ranges = []
        # traverse backwards
        for i in xrange(scanNum, 0, -1):
            keyX = keyPrefix + 'Seg' + '%02d' % (i,) + '/X/Sensor/Value'
            keyY = keyPrefix + 'Seg' + '%02d' % (i,) + '/Y/Sensor/Value'
            x = self.memProxy.getData(keyX)
            y = self.memProxy.getData(keyY)
            ranges.append(math.sqrt(math.pow(x, 2) + math.pow(y, 2)))

        return ranges

    def fetchPCValues(self, keyPrefix, scanNum):
        scans = []

        for i in xrange(scanNum, 0, -1):
            keyX = keyPrefix + 'Seg' + '%02d' % (i,) + '/X/Sensor/Value'
            keyY = keyPrefix + 'Seg' + '%02d' % (i,) + '/Y/Sensor/Value'
            x = self.memProxy.getData(keyX)
            y = self.memProxy.getData(keyY)
            scans.append(x)
            scans.append(y)
            scans.append(0.0)
        ba = struct.pack('%sf' % len(scans), *scans)

        return ba

    def createPointCloudMessage(self, frameID, keyPrefix, scanNum):
        pointCloudMsg = PointCloud2()
        pointCloudMsg.header.frame_id = frameID
        pointCloudMsg.header.stamp = rospy.Time.now()
        pointCloudMsg.height = 1
        pointCloudMsg.width = scanNum
        pointCloudMsg.is_dense = False
        pointCloudMsg.is_bigendian = False
        pointCloudMsg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1)]
        pointCloudMsg.point_step = 4 * 3
        pointCloudMsg.row_step = pointCloudMsg.point_step * pointCloudMsg.width
        pointCloudMsg.data = self.fetchLaserValues(keyPrefix, scanNum)

        return pointCloudMsg

    def createLaserMessage(self, frameID, keyPrefix, scanNum):
        laserScanMsg = LaserScan()
        laserScanMsg.header.frame_id = frameID
        laserScanMsg.angle_min = self.PEPPER_LASER_MIN_ANGLE
        laserScanMsg.angle_max = self.PEPPER_LASER_MAX_ANGLE
        laserScanMsg.angle_increment = self.PEPPER_LASER_FOV / scanNum
        laserScanMsg.range_min = self.PEPPER_LASER_MIN_RANGE
        laserScanMsg.range_max = self.PEPPER_LASER_MAX_RANGE

        return laserScanMsg

    def createPublishers(self):
        self.pcShovelPublisher = rospy.Publisher(self.TOPIC_LASER_SHOVEL + 'pointCloud', PointCloud2, queue_size=1)
        self.pcGroundLeftPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_LEFT + 'pointCloud', PointCloud2,
                                                     queue_size=1)
        self.pcGroundRightPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_RIGHT + 'pointCloud', PointCloud2,
                                                      queue_size=1)
        self.pcSRDFrontPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_FRONT + 'pointCloud', PointCloud2,
                                                   queue_size=1)
        self.pcSRDLeftPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_LEFT + 'pointCloud', PointCloud2,
                                                  queue_size=1)
        self.pcSRDRightPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_RIGHT + 'pointCloud', PointCloud2,
                                                   queue_size=1)

        self.laserShovelPublisher = rospy.Publisher(self.TOPIC_LASER_SHOVEL + 'laserScan', LaserScan, queue_size=1)
        self.laserGroundLeftPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_LEFT + 'laserScan', LaserScan,
                                                        queue_size=1)
        self.laserGroundRightPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_RIGHT + 'laserScan', LaserScan,
                                                         queue_size=1)
        self.laserSRDFrontPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_FRONT + 'laserScan', LaserScan, queue_size=1)
        self.laserSRDLeftPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_LEFT + 'laserScan', LaserScan, queue_size=1)
        self.laserSRDRightPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_RIGHT + 'laserScan', LaserScan, queue_size=1)

    def createMessages(self):
        self.shovelPC = self.createPointCloudMessage(
            self.laserShovelFrame,
            self.PEPPER_MEM_KEY_GROUND_SHOVEL,
            self.PEPPER_LASER_GROUND_SHOVEL_POINTS)
        self.groundLeftPC = self.createPointCloudMessage(
            self.laserGroundLeftFrame,
            self.PEPPER_MEM_KEY_GROUND_LEFT,
            self.PEPPER_LASER_GROUND_LEFT_POINTS)
        self.groundRightPC = self.createPointCloudMessage(
            self.laserGroundRightFrame,
            self.PEPPER_MEM_KEY_GROUND_RIGHT,
            self.PEPPER_LASER_GROUND_RIGHT_POINTS)
        self.srdFrontPC = self.createPointCloudMessage(
            self.laserSRDFrontFrame,
            self.PEPPER_MEM_KEY_SRD_FRONT,
            self.PEPPER_LASER_SRD_POINTS)
        self.srdLeftPC = self.createPointCloudMessage(
            self.laserSRDLeftFrame,
            self.PEPPER_MEM_KEY_SRD_LEFT,
            self.PEPPER_LASER_SRD_POINTS)
        self.srdRightPC = self.createPointCloudMessage(
            self.laserSRDRightFrame,
            self.PEPPER_MEM_KEY_SRD_RIGHT,
            self.PEPPER_LASER_SRD_POINTS)
        self.shovelScan = self.createLaserMessage(
            self.laserShovelFrame,
            self.PEPPER_MEM_KEY_GROUND_SHOVEL,
            self.PEPPER_LASER_GROUND_SHOVEL_POINTS)
        self.groundLeftScan = self.createLaserMessage(
            self.laserGroundLeftFrame,
            self.PEPPER_MEM_KEY_GROUND_LEFT,
            self.PEPPER_LASER_GROUND_LEFT_POINTS)
        self.groundRightScan = self.createLaserMessage(
            self.laserGroundRightFrame,
            self.PEPPER_MEM_KEY_GROUND_RIGHT,
            self.PEPPER_LASER_GROUND_RIGHT_POINTS)
        self.srdFrontScan = self.createLaserMessage(
            self.laserSRDFrontFrame,
            self.PEPPER_MEM_KEY_SRD_FRONT,
            self.PEPPER_LASER_SRD_POINTS)
        self.srdLeftScan = self.createLaserMessage(
            self.laserSRDLeftFrame,
            self.PEPPER_MEM_KEY_SRD_LEFT,
            self.PEPPER_LASER_SRD_POINTS)
        self.srdRightScan = self.createLaserMessage(
            self.laserSRDRightFrame,
            self.PEPPER_MEM_KEY_SRD_RIGHT,
            self.PEPPER_LASER_SRD_POINTS)

    def getLaserData(self):

        if self.laserscan:
            # fetch values

            if self.isShovel:
                self.shovelScan.header.stamp = rospy.Time.now()
                self.shovelScan.ranges = self.fetchLaserValues(
                    self.PEPPER_MEM_KEY_GROUND_SHOVEL,
                    self.PEPPER_LASER_GROUND_SHOVEL_POINTS
                )
                self.laserShovelPublisher.publish(self.shovelScan)

            if self.isGroundLeft:
                self.groundLeftScan.header.stamp = rospy.Time.now()
                self.groundLeftScan.ranges = self.fetchLaserValues(
                    self.PEPPER_MEM_KEY_GROUND_LEFT,
                    self.PEPPER_LASER_GROUND_LEFT_POINTS
                )
                self.laserGroundLeftPublisher.publish(self.groundLeftScan)

            if self.isGroundRight:
                self.groundRightScan.header.stamp = rospy.Time.now()
                self.groundRightScan.ranges = self.fetchLaserValues(
                    self.PEPPER_MEM_KEY_GROUND_RIGHT,
                    self.PEPPER_LASER_GROUND_RIGHT_POINTS
                )
                self.laserGroundRightPublisher.publish(self.groundRightScan)

            if self.isSRDFront:
                self.srdFrontScan.header.stamp = rospy.Time.now()
                self.srdFrontScan.ranges = self.fetchLaserValues(
                    self.PEPPER_MEM_KEY_SRD_FRONT,
                    self.PEPPER_LASER_SRD_POINTS
                )
                self.laserSRDFrontPublisher.publish(self.srdFrontScan)

            if self.isSRDLeft:
                self.srdLeftScan.header.stamp = rospy.Time.now()
                self.srdLeftScan.ranges = self.fetchLaserValues(
                    self.PEPPER_MEM_KEY_SRD_LEFT,
                    self.PEPPER_LASER_SRD_POINTS
                )
                self.laserSRDLeftPublisher.publish(self.srdLeftScan)

            if self.isSRDRight:
                self.srdRightScan.header.stamp = rospy.Time.now()
                self.srdRightScan.ranges = self.fetchLaserValues(
                    self.PEPPER_MEM_KEY_SRD_RIGHT,
                    self.PEPPER_LASER_SRD_POINTS
                )
                self.laserSRDRightPublisher.publish(self.srdRightScan)

        if self.pointcloud:
            # fetch values

            if self.isShovel:
                self.shovelPC.header.stamp = rospy.Time.now()
                self.shovelPC.data = self.fetchPCValues(
                    self.PEPPER_MEM_KEY_GROUND_SHOVEL,
                    self.PEPPER_LASER_GROUND_SHOVEL_POINTS
                )
                self.pcShovelPublisher.publish(self.shovelPC)

            if self.isGroundLeft:
                self.groundLeftPC.header.stamp = rospy.Time.now()
                self.groundLeftPC.data = self.fetchPCValues(
                    self.PEPPER_MEM_KEY_GROUND_LEFT,
                    self.PEPPER_LASER_GROUND_LEFT_POINTS
                )
                self.pcGroundLeftPublisher.publish(self.groundLeftPC)

            if self.isGroundRight:
                self.groundRightPC.header.stamp = rospy.Time.now()
                self.groundRightPC.data = self.fetchPCValues(
                    self.PEPPER_MEM_KEY_GROUND_RIGHT,
                    self.PEPPER_LASER_GROUND_RIGHT_POINTS
                )
                self.pcGroundRightPublisher.publish(self.groundRightPC)

            if self.isSRDFront:
                self.srdFrontPC.header.stamp = rospy.Time.now()
                self.srdFrontPC.data = self.fetchPCValues(
                    self.PEPPER_MEM_KEY_SRD_FRONT,
                    self.PEPPER_LASER_SRD_POINTS
                )
                self.pcSRDFrontPublisher.publish(self.srdFrontPC)

            if self.isSRDLeft:
                self.srdLeftPC.header.stamp = rospy.Time.now()
                self.srdLeftPC.data = self.fetchPCValues(
                    self.PEPPER_MEM_KEY_SRD_LEFT,
                    self.PEPPER_LASER_SRD_POINTS
                )
                self.pcSRDLeftPublisher.publish(self.srdLeftPC)

            if self.isSRDRight:
                self.srdRightPC.header.stamp = rospy.Time.now()
                self.srdRightPC.data = self.fetchPCValues(
                    self.PEPPER_MEM_KEY_SRD_RIGHT,
                    self.PEPPER_LASER_SRD_POINTS
                )
                self.pcSRDRightPublisher.publish(self.srdRightPC)

            # sleep
        self.laserRate.sleep()
