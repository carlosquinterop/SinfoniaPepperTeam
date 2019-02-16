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
from std_msgs.msg import String
from check_door.check_door import CheckDoor
from sinfonia_pepper_robot_toolkit.msg import MoveToVector, T2S, MoveTowardVector


class ApplicationSchedulerNode:

    def __init__(self):
        rospy.init_node('application_scheduler_node', anonymous=True)
        self._rate = rospy.Rate(10)
        self._state = "wake_up"

        self.checkDoor = None
        self.initPublishers()

        self.stateMsg = T2S()
        self.stateMsg.language = "English"

        self.moveToPub = None
        self.stopPub = None
        self.T2Spub = None
        self.moveTowardPub = None
        self.setPosturePub = None

        self._messages = {"wake_up": "",
                          "check_door": "Toc Toc, Hello can you please open the door",
                          "go_to_room": "Thank you, I am going to the party",
                          "idle": "at this moment I dont know what to do",
                          "sleep": ""
                          }
        self.initPublishers()

    def initPublishers(self):
        self.moveToPub = rospy.Publisher("sIA_move_to", MoveToVector, queue_size=10)
        while self.moveToPub.get_num_connections() == 0:
            self._rate.sleep()
        self.stopPub = rospy.Publisher("sIA_stop_move", String, queue_size=10)
        while self.stopPub.get_num_connections() == 0:
            self._rate.sleep()
        self.T2Spub = rospy.Publisher("sIA_say_something", T2S, queue_size=1)
        while self.T2Spub.get_num_connections() == 0:
            self._rate.sleep()
        self.moveTowardPub = rospy.Publisher("sIA_move_toward", MoveTowardVector, queue_size=10)
        while self.moveTowardPub.get_num_connections() == 0:
            self._rate.sleep()
        self.setPosturePub = rospy.Publisher("sIA_set_posture", String, queue_size=10)
        while self.setPosturePub.get_num_connections() == 0:
            self._rate.sleep()

    def appicationSchedulerNode(self):
        while not rospy.is_shutdown():
            print(self._state)

            self.stateMsg.text = self._messages[self._state]
            self.T2Spub.publish(self.stateMsg)

            if self._state == "wake_up":
                self.setPosturePub.publish("StandInit")
                time.sleep(14)
                self._state = "check_door"

            elif self._state == "check_door":
                self.checkDoor = CheckDoor()
                self.checkDoor.initPublishers()
                self.checkDoor.subscribeTopics()
                self.checkDoor.startSonar()
                while not self.checkDoor.checkDoor():
                    continue
                self.checkDoor.stopSonar()
                self._state = "go_to_room"
                # time.sleep(5)

            elif self._state == "go_to_room":
                msg = utils.fillVector([1.0, 0.0, 0.0, 5.0], "mt")
                self.moveToPub.publish(msg)
                time.sleep(5)
                self._state = "sleep"
            else:
                self.stopPub.publish("Stop")
                self.setPosturePub.publish("Crouch")
                time.sleep(10)
                break


            self._rate.sleep()


if __name__ == '__main__':

    node = ApplicationSchedulerNode()
    node.appicationSchedulerNode()