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
from take_order.takeOrder import TakeOrder
from make_order.makeOrder import MakeOrder
from give_order.giveOrder import GiveOrder
from check_door.check_door import CheckDoor
from sinfonia_pepper_robot_toolkit.msg import MoveToVector, T2S, MoveTowardVector
from sinfonia_pepper_tools_interaction.srv import *


class ApplicationSchedulerNode:

    def __init__(self):
        rospy.init_node('application_scheduler_node', anonymous=True)
        self._rate = rospy.Rate(10)
        self._state = "offer_service" #"wake_up"

        self.checkDoor = None
        self.initPublishers()

        self.stateMsg = T2S()
        self.stateMsg.language = "English"

        self.moveToPub = None
        self.stopPub = None
        self.T2Spub = None
        self.moveTowardPub = None
        self.setPosturePub = None
        self.setSecurityPub = None
        self.setAwarenessPub = None

        self._messagesES = {"wake_up": "",
                            "check_door": "Toc Toc, Hola, abreme la puerta porfavor",
                            "enter": "Gracias",
                            "navigation_go_to_living_room": "Voy hacia la sala",
                            "offer_service": "Si alguien quiere una bebida, porfavor acerquese.",
                            "take_order": "",
                            "navigation_go_to_bar": "Me dirijo al Bar",
                            "make_order": "",
                            "navigation_return_to_living_room": "",
                            "give_order": "",
                            "idle": "En este momento no se que hacer",
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
        self.setSecurityPub = rospy.Publisher("sIA_set_security", String, queue_size=10)
        while self.setSecurityPub.get_num_connections() == 0:
            self._rate.sleep()
        self.setAwarenessPub = rospy.Publisher("sIA_set_awareness", String, queue_size=10)
        while self.setAwarenessPub.get_num_connections() == 0:
            self._rate.sleep()



    def speak(self, text):
        rospy.wait_for_service('srvSpeak')
        try:
            sendText = rospy.ServiceProxy('srvSpeak', speak)
            sendText(text)
        except:
            pass

    def appicationSchedulerNode(self):

        while not rospy.is_shutdown():
            print(self._state)
            self.speak(self._messagesES[self._state])
            # self.stateMsg.text = self._messages[self._state]
            # self.T2Spub.publish(self.stateMsg)

            if self._state == "wake_up":
                self.setPosturePub.publish("StandInit")
                time.sleep(11)
                self._state = "check_door"

            elif self._state == "check_door":
                self.checkDoor = CheckDoor()
                self.checkDoor.initPublishers()
                self.checkDoor.subscribeTopics()
                self.checkDoor.startSonar()
                while not self.checkDoor.checkDoor():
                    continue
                self.checkDoor.stopSonar()
                self._state = "enter"

            elif self._state == "enter":
                self.setSecurityPub.publish("OFF")
                msg = utils.fillVector([1.0, 0.0, 0.0, 3.0], "mt")
                self.moveToPub.publish(msg)
                time.sleep(12)
                self._state = "navigation_go_to_living_room"
                self.setSecurityPub.publish("ON")

            elif self._state == "navigation_go_to_living_room":
                msg = utils.fillVector([2.0, 0.0, 0.0, 3.0], "mt")
                self.moveToPub.publish(msg)
                time.sleep(12)
                ### IR A la SALA !!!!!!!!!!!!!!!!!!!!!!!
                self._state = "offer_service"

            elif self._state == "offer_service":
                self.setPosturePub.publish("SayHi")
                time.sleep(10)
                self._state = "take_order"

            elif self._state == "take_order":
                self.setAwarenessPub.publish("ON")
                takeOrder = TakeOrder()
                takeOrder.start()
                self.setAwarenessPub.publish("OFF")
                self._state = "navigation_go_to_bar"

            elif self._state == "navigation_go_to_bar":
                self.setSecurityPub.publish("OFF")
                msg = utils.fillVector([0.0, 0.0, 3.1415, 3.0], "mt")
                self.moveToPub.publish(msg)
                time.sleep(4)
                self.setSecurityPub.publish("ON")

                ### IR AL BAR !!!!!!!!!!!!!!!!!!!!!!!
                self._state = "make_order"

            elif self._state == "make_order":
                makeOrder = MakeOrder()
                makeOrder.start()
                self._state = "navigation_return_to_living_room"

            elif self._state == "navigation_return_to_living_room":
                self.setSecurityPub.publish("OFF")
                msg = utils.fillVector([0.0, 0.0, 3.1415, 3.0], "mt")
                self.moveToPub.publish(msg)
                time.sleep(4)
                self.setSecurityPub.publish("ON")

                ### IR AL BAR !!!!!!!!!!!!!!!!!!!!!!!
                self._state = "give_order"

            elif self._state == "give_order":
                self.setAwarenessPub.publish("ON")
                giveOrder = GiveOrder()
                giveOrder.start()
                self.setAwarenessPub.publish("OFF")
                self._state = "navigation_go_to_bar"

            else:
                self.stopPub.publish("Stop")
                self.setPosturePub.publish("Crouch")
                time.sleep(10)
                break


            self._rate.sleep()


if __name__ == '__main__':

    node = ApplicationSchedulerNode()
    node.appicationSchedulerNode()