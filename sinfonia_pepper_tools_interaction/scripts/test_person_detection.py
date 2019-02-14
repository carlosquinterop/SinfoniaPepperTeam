#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    print(data.data)

def test_person_det():

    rospy.init_node('sIA_test_detect_person', anonymous=True)
    rospy.Subscriber("person_detection", String, callback)
    rospy.spin()

if __name__ == '__main__':
    test_person_det()
