#! /usr/bin/env python3

import rospy
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

do_image = False
count = 0
name = "1"
print("started")

def take_image(a):
    global do_image, name
    print("a")
    name = a
    do_image = True
    print(name)


def imageCallback(data):
    global do_image, name, count
    if do_image:
        print("b")
        do_image = False
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        path = '/home/fizzer/ros_ws/src/my_controller/src/Photos/data.jpg'
        print(path)
        count += 1
        cv2.imwrite(path, cv_image)




rospy.init_node('bigWins')
scoreTracker = rospy.Subscriber('/score_tracker', String, take_image)
control = rospy.Subscriber('/R1/pi_camera/image_raw', Image, imageCallback)

rospy.spin()