#! /usr/bin/env python3

import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

SAMPLE_RATE = 3

frameCounter = 1

print("started")

def imageCallback(data):
    global frameCounter
    if frameCounter < SAMPLE_RATE:
        frameCounter += 1
    else:
        frameCounter = 1
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        imageHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        filtered = cv2.inRange(imageHSV, np.array([116,100,50]),np.array([125,255,255]))
        eroded = cv2.erode(filtered,np.ones((3,3),np.uint8))
        contours, hierarchy = cv2.findContours(eroded, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        maxBound = 0
        maxind = 0
        i = 0
        for contour in contours:
            if cv2.arcLength(contour, True) > maxBound:
                maxBound = cv2.arcLength(contour, True)
                maxind = i
            i += 1

        if maxBound > 500 and hierarchy[0,maxind,2] != -1:
            poly = cv2.approxPolyDP(contours[hierarchy[0,maxind,2]], 0.01 * maxBound, True)
            if len(poly) == 4:

                corners = createCornerArray(poly)
                poly = np.array(poly, dtype='float32')
                matrix = cv2.getPerspectiveTransform(poly, corners)
                final = cv2.warpPerspective(imageHSV, matrix, (300,150))
                img_gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
                if img_gray[130,280] < 160:
                    img_gray = ~img_gray

                letters = list()

                for i in range(12):
                    letters.append(img_gray[90:130, (22*i + (i//4) + 14) : (22*(i+1) + 20 + (i//4))])


                cv2.imshow("Test", letters[5])
                cv2.waitKey(5)

    


def createCornerArray(polygon):
    x = polygon[:,0,0]
    y = polygon[:,0,1]
    xav = np.average(x)
    yav = np.average(y)
    out = np.zeros((4,2))
    for i in range(4):
        right = (x[i] > xav)
        bottom = (y[i] > yav)
        out[i,0] = 300*right
        out[i,1] = 150 * bottom

    return np.array(out, dtype='float32')
        



rospy.init_node('ClueTracker')
control = rospy.Subscriber('/R1/pi_camera/image_raw', Image, imageCallback)

rospy.spin()