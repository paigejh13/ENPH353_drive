#! /usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

dest = 0

def image_process(frame, xVal):
  frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

  #set all pixels that make up the path to 0
  threshold = 200
  _, frame2 = cv2.threshold(frame1, threshold, 255, cv2.THRESH_BINARY)

  #blur the frame
  #img_blur1 = cv2.GaussianBlur(frame2,(15,15),0)

  #take the difference of the blurred and non blurred image to filter noise
  #and identify edges of the path
  #img_delta10 = img_blur1.astype(np.float32) - frame2.astype(np.float32)
  #img_delta10 =  cv2.normalize(img_delta10, img_delta10, 0, 1, cv2.NORM_MINMAX)
  img_delta10 = frame2


  #identify pixel locations of the path's edge
  edgeList = list()
  edgeLoc = np.zeros(2)
  k = 0
  total = 0

  for i in range(1, img_delta10.shape[1]):
    if abs(img_delta10[220,i] - img_delta10[220,i-1]) > 0.32:
      edgeList.append(i)
      k = k+1

  for i in range(0, len(edgeList)):
    total = total + edgeList[i]

  #centre the circle on the average of path's edge pixel locations
  if k == 1:
    xVal = total/2
  elif k != 0:
    xVal = total/k
      
  return xVal


def Callback(msg):
    global dest
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg)
    dest = image_process(cv_image, dest)
    #diff = dest-320/2
    #move = Twist()
    #move.linear.x = 0.3
    #move.angular.z= -0.05*diff
    #print(move)
    #pub.publish(move)

rospy.init_node('biggerWins')
scoreTracker = rospy.Publisher('/score_tracker', String, queue_size=1)
control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
camera = rospy.Subscriber('/R1/pi_camera/image_raw', Image, Callback)

rate = rospy.Rate(2)
rospy.spin()