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
import matplotlib.pyplot as plt

left = 0
right = 1280
pinkLine = False
state = 1
speed = 0.25
cross_walk_passed = False
fork_passed = False

def check_crosswalk(im):
  return False

def check_pedestrian(im):
  return False

def check_fork(im):
  return False

def grass(im):
  avg_bright = np.sum([im[600:620, 600:680, 0:3]])/(20*80)
  hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
  lower_blue = np.array([60, 35, 140]) 
  upper_blue = np.array([180, 255, 255])  
  mask = cv2.inRange(hsv, lower_blue, upper_blue)
  im = cv2.bitwise_and(im, im, mask = mask) 

  im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

  threshold = 0.49812*avg_bright-45.
  _, im = cv2.threshold(im, threshold, 255, cv2.THRESH_BINARY)

  cv2.imshow("image", im)
  cv2.waitKey(2)

  Llist = list()
  Rlist = list()

  for j in range(600, 620):
    for i in range(0, 640):
      if im[j, i] > 10:
        Llist.append(i)
    for i in range(641, 1280):
        if im[j, i] > 10:
          Rlist.append(i)
          
  avgL = 0
  avgR = 0
  for i in Llist:
    avgL+=i
  for i in Rlist:
    avgR += i
  if len(Llist) > 150:
     avgLeft=avgL/len(Llist)
  else:
    avgLeft = 120
  if len(Rlist) > 150:
    avgRight = avgR/len(Rlist)
  else:
    avgRight = 1160
  return avgLeft, avgRight

def pavement(im):
  global cross_walk_passed, fork_passed
  if cross_walk_passed == False:
    if check_crosswalk(im) == True:
      while check_pedestrian(im) == True:
        move = Twist()
        control.publish(move)
      move = Twist()
      move.linear.x = 1
      control.publish(move)
  elif fork_passed == False:
    if check_fork(im) == True:
      for i in range(15000):
        move = Twist()
        move.linear.x = 0.1
        move.angular.z = 0.2
        control.publish(move)

  im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

  threshold = 250
  _, im = cv2.threshold(im, threshold, 255, cv2.THRESH_BINARY)

  Llist = list()
  Rlist = list()

  for j in range(600, 620):
    for i in range(0, 640):
      if im[j, i] > 10:
        Llist.append(i)
    for i in range(641, 1280):
      if im[j, i] > 10:
        Rlist.append(i)
          
  avgL = 0
  avgR = 0
  for i in Llist:
    avgL+=i
  for i in Rlist:
    avgR += i
  if len(Llist) > 0:
     avgLeft=avgL/len(Llist)
  else:
    avgLeft = 0
  if len(Rlist) > 0:
    avgRight = avgR/len(Rlist)
  else:
    avgRight = 1280
  return avgLeft, avgRight

def off_road(im):
  for i in range(365000):
    move = Twist()
    move.linear.x = 0.1
    move.angular.z=1
    control.publish(move)
  print("turn 1")
  for i in range(180000):
    move = Twist()
    move.linear.x = 1
    move.angular.z= 0
    control.publish(move)
  print("drive")
  for i in range(15000):
    move = Twist()
    move.linear.x = 0
    move.angular.z=0
    control.publish(move)
  print("turn 2")
  
  return 120,1160

def image_process(cv_image, pinkLine, state, speed):
  hsv1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv1, (140, 25, 25), (160, 255,255))
  pink = cv2.bitwise_and(cv_image,cv_image, mask= mask)

  pinkLineFound = False
  sum = np.sum([pink[620:630, 635:645, 0:2]])
  if sum > 200:
      pinkLineFound = True
        
  if pinkLine == False and pinkLineFound == True:
    pinkLine = True
    print("pinkLine found")
  elif pinkLine == True and pinkLineFound == False:
    pinkLine = False
    state += 1
    print("state", state)
  
  if state == 1:
    left, right = grass(cv_image)
    speed = 0.30
  elif state == 3:
    left, right = grass(cv_image)
    speed = 0.4
  else:
    speed = 0.15
    left, right = grass(cv_image)
  return left, right, pinkLine, state, speed
    

def Callback(msg):
    global left, right, pinkLine, state, speed
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg)
    left, right, pinkLine, state, speed = image_process(cv_image, pinkLine, state, speed)
    diff = (right - left)/2 + left - 640
    move = Twist()
    move.linear.x = speed
    move.angular.z= -diff/95
    #print(left, right, diff, move.angular.z)
    control.publish(move)

rospy.init_node('biggerWins')
scoreTracker = rospy.Publisher('/score_tracker', String, queue_size=1)
control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
camera = rospy.Subscriber('/R1/pi_camera/image_raw', Image, Callback)

rate = rospy.Rate(2)
rospy.spin()

def record():
  if sum > 200:
      pinkLineFound = True
  ymax, xmax, channels = im.shape
  total_red = 0
  total_blue = 0
  total_green = 0 
  count = 0

  for i in range(600, 680):
    for j in range(600,620):
      r, g, b = im[j, i]
      total_red += r
      total_green += g
      total_blue += b
      count += 1
  avg_bright = (total_red+total_green+total_blue)/count
  return