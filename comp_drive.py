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
speed = 0.3
cross_walk_passed = False
fork_passed = False
time_in_crosswalk = 0
in_crosswalk = False
turn_complete = False 

def check_crosswalk(im): 
  hsv1 = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv1, (120, 30, 0), (140, 255, 255))
  sum = np.sum([mask[700:720, 635:645]])
  if sum > 200:
    return True
  else:
    return False

def tunnel_detect(im):
  hsv  = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv, (110, 0, 100), (120, 255,255))
  return mask


  pinkLineFound = False
  sum = np.sum([mask[300:600, 0:1280]])
  if sum > 255*1000:
      pinkLineFound = True
  return pinkLineFound, mask
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
  global cross_walk_passed, fork_passed, speed, in_crosswalk, time_in_crosswalk

  if cross_walk_passed == False:
    if check_crosswalk(im) == True:
      in_crosswalk = True
      speed = 1
    elif in_crosswalk == True and time_in_crosswalk > 10:
      in_crosswalk = False
      cross_walk_passed = True
      speed = 0.3
    elif in_crosswalk == True:
      time_in_crosswalk += 1
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
  return avgLeft, avgRight, speed

def off_road(im):
  global turn_complete
  if turn_complete == False:
    target = 200

  mask = tunnel_detect(im)
  locations = np.argwhere(mask > 250)
  if len(locations) > 10000:
    avg_location = np.average(locations[:,1])
    target = avg_location+250
    turn_complete = True
  elif turn_complete == True: 
    target = 640

  
  return target+100, target-100

def image_process(cv_image, pinkLine, state, speed):
  global turn_complete
  rotated = False
  hsv1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv1, (140, 25, 25), (160, 255,255))

  pinkLineFound = False
  sum = np.sum([mask[620:630, 635:645]])
  if sum > 200:
      pinkLineFound = True
        
  if pinkLine == False and pinkLineFound == True:
    pinkLine = True
  elif pinkLine == True and pinkLineFound == False:
    pinkLine = False
    state += 1

  if state == 4 and turn_complete == False:
    state = 3
  if state == 1:
    left, right, speed = pavement(cv_image)
  elif state == 3:
    left, right = off_road(cv_image)
    speed = 0.3
  elif state == 2:
    speed = 0.15
    left, right = grass(cv_image)
  else: 
    left, right = grass(cv_image) 
    if rotated == False and (right - left)/2 + left - 640 >1:
      left = 100
      right = 800
      speed = 0
    else:
      rotated = True
      speed = 0.15 
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