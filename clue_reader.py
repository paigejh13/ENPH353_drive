#! /usr/bin/env python3

import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tensorflow.keras import models
import time
from rosgraph_msgs.msg import Clock

SAMPLE_RATE = 3

frameCounter = 1
currentMaxBound = 0
currentBestImage = (0,0)
storedVal = False
clueCounter = 1
clueSet = set()

model = models.load_model('/home/fizzer/ros_ws/src/my_controller/src/model2.h5', compile=False)
model.compile()
startMessage = str("BestTeam,password,0,clue")
stopMessage = str("BestTeam,password,-1,clue")

print("started")

def imageCallback(data):
    global frameCounter, currentBestImage, currentMaxBound, storedVal, clueCounter
    if frameCounter < SAMPLE_RATE:
        frameCounter += 1
    else:
        #clocktime = rospy.waitForMessage('/clock', Clock)
        
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

        if maxBound > 500 and hierarchy[0,maxind,2] != -1 and maxBound > currentMaxBound:
            poly = cv2.approxPolyDP(contours[hierarchy[0,maxind,2]], 0.01 * maxBound, True)
            if len(poly) == 4:
                storedVal = True
                currentMaxBound = maxBound
                currentBestImage = (imageHSV, poly)
                #cv2.imshow("best",currentBestImage[0])
                #cv2.waitKey(5)

        elif maxBound < 250 and storedVal:
            currentMaxBound = 0
            storedVal = False
            result, clueNum = doAnalysis(currentBestImage[0],currentBestImage[1])
            submitClue(clueNum, result)



def doAnalysis(image, poly):
    corners = createCornerArray(poly)
    poly = np.array(poly, dtype='float32')
    matrix = cv2.getPerspectiveTransform(poly, corners)
    final = cv2.warpPerspective(image, matrix, (300,200))
    img_gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
    if img_gray[130,280] < 160:
        img_gray = ~img_gray

    avr = np.average(img_gray)

    binImage = cv2.inRange(img_gray, 0, avr*0.75)
    #cv2.imshow("test",img_gray)
    #cv2.waitKey(5)

    letters = splitLetters(binImage)

    word = useModel(letters)
    

    clueNum = findClueNum(binImage)
    return "".join(word), clueNum

def findClueNum(image):
    ref = {28:1,31:2,12:3,29:4,25:5,22:6,32:7,11:8}
    letter = image[20:64,120:148]
    pred = model.predict(np.expand_dims(letter, axis=0))[0]
    index = np.argmax(pred)
    print(index)
    clueNum = ref.get(index)
    if clueNum == None:
        if clueCounter < 4:
            clueNum = 3
        else:
            clueNum = 4

    return clueNum



def submitClue(clueNum, value):
    global clueCounter
    if value not in clueSet:
        clueSet.add(value)
        message = str("BestTeam,password,"+str(clueNum) + ',' + str(value))
        scoreTracker.publish(message)
        if clueNum == 8:
            time.sleep(1)
            scoreTracker.publish(stopMessage)
        clueCounter += 1
    

def useModel(letters):
    global model
    ref = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    out = list()
    for l in letters:
        pred = model.predict(np.expand_dims(l, axis=0))[0]
        out.append(ref[np.argmax(pred)])
    return out


def splitLetters(clue):
    letters = list()
    for i in range(12):
        a = clue[128:172, (22*i + (i//4) + 14) : (22*(i+1) + 20 + (i//4))]
        if np.average(a) > 20:
            letters.append(a)
        b = str(i)
        #cv2.imshow(b,a)
    return letters

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
        out[i,1] = 200 * bottom

    return np.array(out, dtype='float32')




rospy.init_node('ClueTracker')
scoreTracker = rospy.Publisher('/score_tracker', String, queue_size=1)
control = rospy.Subscriber('/R1/pi_camera/image_raw', Image, imageCallback)



time.sleep(1)
scoreTracker.publish(startMessage)

rospy.spin()