#! /usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rospy.init_node('bigWins')
scoreTracker = rospy.Publisher('/score_tracker', String, queue_size=1)
control = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)

startMessage = str("BestTeam,password,0,clue")
stopMessage = str("BestTeam,password,-1,clue")

time.sleep(1)

scoreTracker.publish(startMessage)
move = Twist()
move.linear.x = 1

control.publish(move)
time.sleep(5)

move.linear.x = 0
control.publish(move)
scoreTracker.publish(stopMessage)
time.sleep(5)

rospy.signal_shutdown("we remain winning")
