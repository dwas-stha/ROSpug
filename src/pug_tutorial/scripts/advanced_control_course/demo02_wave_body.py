#!/usr/bin/env python3
# coding=utf8
import time
import math
import rospy
from pug_control.msg import Pose

rospy.init_node('wave_body', log_level=rospy.INFO)

pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)

time.sleep(0.1)

pose_pub.publish(0, 0, 0, -0.13, 0, 0, 0, 0.5)
time.sleep(0.5)

for i in range(3):
    pose_pub.publish(0, math.radians(-15), 0, -0.13, 0, 0, 0, 0.5)
    time.sleep(0.5)
    pose_pub.publish(0, math.radians(0), 0, -0.13, 0, 0, 0, 0.5)
    time.sleep(0.5)

    pose_pub.publish(0, math.radians(15), 0, -0.13, 0, 0, 0, 0.5)
    time.sleep(0.5)
    pose_pub.publish(0, math.radians(0), 0, -0.13, 0, 0, 0, 0.5)
    time.sleep(0.5)
