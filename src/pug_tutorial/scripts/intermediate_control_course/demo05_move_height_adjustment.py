#!/usr/bin/env python3
# coding=utf8
import time
import rospy
from pug_control.msg import Velocity, Pose, Gait

rospy.init_node('move_height', log_level=rospy.INFO)

pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)

time.sleep(0.1)

pose_pub.publish(0, 0, 0, -0.13, 0.01, 0, 0, 0.5)
time.sleep(0.5)

gait_pub.publish(0.2, 0.18, 0.02, 0.04)

velocity_pub.publish(0.06, 0, 0, False)

for i in range(30):
    pose_pub.publish(0, 0, 0, -0.15 + i*0.001, 0, 0, 0, 0)
    time.sleep(0.1)

velocity_pub.publish(0, 0, 0, True)
