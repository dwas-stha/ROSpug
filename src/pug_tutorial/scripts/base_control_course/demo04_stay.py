#!/usr/bin/env python3
# coding=utf8
import time
import rospy
from pug_control.msg import Velocity, Pose, Gait

rospy.init_node('stay', log_level=rospy.INFO)

pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)

time.sleep(0.1)

pose_pub.publish(0, 0, 0, -0.13, 0.01, 0, 0, 0.5)
time.sleep(0.5)

gait_pub.publish(0.2, 0.18, 0.02, 0.04)

velocity_pub.publish(0, 0, 0, False)
time.sleep(3)
velocity_pub.publish(0, 0, 0, True)
