#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import numpy as np
from ros_robot_controller.msg import SetJointAngle
from ros_robot_controller.srv import SetLegIK, SetLegFK

rospy.init_node('kinematics', log_level=rospy.INFO)


                            # FR     FL     BR     BL
foot_locations = np.array([[-0.01, -0.01, -0.01, -0.01],  # X
                           [ 0.0,   0.0,   0.0,   0.0],   # Y
                           [-0.1,  -0.1,  -0.1,  -0.1]    # Z
                          ])

set_leg_ik = rospy.ServiceProxy('/ros_robot_controller/robot/set_leg_ik', SetLegIK)
set_leg_fk = rospy.ServiceProxy('/ros_robot_controller/robot/set_leg_fk', SetLegFK)
set_angle_pub = rospy.Publisher('/ros_robot_controller/robot/set_joint_angle', SetJointAngle, queue_size=1)
time.sleep(0.1)

# 输入四个落脚点位置, 返回每个关节的角度，坐标是以大腿关节为原点, 要结合send_joint_angle去驱动, 输入为3x4的数组
res = set_leg_ik(foot_locations.reshape(12).tolist())
if res.success:
    joint_angle = res.joint_angle
    print('ik input:\n', foot_locations)
    print('ik output:\n', joint_angle)
    set_angle_pub.publish(joint_angle, 0.5)
    time.sleep(0.5)
    # # 输入每个关节的角度, 返回四个落脚点坐标，坐标是以大腿关节为原点，输入为3x4的数组, 不驱动只计算
    res = set_leg_fk(joint_angle)
    if res.success:
        print('fk output:\n', res.position)

print()

foot_locations[2][0] -= 0.05
foot_locations[2][1] -= 0.05
foot_locations[2][2] -= 0.05
foot_locations[2][3] -= 0.05

res = set_leg_ik(foot_locations.reshape(12).tolist())
if res.success:
    joint_angle = res.joint_angle
    print('ik input:\n', foot_locations)
    print('ik output:\n', joint_angle)
    set_angle_pub.publish(joint_angle, 0.5)
    time.sleep(0.5)
    res = set_leg_fk(joint_angle)
    if res.success:
        print('fk output:\n', res.position)

