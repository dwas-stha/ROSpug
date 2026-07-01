#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R

class SelfBalancingNode:
    def __init__(self):
        rospy.init_node("self_balancing_node")
        rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size=2)

    def imu_callback(self, imu_msg):
        try:
            q = imu_msg.orientation
            r = R.from_quat((q.x, q.y, q.z, q.w))
            x, y, z = r.as_euler('xyz', degrees=True)
            rospy.loginfo("Pitch:{:>6.2f}°, Roll:{:>6.2f}°, Yaw:{:>6.2f}°".format(x, y, z))
        except Exception as e:
            rospy.logerr(str(e))

if __name__ == "__main__":
    try:
        SelfBalancingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
