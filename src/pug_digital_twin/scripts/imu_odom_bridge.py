#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ImuOdomBridge(object):
    def __init__(self):
        self.odom_pub = rospy.Publisher(
            "/digital_twin/odom",
            Odometry,
            queue_size=10
        )

        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_footprint")

        self.x = rospy.get_param("~x", 0.0)
        self.y = rospy.get_param("~y", 0.0)
        self.z = rospy.get_param("~z", 0.0)

        self.roll_sign = rospy.get_param("~roll_sign", 1.0)
        self.pitch_sign = rospy.get_param("~pitch_sign", 1.0)
        self.yaw_sign = rospy.get_param("~yaw_sign", 1.0)

        imu_topic = rospy.get_param("~imu_topic", "/imu")

        rospy.Subscriber(imu_topic, Imu, self.imu_cb, queue_size=10)

        rospy.loginfo("Publishing /digital_twin/odom from %s", imu_topic)

    def imu_cb(self, msg):
        q = msg.orientation

        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        roll *= self.roll_sign
        pitch *= self.pitch_sign
        yaw *= self.yaw_sign

        out_q = quaternion_from_euler(roll, pitch, yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.pose.pose.orientation.x = out_q[0]
        odom.pose.pose.orientation.y = out_q[1]
        odom.pose.pose.orientation.z = out_q[2]
        odom.pose.pose.orientation.w = out_q[3]

        self.odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("imu_odom_bridge")
    ImuOdomBridge()
    rospy.spin()
