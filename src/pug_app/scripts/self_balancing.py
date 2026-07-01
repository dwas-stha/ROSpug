#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
from sensor_msgs.msg import Imu
from pug_sdk import pid
from pug_control.msg import Pose
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R
from ros_robot_controller.msg import BuzzerState

class SelfBalancingNode:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name

        self.pid_pitch = pid.PID(0.12, 0.0, 0.0002)
        self.pid_roll = pid.PID(0.15, 0.0, 0.0002)
        self.pitch = 0
        self.roll = 0
        self.init_finish = False
        self.pitch_init = 0
        self.roll_init = 0
        self.count = 0
        self.imu_sub = None
        self.enable_self_balancing = False

        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        rospy.Service('~set_running', SetBool, self.set_running)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def set_running(self, msg):
        rospy.loginfo("self_balancing")
        if msg.data:
            self.pitch = 0
            self.roll = 0
            self.init_finish = False
            self.pitch_init = 0
            self.roll_init = 0
            self.count = 0
            self.pid_pitch.clear()
            self.pid_roll.clear()   
            if self.imu_sub is None:
                self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

            self.pose_pub.publish(0, 0, 0, -0.13, 0, 0, 0, 1)
            time.sleep(1.5)
            self.enable_self_balancing = msg.data
        else:
            self.enable_self_balancing = msg.data
            try:
                if self.imu_sub is not None:
                    self.imu_sub.unregister()
                    self.imu_sub = None
            except Exception as e:
                rospy.logerr(str(e))
            self.pose_pub.publish(0, 0, 0, -0.13, 0, 0, 0, 1)
        return [True, 'set_running']

    def imu_callback(self, msg):
        if self.enable_self_balancing:
            try:
                q = msg.orientation
                r = R.from_quat((q.x, q.y, q.z, q.w))
                x, y, z = r.as_euler('xyz')

                # rospy.loginfo("Pitch:{:>6.2f}, Roll:{:>6.2f}".format(x, y))
                if self.init_finish:
                    self.pid_pitch.SetPoint = self.pitch_init
                    self.pid_roll.SetPoint = self.roll_init
                    if abs(x - self.pitch_init) < 0.04:
                        x = self.pitch_init
                    self.pid_pitch.update(x)
                    if abs(y - self.roll_init) < 0.02:
                        y = self.roll_init
                    self.pid_roll.update(y)
                    try:
                        self.pitch -= self.pid_pitch.output
                        self.roll += self.pid_roll.output
                        pitch_limit = 0.35
                        roll_limit = 0.30
                        self.roll = roll_limit if self.roll > roll_limit else (-roll_limit if self.roll < -roll_limit else self.roll)
                        self.pitch = pitch_limit if self.pitch > pitch_limit else (-pitch_limit if self.pitch < -pitch_limit else self.pitch)
                        # print(math.degrees(self.pitch))
                        self.pose_pub.publish(self.roll, self.pitch, 0, -0.13, 0, 0, 0, 0)
                    except Exception as e:
                        rospy.logerr(str(e))
                        self.pitch = 0
                        self.roll = 0
                else:
                    self.count += 1
                    if self.count > 50:
                        self.count = 0
                        self.init_finish = True
                        self.pitch_init = x
                        self.roll_init = y
                        self.buzzer_pub.publish(1900, 0.3, 0.01, 1)
                        rospy.loginfo('init finish pitch:{} roll:{}'.format(x, y))
            except Exception as e:
                rospy.logerr(str(e))
        else:
            time.sleep(0.1)

if __name__ == "__main__":
    SelfBalancingNode('self_balancing')
