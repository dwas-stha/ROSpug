#!/usr/bin/env python3
# encoding: utf-8
import time
import math
import rospy
import signal
from threading import RLock
from pug_app.common import Heart
from pug_app.srv import SetTarget

from pug_control.srv import SetActionName
from pug_control.msg import Velocity, Pose, Gait
from std_srvs.srv import Trigger, SetBool, Empty, TriggerRequest

class Performance:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.enter = False
        self.running = True
        self.mode = None
        self.stop = False
        self.last_mode = None
        self.action_finish = True
        self.self_balancing_enable = False
        self.lock = RLock()
        signal.signal(signal.SIGINT, self.shutdown)
        self.gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
        self.velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)

        rospy.Service('~enter', Trigger, self.enter_func)
        rospy.Service('~exit', Trigger, self.exit_func)
        
        rospy.Service('~set_self_balancing', SetBool, self.set_self_balancing)
        rospy.Service('~set_target', SetTarget, self.set_target)

        self.go_home_srv = rospy.ServiceProxy('/pug_control/go_home', Empty)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        self.enable_self_balancing_srv = rospy.ServiceProxy('/self_balancing/set_running', SetBool)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_func(None))  # 心跳包(heartbeat package)
        self.run()

    def shutdown(self, signum, frame):
        self.running = False

    def enter_func(self, msg):
        rospy.loginfo("enter performance")
        with self.lock:
            self.enter = True
            self.mode = None
            self.action_finish = True
            self.self_balancing_enable = False
            self.enable_self_balancing_srv(self.self_balancing_enable)       

        return [True, 'enter']

    def exit_func(self, msg):
        rospy.loginfo("exit performance")
        with self.lock:
            self.enter = False
            self.go_home_srv()
            self.self_balancing_enable = False
            self.enable_self_balancing_srv(self.self_balancing_enable)
            if isinstance(msg, TriggerRequest):
                self.heart.stop()
        return [True, 'exit']

    def set_self_balancing(self, msg):
        self.self_balancing_enable = msg.data
        self.enable_self_balancing_srv(self.self_balancing_enable)
        
        return [True, 'set_self_balancing']

    def set_target(self, msg):
        with self.lock:
            self.mode = msg.data
        return [True, 'set_target']

    def run(self):
        while self.running:
            with self.lock:
                if self.enter:
                    if not self.self_balancing_enable and self.action_finish:
                        # print(1, self.last_mode, self.mode)
                        if self.last_mode == "sit" and self.mode != 'sit' and self.mode is not None:
                            self.run_action_group_srv('up')
                        if self.last_mode == "lie_down" and self.mode != 'lie_down' and self.mode is not None:
                            self.run_action_group_srv('stand')
                        if self.mode == '4_legs_stand':
                            self.go_home_srv()
                        elif self.mode == 'mark_time':#原地踏步
                            self.pose_pub.publish(0, 0, 0, -0.13, 0, 0, 0, 0.5)
                            time.sleep(0.5)
                            self.gait_pub.publish(0.2, 0.15, 0.0, 0.05)
                            self.velocity_pub.publish(0, 0, 0, False)
                            time.sleep(2)
                            self.velocity_pub.publish(0, 0, 0, True)
                        elif self.mode == 'crawl': #匍匐前进
                            self.gait_pub.publish(0.2, 0.15, 0.0, 0.05)
                            self.pose_pub.publish(0, 0, 0, -0.11, 0, 0, 0, 0.3)
                            time.sleep(0.3)
                            self.velocity_pub.publish(0.05, 0, 0, False)
                            time.sleep(3)
                            self.velocity_pub.publish(0, 0, 0, True)
                        elif self.mode in ["turn_pitch", "turn_roll", "multiaxial"]:
                            self.pose_pub.publish(0, 0, 0, -0.13, 0, 0, 0, 0.5)
                            time.sleep(0.5)
                            times = 0
                            angle_now = 0
                            angle_step = 0.32
                            angle_max = 15
                            roll = 0
                            pitch = 0
                            # if self.mode == 'turn_pitch':
                                # angle_max = 17
                            # if self.mode == 'turn_roll':
                                # angle_max = 15
                            while True:
                                if not self.enter:
                                    break
                                if self.mode in ["turn_pitch", "turn_roll"]:
                                    # print(times, angle_now)
                                    if times % 3 == 0:
                                        angle_now -= math.radians(angle_step)
                                        if angle_now <= math.radians(-angle_max):
                                            times += 1
                                    elif times % 3 == 1:
                                        angle_now += math.radians(angle_step)
                                        if angle_now >= math.radians(angle_max):
                                            times += 1
                                    elif times % 3 == 2:
                                        angle_now -= math.radians(angle_step)
                                        if angle_now <= math.radians(0):
                                            times += 1
                                            if times == 6:
                                                break
                                    if self.mode == "turn_pitch":
                                        self.pose_pub.publish(0, angle_now, 0, -0.13, 0, 0, 0, 0)
                                    else:
                                        self.pose_pub.publish(angle_now, 0, 0, -0.13, 0, 0, 0, 0)
                                    time.sleep(0.006)
                                elif self.mode == "multiaxial":
                                    if times == 12:
                                        roll += math.radians(angle_step)
                                        if roll >= math.radians(0):
                                            break
                                    elif times % 6 == 0:
                                        roll -= math.radians(angle_step)
                                        if roll <= math.radians(-angle_max):
                                            times += 1
                                    elif times % 6 == 1:
                                        pitch -= math.radians(angle_step)
                                        if pitch <= math.radians(-angle_max):
                                            times += 1
                                    elif times % 6 == 2:
                                        roll += math.radians(angle_step)
                                        if roll >= math.radians(angle_max):
                                            times += 1
                                    elif times % 6 == 3:
                                        pitch += math.radians(angle_step)
                                        if pitch >= math.radians(angle_max):
                                            times += 1
                                    elif times % 6 == 4:
                                        roll -= math.radians(angle_step)
                                        if roll <= math.radians(-angle_max):
                                            times += 1
                                    elif times % 6 == 5:
                                        pitch -= math.radians(angle_step)
                                        if pitch <= math.radians(0):
                                            times += 1
                                    self.pose_pub.publish(roll, pitch, 0, -0.13, 0, 0, 0, 0)
                                    time.sleep(0.006)
                        elif self.mode is not None:
                            # print(self.mode)
                            self.run_action_group_srv(self.mode)
                        else:
                            time.sleep(0.1)
                        if self.mode is not None:
                            self.last_mode = self.mode
                        self.mode = None
                        self.action_finish = True
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.1)
           

if __name__ == '__main__':
    Performance('performance')
