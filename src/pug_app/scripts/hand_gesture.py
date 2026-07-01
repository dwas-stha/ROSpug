#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/19
# @author:aiden
# 手势控制
import cv2
import time
import math
import rospy
import threading
import numpy as np
from threading import RLock
from pug_app.msg import Points
from pug_app.common import Heart
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, Trigger, TriggerResponse, TriggerRequest, Empty


class HandGestureControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.image_sub = None
        self.lock = RLock()
        self.last_point = [0, 0]
        self.linear_x_speed = 0.13
        self.linear_y_speed = 0.08
        self.th = None
        self.start = True
        self.vel_pub = rospy.Publisher('/pug_control/cmd_vel', Twist, queue_size=1)
        self.go_home_srv = rospy.ServiceProxy('/pug_control/go_home', Empty)
        rospy.Service('~enter', Trigger, self.enter_func)  # 进入玩法(enter the game)
        rospy.Service('~exit', Trigger, self.exit_func)  # 退出玩法(exit the game)
        rospy.Service('~set_running', SetBool, self.set_running_func)  # 开启玩法(start the game)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_func(None))  # 心跳包(heartbeat package)

    def enter_func(self, msg):
        with self.lock:
            self.go_home_srv()
            self.start = False
        rospy.ServiceProxy('/hand_trajectory/enter', Trigger)()
        self.image_sub = rospy.Subscriber('/hand_trajectory/points', Points, self.get_hand_points_callback)
        rospy.loginfo("hand gesture control enter")

        return TriggerResponse(success=True)

    def exit_func(self, msg):
        with self.lock:
            self.start = False
        if isinstance(msg, TriggerRequest):
            self.heart.stop()
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        rospy.ServiceProxy('/hand_trajectory/exit', Trigger)()
        rospy.ServiceProxy('/hand_trajectory/stop', Trigger)()
        rospy.loginfo("hand gesture control exit")
        self.vel_pub.publish(Twist())
        return TriggerResponse(success=True)

    def set_running_func(self, req):
        rospy.loginfo("set_running")

        if req.data:
            self.start = True
            rospy.ServiceProxy('/hand_trajectory/start', Trigger)()
        else:
            self.start = False
            rospy.ServiceProxy('/hand_trajectory/stop', Trigger)()
        self.vel_pub.publish(Twist())
        return SetBoolResponse(success=req.data)

    def move_action(self, *args):
        status = 0
        t_start = rospy.get_time()
        while self.start:
            current_time = rospy.get_time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = args[0]
                self.vel_pub.publish(twist)
                t_start = current_time + args[1] / 50.0 / self.linear_x_speed
            elif status == 1 and t_start < current_time:
                break
            time.sleep(0.01)
        self.vel_pub.publish(Twist())

    def get_hand_points_callback(self, msg):
        points = []
        left_and_right = [0]
        up_and_down = [0]
        if len(msg.points) >= 5:
            for i in msg.points:
                if int(i.x) - self.last_point[0] > 0:
                    left_and_right.append(1)
                else:
                    left_and_right.append(-1)
                if int(i.y) - self.last_point[1] > 0:
                    up_and_down.append(1)
                else:
                    up_and_down.append(-1)
                points.extend([(int(i.x), int(i.y))])
                self.last_point = [int(i.x), int(i.y)]
            line = cv2.fitLine(np.array(points), cv2.DIST_L2, 0, 0.01, 0.01)
            angle = int(abs(math.degrees(math.acos(line[0][0]))))
            twist = Twist()
            if 0 <= angle < 30:
                if sum(left_and_right) > 0:
                    twist.linear.y = self.linear_y_speed
                else:
                    twist.linear.y = -self.linear_y_speed
            elif 60 < angle <= 90:
                if sum(up_and_down) > 0:
                    twist.linear.x = self.linear_x_speed
                else:
                    twist.linear.x = -self.linear_x_speed
            if self.th is None:
                self.th = threading.Thread(target=self.move_action, args=(twist, len(points)))
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.move_action, args=(twist, len(points)))
                    self.th.start()
                else:
                    self.start = False
                    time.sleep(0.1)


if __name__ == "__main__":
    HandGestureControlNode('hand_gesture')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

