#!/usr/bin/env python3
# encoding: utf-8
import cv2
import math
import time
import queue
import rospy
import signal
import numpy as np
from pug_app import common
from threading import RLock
from sensor_msgs.msg import Image
from pug_control.srv import SetActionName
from geometry_msgs.msg import Point, Twist
from pug_app.common import Heart, ColorPicker
from std_srvs.srv import SetBool, Trigger, TriggerResponse, TriggerRequest, SetBoolResponse

from pug_sdk import pid
from pug_control.msg import Velocity, Pose
from pug_control.srv import SetFloat64, SetPoint, SetFloat64Response, SetPointResponse

class ObjectTracker:
    def __init__(self, color):
        self.y_dis = 0
        self.pid_y = pid.PID(0.0003, 0.0, 0.0)
        self.target_lab, self.target_rgb = color
        self.weight_sum = 1.0
        self.image_resize = (320, 180)

    def __call__(self, image, result_image, threshold):
        twist = Twist()
        img_h, img_w = image.shape[:2]
        image = cv2.resize(image, self.image_resize)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间
        image = cv2.GaussianBlur(image, (5, 5), 5)

        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓
        circle = None
        if len(contour_area) > 0:
            contour, area = max(contour_area, key=lambda c_a: c_a[1])
            circle = cv2.minEnclosingCircle(contour)
            (x, y), r = circle
            x = x / self.image_resize[0] * img_w
            y = y / self.image_resize[1] * img_h
            r = r / self.image_resize[0] * img_w

            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (self.target_rgb[0],
                                                                               self.target_rgb[1],
                                                                               self.target_rgb[2]), 2)

            self.pid_y.SetPoint = img_h / 2.0

            if abs(y - img_h / 2.0) < 20:
                y = img_h / 2.0
            self.pid_y.update(y)
            self.y_dis += self.pid_y.output

            self.y_dis = np.radians(20) if self.y_dis > np.radians(20) else self.y_dis
            self.y_dis = np.radians(-20) if self.y_dis < np.radians(-20) else self.y_dis
            return result_image, self.y_dis
        else:
            return result_image, None

class ObjectTrackNode:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.start = False
        self.threshold = 0.2
        self.color_picker = None
        self.tracker = None
        self.image_sub = None
        self.lock = RLock()
        self.running = True
        self.enter = False
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)  # register result image publisher
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        rospy.Service('~enter', Trigger, self.enter_func)
        rospy.Service('~exit', Trigger, self.exit_func)
        rospy.Service('~set_running', SetBool, self.set_running)
        
        rospy.Service('~set_target_color', SetPoint, self.set_target_color_srv_callback)
        rospy.Service('~get_target_color', Trigger, self.get_target_color_srv_callback)
        rospy.Service('~set_threshold', SetFloat64, self.set_threshold_srv_callback)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_func(None))  # 心跳包(heartbeat package)
        
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        self.run()

    def shutdown(self, signum, frame):
        self.running = False

    def enter_func(self, msg):
        rospy.loginfo("enter object tracking")
        self.run_action_group_srv('stand')
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                    self.image_sub = None
            except Exception as e:
                rospy.logerr(str(e))
            self.image_sub = rospy.Subscriber('/csi_camera/image_rect_color', Image, self.image_callback)
            self.threshold = 0.2
            self.color_picker = None
            self.tracker = None
            self.enter = True
        return [True, 'enter']

    def exit_func(self, msg):
        rospy.loginfo("exit object tracking")
        with self.lock:
            self.start = False
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                    self.image_sub = None
            except Exception as e:
                rospy.logerr(str(e))
            if isinstance(msg, TriggerRequest):
                self.heart.stop()
                self.run_action_group_srv('stand')
            self.color_picker = None
            self.tracker = None
            self.enter = False
        return [True, 'exit']

    def set_running(self, msg):
        if msg.data:
            rospy.loginfo("start running object tracking")
            with self.lock:
                self.start = True
        else:
            rospy.loginfo("stop running object tracking")
            with self.lock:
                self.start = False

        return [True, 'set_running']

    def set_target_color_srv_callback(self, msg):
        rospy.loginfo("set_target_color")
        with self.lock:
            x, y = msg.data.x, msg.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(msg.data, 20)
        return SetPointResponse(success=True)

    def get_target_color_srv_callback(self, msg):
        rospy.loginfo("get_target_color")
        rsp = TriggerResponse(success=False, message="")
        with self.lock:
            if self.tracker is not None:
                rsp.success = True
                rgb = self.tracker.target_rgb
                rsp.message = "{},{},{}".format(int(rgb[2]), int(rgb[1]), int(rgb[0]))
        return rsp

    def set_threshold_srv_callback(self, msg):
        rospy.loginfo("threshold")
        with self.lock:
            self.threshold = msg.data
            return SetFloat64Response(success=True)
   
    def image_callback(self, ros_image):
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将自定义图像消息转化为图像
        cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(cv2_img)

    def run(self):
        while self.running:
            if self.enter:
                image = self.image_queue.get(block=True)
                frame_result = image.copy()
                with self.lock:
                    # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取
                    if self.color_picker is not None:  # 拾取器存在
                        target_color, frame_result = self.color_picker(image, frame_result)
                        if target_color is not None:
                            self.color_picker = None
                            self.tracker = ObjectTracker(target_color)
                    else:
                        if self.tracker is not None:
                            try:
                                img_h, img_w = frame_result.shape[:2]
                                cv2.line(frame_result, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)),
                                         (255, 255, 0), 2)
                                cv2.line(frame_result, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)),
                                         (255, 255, 0), 2)
                                frame_result, y_dis = self.tracker(image, frame_result, self.threshold)
                                if not self.start:
                                    self.tracker.pid_y.clear()
                                else:
                                    self.pose_pub.publish(0, y_dis, 0, -0.13, 0, 0, 0, 0)
                            except Exception as e:
                                rospy.logerr(str(e))
        
                self.image_pub.publish(common.cv2_image2ros(frame_result, self.name))
            else:
                time.sleep(0.1)
    
if __name__ == '__main__':
    ObjectTrackNode('object_tracking')
