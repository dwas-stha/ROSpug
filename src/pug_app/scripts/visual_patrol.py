#!/usr/bin/env python3
# encoding: utf-8
# 巡线(line following)
import cv2
import time
import math
import rospy
import queue
import signal
import threading
import numpy as np
from pug_sdk import pid, common
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from pug_app.common import cv2_image2ros
from pug_app.common import Heart, ColorPicker
from pug_control.msg import Velocity, Pose, Gait
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from pug_control.srv import SetPoint, SetPointRequest, SetPointResponse
from pug_control.srv import SetFloat64, SetFloat64Request, SetFloat64Response

class LineFollower:
    def __init__(self, color):
        self.target_lab, self.target_rgb = color
        self.rois = ((290, 300,  0, 640, 0.02), (320, 330,  0, 640, 0.10), (350, 360,  0, 640, 0.88))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(get the contour of the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        h, w = image.shape[:2]
        min_color = [int(self.target_lab[0] - 50 * threshold * 2), 
                     int(self.target_lab[1] - 50 * threshold), 
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2), 
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi(intercept roi)
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(image binarization)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            #cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.int0(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)
                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)
                centroid_sum += line_center_x * roi[-1]
        if centroid_sum == 0:
            return result_image, None
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)

        return result_image, deflection_angle

class LineFollowingNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.start = False
        self.color_picker = None
        self.follower = None
        # self.pid = pid.PID(0.9, 0.01, 0.001)
        self.pid = pid.PID(1.1, 0.0, 0.0)
        self.empty = 0
        self.threshold = 0.35
        self.lock = threading.RLock()
        self.image_sub = None
        self.running = True
        self.enter = False
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 底盘控制(chassis control)

        self.go_home_srv = rospy.ServiceProxy('/pug_control/go_home', Empty)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
        self.velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)

        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布(publish the image processing result)
        self.enter_srv = rospy.Service('~enter', Trigger,  self.enter_srv_callback)  # 进入玩法(enter the game)
        self.exit_srv = rospy.Service("~exit", Trigger, self.exit_srv_callback)  # 退出玩法(exit the game)
        self.set_running_srv = rospy.Service('~set_running', SetBool, self.set_running_srv_callback)  # 开启玩法(start the game)
        self.set_target_color_srv = rospy.Service('~set_target_color', SetPoint, self.set_target_color_srv_callback)  # 设置颜色(set the color)
        self.get_target_color_srv = rospy.Service('~get_target_color', Trigger, self.get_target_color_srv_callback)   # 获取颜色(get the color)
        self.set_threshold_srv = rospy.Service('~set_threshold', SetFloat64, self.set_threshold_srv_callback)  # 设置阈值(set the threshold)

        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
        self.run()

    def shutdown(self, signum, frame):
        self.running = False

    def enter_srv_callback(self, msg):
        rospy.loginfo("visual patrol enter")
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None
        except Exception as e:
            rospy.logerr(str(e))

        with self.lock:
            self.start = False
            self.enter = True
            self.color_picker = None
            self.follower = None
            self.threshold = 0.35
            self.velocity_pub.publish(0, 0, 0, True)
            self.pose_pub.publish(0, math.radians(-18), 0, -0.13, 0, 0, 0, 0.5)
            self.gait_pub.publish(0.2, 0.15, 0.0, 0.05)
            self.image_sub = rospy.Subscriber('/csi_camera/image_rect_color', Image, self.image_callback)  # 摄像头订阅(subscribe to the camera)
        return TriggerResponse(success=True)

    def exit_srv_callback(self, msg):
        rospy.loginfo("visual patrol exit")
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None
        except Exception as e:
            rospy.logerr(str(e))
        with self.lock:
            self.enter = False
            self.start = False
            if isinstance(msg, TriggerRequest):
                self.heart.stop()
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)
                self.go_home_srv()
        return TriggerResponse(success=True)

    def set_target_color_srv_callback(self, req: SetPointRequest):
        rospy.loginfo("set_target_color")
        with self.lock:
            x, y = req.data.x, req.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(req.data, 20)
                self.cmd_vel_pub.publish(Twist())
        return SetPointResponse(success=True)

    def get_target_color_srv_callback(self, msg):
        rospy.loginfo("get_target_color")
        rsp = TriggerResponse(success=False, message="")
        with self.lock:
            if self.follower is not None:
                rsp.success = True
                rgb = self.follower.target_rgb
                rsp.message = "{},{},{}".format(int(rgb[2]), int(rgb[1]), int(rgb[0]))
                print("{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2])))
        return rsp

    def set_running_srv_callback(self, req: SetBoolRequest):
        rospy.loginfo("set_running")
        with self.lock:
            self.start = req.data
            if not self.start:
                self.cmd_vel_pub.publish(Twist())
        return SetBoolResponse(success=req.data)

    def set_threshold_srv_callback(self, req: SetFloat64Request):
        rospy.loginfo("set threshold")
        with self.lock:
            self.threshold = req.data
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
                    # 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取(color picker and line recognition are exclusive. If there is color picker, start picking)
                    if self.color_picker is not None:  # 拾取器存在(color picker exists)
                        try:
                            target_color, frame_result = self.color_picker(image, frame_result)
                            if target_color is not None:
                                self.color_picker = None
                                self.follower = LineFollower(target_color) 
                        except Exception as e:
                            rospy.logerr(str(e))
                    else:
                        twist = Twist()
                        twist.linear.x = 0.15
                        if self.follower is not None:
                            try:
                                frame_result, deflection_angle = self.follower(image, frame_result, self.threshold)
                                # print(deflection_angle)
                                if deflection_angle is not None and self.start:
                                    self.pid.update(deflection_angle/2)
                                    twist.angular.z = common.set_range(-self.pid.output, -1.0, 1.0)
                                    self.cmd_vel_pub.publish(twist)
                                else:
                                    self.pid.clear()
                            except Exception as e:
                                rospy.logerr(str(e))
                self.image_pub.publish(cv2_image2ros(frame_result, self.name))
            else:
                time.sleep(0.1)

if __name__ == "__main__":
    LineFollowingNode('visual_patrol')
