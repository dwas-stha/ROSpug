#!/usr/bin/env python3
# encoding: utf-8
# Author:hiwonder
import cv2
import time
import math
import queue
import rospy
import signal
import numpy as np
from pug_sdk import common, pid
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from pug_control.srv import SetActionName
from pug_control.msg import Velocity, Pose, Gait

class VisualPatrolNode:

    roi = [(290, 300,  0, 640, 0.02),
           (320, 330,  0, 640, 0.10),
           (350, 360,  0, 640, 0.88)]

    roi_h1 = roi[0][0]
    roi_h2 = roi[1][0] - roi[0][0]
    roi_h3 = roi[2][0] - roi[1][0]
    roi_h_list = [roi_h1, roi_h2, roi_h3]
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)  # INFO
        self.name = name
        self.running = True
        self.pid = pid.PID(1.1, 0.0, 0.0)
        self.line_color = 'black'
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        self.lab_data = common.get_yaml_data('/home/hiwonder/pug/src/lab_config/config/lab_config.yaml')['color_range_list']

        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber("/csi_camera/image_rect_color", Image, self.image_callback, queue_size=1)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        time.sleep(0.1)
        self.gait_pub.publish(0.2, 0.15, 0.0, 0.05)
        self.pose_pub.publish(0, math.radians(-18), 0, -0.13, 0, 0, 0, 0.5)
        self.run()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 将自定义图像消息转化为图像
        cv2_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(cv2_image)

    # 找出面积最大的轮廓
    # 参数为要比较的轮廓的列表
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c
        return area_max_contour, contour_area_max  # 返回最大的轮廓

    # 主线程运行图像处理
    def run(self):
        while self.running:
            image = self.image_queue.get(block=True)
            
            img_copy = image.copy()
            img_h, img_w = image.shape[:2]
            frame_gb = cv2.GaussianBlur(img_copy, (3, 3), 3)

            n = 0
            center_ = []
            weight_sum = 0
            centroid_x_sum = 0
            # 将图像分割成上中下三个部分，这样处理速度会更快，更精确
            for r in self.roi:
                roi_h = self.roi_h_list[n]
                blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
                frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
                n += 1
                if self.line_color in self.lab_data:
                    frame_mask = cv2.inRange(frame_lab,
                                             (self.lab_data[self.line_color]['min'][0],
                                              self.lab_data[self.line_color]['min'][1],
                                              self.lab_data[self.line_color]['min'][2]),
                                             (self.lab_data[self.line_color]['max'][0],
                                              self.lab_data[self.line_color]['max'][1],
                                              self.lab_data[self.line_color]['max'][2]))  # 对原图像和掩模进行位运算
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓
                    max_contours = self.getAreaMaxContour(contours)[0]  # 找到最大面积的轮廓
    
                    if max_contours is not None:  # 如果轮廓不为空   
                        rect = cv2.minAreaRect(max_contours)  # 最小外接矩形
                        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                        for i in range(4):
                            box[i, 1] = int(box[i, 1] + (n - 1) * roi_h + self.roi[0][0])
                        cv2.drawContours(image, [box], -1, (0, 0, 255, 255), 2)  # 画出四个点组成的矩形
                        # 获取矩形的对角点
                        pt1_x, pt1_y = box[0, 0], box[0, 1]
                        pt3_x, pt3_y = box[2, 0], box[2, 1]
    
                        center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  # 中心点
                        cv2.circle(image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)  # 画出中心点
                        center_.append([center_x, center_y])
                        # 按权重不同对上中下三个中心点进行求和
                        centroid_x_sum += center_x * r[4]
                        weight_sum += r[4]
            twist = Twist()
            twist.linear.x = 0.15
            if weight_sum is not 0:
                # 求最终得到的中心点
                line_centerx = int(centroid_x_sum / weight_sum)
                deflection_angle = -math.atan((line_centerx - (img_w / 2.0)) / (img_h / 2.0))/2 
                # print(deflection_angle)
                # self.pid.SetPoint = 0.138
                self.pid.update(deflection_angle)
                twist.angular.z = common.set_range(-self.pid.output, -1.0, 1.0)
                self.cmd_vel_pub.publish(twist)

            cv2.imshow(self.name, image)
            cv2.waitKey(1)
        self.run_action_group_srv('stand')
        rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    VisualPatrolNode('visual_patrol_demo')
