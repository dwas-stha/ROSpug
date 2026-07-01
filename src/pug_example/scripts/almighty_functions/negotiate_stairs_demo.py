#!/usr/bin/env python3
# encoding: utf-8
# Author:hiwonder
import cv2
import time
import math
import rospy
import queue
import signal
import threading
import numpy as np
from pug_sdk import common
from sensor_msgs.msg import Image
from pug_control.srv import SetActionName
from pug_control.msg import Velocity, Pose, Gait

class NegotiateStairsNode:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)  # INFO
        self.name = name
        self.running = True
        self.target_color = 'red'
        self.status = 'detect_stair'
        self.object_data = queue.Queue(1)
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.lab_data = common.get_yaml_data('/home/hiwonder/pug/src/lab_config/config/lab_config.yaml')['color_range_list']

        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber("/csi_camera/image_rect_color", Image, self.image_callback, queue_size=1)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
        self.velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        time.sleep(0.2)

        self.gait_pub.publish(0.2, 0.15, 0.0, 0.06)
        self.pose_pub.publish(0, math.radians(-17), 0, -0.13, 0, 0, 0, 0.5)
        time.sleep(1)
        threading.Thread(target=self.action_thread, daemon=True).start()
        self.run()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 将自定义图像消息转化为图像
        cv2_img = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(cv2_img)
    
     # 巡线逻辑处理
    # 机器人运动函数
    def action_thread(self):
        up_stairs_time = 0
        while self.running:
            object_data = self.object_data.get(block=True)
            object_centery, target_area = object_data[0], object_data[1]
            # print(object_centery, target_area)
            if self.status == 'detect_stair':
                self.velocity_pub.publish(0.08, 0, 0, False)
                time.sleep(0.2)
                if target_area > 8000 or object_centery > 200:
                    self.status = "find_stair"
                    self.velocity_pub.publish(0.08, 0, 0, False)
                    time.sleep(1.5)
                    self.velocity_pub.publish(0, 0, 0, True) # 停下
                    up_stairs_time = time.time()
            if self.status == "find_stair":
                for i in range(5):
                    self.run_action_group_srv("forward_little")
                for i in range(3):
                    self.run_action_group_srv("up_stair_0")
                            
                self.run_action_group_srv("up_stair_3")
                if time.time() - up_stairs_time > 25:
                    self.status = "down_stair"
                    self.pose_pub.publish(0, 0, 0, -0.13, 0.01, 0, 0, 0.5)
                    time.sleep(0.5)
                
            if self.status == "down_stair" :
                self.velocity_pub.publish(0.13, 0, 0, False)
                time.sleep(2)
                self.velocity_pub.publish(0, 0, 0, True) # 停下
                time.sleep(0.5)
                self.pose_pub.publish(0, math.radians(16), 0, -0.13, 0.01, 0, 0, 0.5)
                time.sleep(1)
                self.velocity_pub.publish(0.13, 0, 0, False)
                time.sleep(4)
                self.velocity_pub.publish(0, 0, 0, True) # 停下
                time.sleep(0.5)               
                self.pose_pub.publish(0, 0, 0, -0.13, 0.01, 0, 0, 0.5)
                time.sleep(0.5)
                self.status = "end" 
        
            if self.status == "end" :
                time.sleep(0.01)
                
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
            if self.status == 'detect_stair' or self.status == 'find_stair' : # 检测台阶/发现台阶
                img_copy = image.copy()
                gb_img = cv2.GaussianBlur(img_copy, (3, 3), 3)  # 高斯模糊
                frame_lab = cv2.cvtColor(gb_img, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
                frame_mask = cv2.inRange(frame_lab,
                                         (self.lab_data[self.target_color]['min'][0],
                                          self.lab_data[self.target_color]['min'][1],
                                          self.lab_data[self.target_color]['min'][2]),
                                         (self.lab_data[self.target_color]['max'][0],
                                          self.lab_data[self.target_color]['max'][1],
                                          self.lab_data[self.target_color]['max'][2]))  # 对原图像和掩模进行位运算

                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算

                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有外轮廓
                areaMax_contour, area = self.getAreaMaxContour(contours)  # 找到最大的轮廓
                object_data = [0, 0]
                if areaMax_contour is not None:
                    rect = cv2.minAreaRect(areaMax_contour)  # 最小外接矩形
                    box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                    cv2.drawContours(image, [box], -1, (0, 0, 255), 2)  # 画出四个点组成的矩形
                    # 获取矩形的对角点
                    pt1_x, pt1_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    object_data = [int((pt1_y + pt3_y) / 2), area]
                if self.object_data.full():
                    # 如果队列已满，丢弃最旧的图像
                    self.object_data.get()
                # 将图像放入队列
                self.object_data.put(object_data)
            cv2.imshow(self.name, image)
            cv2.waitKey(1)
        self.run_action_group_srv('stand')
        rospy.signal_shutdown('shutdown')
        
if __name__ == '__main__':
    NegotiateStairsNode('negotiate_stairs')      

