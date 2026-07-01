#!/usr/bin/env python3
# encoding: utf-8
# Author:hiwonder
import cv2
import time
import math
import queue
import rospy
import signal
import threading
import numpy as np
from pug_sdk import common
from sensor_msgs.msg import Image
from pug_control.srv import SetActionName
from pug_control.msg import Velocity, Pose, Gait

class KickBallNode:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)  # INFO
        self.name = name
        self.status = 'find_ball'
        self.target_color = 'purple'
        self.running = True
        self.image_queue = queue.Queue(maxsize=2)
        self.ball_position_queue = queue.Queue(1)
        signal.signal(signal.SIGINT, self.shutdown)
        self.lab_data = common.get_yaml_data('/home/hiwonder/pug/src/lab_config/config/lab_config.yaml')['color_range_list']

        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber("/csi_camera/image_rect_color", Image, self.image_callback, queue_size=1)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.gait_pub = rospy.Publisher('/pug_control/gait', Gait, queue_size=1)
        self.velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        time.sleep(0.1)

        self.gait_pub.publish(0.2, 0.15, 0.0, 0.02)
        self.pose_pub.publish(0, math.radians(-17), 0, -0.13, 0, 0, 0, 0.5)
        threading.Thread(target=self.action_thread, daemon=True).start()
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

    # 机器人运动函数
    def action_thread(self):
        while self.running:
            ball_position = self.ball_position_queue.get(block=True)
            if ball_position[0] != -1:
                ball_center_x, ball_center_y = ball_position[0], ball_position[1]
                # print(ball_center_x, ball_center_y)
                if self.status == 'find_ball':  # 发现方块
                    if ball_center_y < 250 and 200 < ball_center_x < 600: # 前进
                        self.velocity_pub.publish(0.1, 0, 0, False)
                        time.sleep(0.5)
                    elif ball_center_y < 300 and 200 < ball_center_x < 600: # 小步前进
                        self.velocity_pub.publish(0.05, 0, 0, False)
                        time.sleep(0.5)
                    elif ball_center_x > 600:  # 右转
                        self.velocity_pub.publish(0.01, 0, math.radians(-10), False)
                    elif ball_center_x < 540:  # 左转
                        self.velocity_pub.publish(0.01, 0, math.radians(10), False)
                    else:
                        self.status = 'kick_ball'
                elif self.status == 'kick_ball' :
                    if ball_center_y < 330: # 小步前进
                        self.velocity_pub.publish(0.05, 0, 0, False)
                        time.sleep(0.5)
                    elif ball_center_x > 600:  # 右转
                        self.velocity_pub.publish(0.0, 0, math.radians(-10), False)
                    elif ball_center_x < 540:  # 左转
                        self.velocity_pub.publish(0.0, 0, math.radians(10), False)
                    else:
                        self.velocity_pub.publish(0.08, 0, 0, False) # 小步前进
                        time.sleep(0.8)
                        self.velocity_pub.publish(0, 0, 0, True) # 停下
                        time.sleep(0.5)
                        self.run_action_group_srv('stand')
                        self.run_action_group_srv('kick_ball_right')
                        self.status = 'End'
                elif self.status == 'End':  # 结束
                    time.sleep(0.01)
            else:
                time.sleep(0.01)
    
    # 主线程运行图像处理
    def run(self):
        while self.running:
            image = self.image_queue.get(block=True)

            if self.status == 'find_ball' or self.status == 'kick_ball':  # 发现方块
                GaussianBlur_img = cv2.GaussianBlur(image, (3, 3), 3)  # 高斯模糊
                frame_lab = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
                frame_mask = cv2.inRange(frame_lab,
                                         (self.lab_data[self.target_color]['min'][0],
                                          self.lab_data[self.target_color]['min'][1],
                                          self.lab_data[self.target_color]['min'][2]),
                                         (self.lab_data[self.target_color]['max'][0],
                                          self.lab_data[self.target_color]['max'][1],
                                          self.lab_data[self.target_color]['max'][2]))  # 对原图像和掩模进行位运算
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有外轮廓
                areaMax_contour = self.getAreaMaxContour(contours)[0]  # 找到最大的轮廓

                if self.ball_position_queue.full():
                    self.ball_position_queue.get()
                if areaMax_contour is not None:
                    ball = cv2.minEnclosingCircle(areaMax_contour)  # 获取最小外接圆的圆心以及半径
                    ball_centerx = int(ball[0][0])
                    ball_centery = int(ball[0][1])
                    # print(ball_centerx, ball_centery, ball[1])
                    self.ball_position_queue.put([ball_centerx, ball_centery])
                    cv2.circle(image, (ball_centerx, ball_centery), int(ball[1]), (0, 255, 0), 2)
                    cv2.circle(image, (ball_centerx, ball_centery), 5, (0, 255, 0), -1)
                else:
                    self.ball_position_queue.put([-1, -1])
            cv2.imshow(self.name, image)
            cv2.waitKey(1)
        self.run_action_group_srv('stand')
        rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    KickBallNode('kick_ball')
