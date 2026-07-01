#!/usr/bin/env python3
# encoding: utf-8
import cv2
import math
import time
import rospy
import signal
import numpy as np
from threading import RLock
from pug_sdk import apriltag
from pug_app.common import Heart
from sensor_msgs.msg import Image
from pug_control.msg import Velocity, Pose
from pug_app.srv import SetTarget
from pug_control.srv import SetActionName
from std_srvs.srv import Trigger, SetBool, Empty, TriggerRequest

class ApriltagDetectNode:
    # 用到的动作组名称
    # id_1_action = 'shake_hands'
    id_2_action = 'push-up'
    # id_3_action = 'multi_turn'

    detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        self.image_sub = None
        self.lock = RLock()
        self.start = False
        self.tag_id = None
        self.running = True
        self.enter = False
        self.action_finish = True
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)  # register result image publisher
    
        rospy.Service('~enter', Trigger, self.enter_func)
        rospy.Service('~exit', Trigger, self.exit_func)
        rospy.Service('~set_running', SetBool, self.set_running)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_func(None))  # 心跳包(heartbeat package)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.velocity_pub = rospy.Publisher('/pug_control/velocity_move', Velocity, queue_size=1)
        self.go_home_srv = rospy.ServiceProxy('/pug_control/go_home', Empty)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)

        self.move()
       
    def shutdown(self, signum, frame):
        self.running = False

    def enter_func(self, msg):
        rospy.loginfo("enter apriltag detect")
        with self.lock:
            self.go_home_srv()
            time.sleep(0.5)
            self.tag_id = None
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                    self.image_sub = None
            except Exception as e:
                rospy.logerr(str(e))
            self.image_sub = rospy.Subscriber('/csi_camera/image_rect_color', Image, self.image_callback)
            self.enter = True
        return [True, 'enter']

    def exit_func(self, msg):
        rospy.loginfo("exit apriltag detect")
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
            self.enter = False
        return [True, 'exit']

    def set_running(self, msg):
        if msg.data:
            rospy.loginfo("start running apriltag detect")
            with self.lock:
                self.start = True
        else:
            rospy.loginfo("stop running apriltag detect")
            with self.lock:
                self.start = False
                self.tag_id = None

        return [True, 'set_running']
    
    def apriltag_detect(self, img):   
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        detections = self.detector.detect(gray, return_image=False)
        if len(detections) != 0:
            for detection in detections:                       
                corners = np.rint(detection.corners)  # 获取四个角点
                cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 5, cv2.LINE_AA)
                tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
                if tag_family == 'tag36h11':
                    tag_id = str(detection.tag_id)  # 获取tag_id
                    return tag_id
                else:
                    return None
        else:
            return None
    
    def move(self):
        while self.running:
            if self.enter:
                if self.start:
                    # time.sleep(0.01)
                    self.action_finish = False
                    if self.tag_id == '1':
                        self.go_home_srv()
                        self.velocity_pub.publish(0, 0, 0, False)
                        time.sleep(2)
                        self.velocity_pub.publish(0, 0, 0, True)
                        time.sleep(2)
                        self.go_home_srv()
                        time.sleep(0.5)
                        self.tag_id = None
                    elif self.tag_id == '2':
                        # self.go_home_srv()
                        self.run_action_group_srv(self.id_2_action)
                        time.sleep(0.5)
                        time.sleep(0.5)
                        self.tag_id = None
                    elif self.tag_id == '3':
                        self.pose_pub.publish(0, 0, 0, -0.13, 0, 0, 0, 0.5)
                        time.sleep(0.5)
                        times = 0
                        angle_now = 0
                        angle_step = 0.32
                        angle_max = 15
                        roll = 0
                        pitch = 0
                        while True:
                            if not self.enter or not self.start or self.tag_id != '3':
                                self.tag_id = None
                                break
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
                        self.tag_id = None
                    self.action_finish = True
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.1)
        rospy.signal_shutdown('shutdown')
    
    def image_callback(self, ros_image):
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将自定义图像消息转化为图像
        cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            if self.start and self.action_finish:
                self.tag_id = self.apriltag_detect(cv2_img)
        frame_result = cv2_img.copy()
        rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tobytes()
        ros_image.data = rgb_image
        self.image_pub.publish(ros_image)

if __name__ == '__main__':
    ApriltagDetectNode('apriltag_detect')
