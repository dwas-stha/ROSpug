#!/usr/bin/env python3
import cv2
import enum
import time
import rospy
import signal
import threading
import numpy as np
from queue import Queue
import mediapipe as mp
# from pug_sdk import buzzer
from sensor_msgs.msg import Image
from pug_control.msg import Pose
from pug_control.srv import SetActionName
from ros_robot_controller.msg import BuzzerState
from pug_sdk.common import FPS, distance, vector_2d_angle

class State(enum.Enum):
    NULL = 0
    INTO_IMITATION_1 = 1
    IMITATION = 2

def get_angle(p1, p2, p3):
    """
    获取三个点形成的夹角
    :param p1: 第一个点
    :param p2: 第二个点
    :param p3: 第三个点
    :return: 角度
    """
    angle = vector_2d_angle(p2 - p1, p3 - p2)
    return angle

def is_pentagon(landmarks):
    """
    通过手工2d几何特征判断两手是否举过头顶, 并且手臂与肩部形成五边形
    :param landmarks: 肢体的各个关键点
    :return: True or False 符合或不符合
    """
    try:
        shoulder_width = distance(landmarks[12], landmarks[11])  # 肩宽
        hand_dist = distance(landmarks[16], landmarks[15])  # 两手腕的距离
        if hand_dist > shoulder_width:  # 手腕距离要比肩宽小
            return False
        for p in landmarks[:7]:  # 两手腕要举过头，比眼鼻都高
            if landmarks[15][1] > p[1] or landmarks[16][1] > p[1]:
                return False
        if get_angle(landmarks[11], landmarks[12], landmarks[14]) < 40:
            return False  # 左上臂要举起超过肩部40度以上
        if get_angle(landmarks[13], landmarks[11], landmarks[12]) < 40:
            return False  # 右上臂要举起超过肩部40度以上
    except BaseException as e:
        print(1, e)
    return True

def is_level(landmarks, angle_threshold=15):
    """
    通过手工2d几何特征判断肩部是否和画面水平
    :param landmarks:
    :param angle_threshold:
    :return:
    """
    try:
        p0 = landmarks[12].copy()
        p0[0] = 0
        if abs(get_angle(p0, landmarks[12], landmarks[11])) > angle_threshold:
            return False
    except BaseException as e:
        print(2, e)
    return True

def is_flat(landmarks, angle_threshold):
    """
    通过手工2d几何特征判断双臂是否展开
    :param landmarks:
    :param angle_threshold:
    :return: True or False 符合或不符合
    """
    try:
        arm_marks = [15, 13, 11, 12, 14, 16]
        for i in range(3):
            angle = get_angle(landmarks[arm_marks[i]], landmarks[arm_marks[i + 1]], landmarks[arm_marks[i + 2]])
            if abs(angle) > angle_threshold:
                return False
    except BaseException as e:
        print(3, e)
    return is_level(landmarks, angle_threshold)

def is_cross(landmarks):
    """
    通过手工2d几何特征判断双臂是否居高并交叉
    :param landmarks:
    :return: True of False
    """
    try:
        if landmarks[16][0] <= landmarks[15][0]:
            return False
    except BaseException as e:
        print(4, e)
    return is_pentagon(landmarks)

def mp_pose_landmarks(results, img_rgb, draw=True):
    lm_list = []
    if results and results.pose_landmarks:
        h, w, = img_rgb.shape[:2]
        for idx, lm in enumerate(results.pose_landmarks.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)
            lm_list.append([cx, cy])
            if draw:
                cv2.circle(img_rgb, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        cv2.circle(img_rgb, tuple(lm_list[11]), 5, (255, 255, 0), cv2.FILLED)
        cv2.circle(img_rgb, tuple(lm_list[12]), 5, (0, 255, 255), cv2.FILLED)
        cv2.circle(img_rgb, tuple(lm_list[14]), 5, (0, 255, 0), cv2.FILLED)

    if len(lm_list) > 0:
        return np.array(lm_list)
    else:
        return None

class MankindPoseNode:
    def __init__(self):
        rospy.init_node("pose_control_node")
        self.state = State.NULL
        self.timestamp = 0
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.4
        )
        self.drawing = mp.solutions.drawing_utils

        self.r_x_dist = 0
        self.l_x_dist = 0
        self.count = 0
        self.height = None
        self.running = True
        self.action_finish = True

        self.fps = FPS()
        self.fps.fps = 10
        self.fps.update()
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = Queue(maxsize=2)  # 图像队列，增大队列大小
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.image_sub = rospy.Subscriber("/csi_camera/image_rect_color", Image, self.image_enqueue, queue_size=1)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        time.sleep(0.1)
        self.run_action_group_srv('stand')
        rospy.loginfo("human pose node created")
        threading.Thread(target=self.action_thread, daemon=True).start()
        self.run()

    def shutdown(self, signum, frame):
        self.running = False

    def image_enqueue(self, ros_image):
        # 将图像数据存入队列
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) 
        rgb_image = cv2.flip(rgb_image, 1)  # 镜像画面
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    def stop_imitation(self):
        self.state = State.NULL
        self.buzzer_pub.publish(1900, 0.5, 0.01, 1)
   
    def action_thread(self):
        while self.running:
            if self.action_finish:
                if self.height is not None:
                    self.action_finish = False
                    self.pose_pub.publish(0, 0, 0, self.height, 0, 0, 0, 0.3)
                    time.sleep(0.3)
                    self.action_finish = True
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def run(self):
        while self.running:
            rgb_image = self.image_queue.get()  # 从队列中取出图像数据进行处理
            result_image = np.copy(rgb_image)
            try:
                results = self.pose.process(rgb_image)
            except:
                results = False
            if results:
                self.drawing.draw_landmarks(result_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
                landmarks = mp_pose_landmarks(results, result_image, True)
                if landmarks is not None:
                    if self.state == State.NULL:
                        if time.time() - self.timestamp > 5:
                            if is_pentagon(landmarks):  # 双手举高开始测试进入模仿模式
                                self.state = State.INTO_IMITATION_1
                                self.timestamp = time.time()
                                self.buzzer_pub.publish(1900, 0.1, 0.01, 2)
                    elif self.state == State.INTO_IMITATION_1:
                        if is_flat(landmarks, 30):  # 双手水平展开
                            self.count += 1
                            if self.count > 2:
                                self.state = State.IMITATION  # 计入模仿模式
                                # 计算两上臂的像素长度
                                self.r_x_dist = distance(landmarks[13], landmarks[11])
                                self.l_x_dist = distance(landmarks[12], landmarks[14])
                                self.buzzer_pub.publish(1900, 0.1, 0.01, 2)
                        else:
                            self.count = 0
                            # 超时没有成功进入模仿模式就重新识别
                            if time.time() - self.timestamp > 5:
                                self.state = State.NULL
                                self.buzzer_pub.publish(1900, 0.5, 0.01, 1)
                    else:
                        self.timestamp = time.time()
                        # 模仿动作
                        if not is_cross(landmarks):
                            left_angle_2 = get_angle(landmarks[11], landmarks[12], landmarks[14])
                            right_angle_2 = get_angle(landmarks[12], landmarks[11], landmarks[13])
                            height = None
                            if left_angle_2 > -20 and left_angle_2 < 10:
                                height = -0.13
                            elif left_angle_2 <= -20:
                                height = -0.09
                            elif left_angle_2 >= 10:
                                height = -0.16
                            self.height = height
                        else:  # 双手居高交叉时退出模仿模式
                            self.stop_imitation()
                else:
                    # 超过一定时间没有识别到肢体就退出模仿模式
                    if time.time() - self.timestamp > 2 and self.state != State.NULL:
                        self.stop_imitation()

            self.fps.update()
            self.fps.show_fps(result_image)
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow('image', result_image)
            cv2.waitKey(1)
        self.run_action_group_srv('stand')
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    MankindPoseNode()
