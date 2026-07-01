#!/usr/bin/env python3
import os
import cv2
import time
import queue
import rospy
import signal
import numpy as np
from pug_sdk import pid
from pug_sdk.common import FPS
from sensor_msgs.msg import Image
from pug_control.msg import  Pose
from pug_control.srv import SetActionName

class KCFTrackingNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.y_dis = 0
        self.y_pid = pid.PID(P=0.0005, I=0, D=0)

        self.image_queue = queue.Queue(maxsize=2)
        self.fps = FPS()
        signal.signal(signal.SIGINT, self.shutdown)
        self.running = True
        self.mouse_click = False
        self.selection = None  # 实时跟踪鼠标的跟踪区域
        self.track_window = None  # 要检测的物体所在区域
        self.drag_start = None  # 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False
        self.params = cv2.TrackerNano_Params()
        model_path = os.path.split(os.path.realpath(__file__))[0]
        self.params.backbone = os.path.join(model_path, 'nanotrack_backbone_sim.onnx')
        self.params.neckhead = os.path.join(model_path, 'nanotrack_head_sim.onnx')
        self.tracker = cv2.TrackerNano_create(self.params)

        cv2.namedWindow(name, 1)
        cv2.setMouseCallback(name, self.onmouse)
        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber("/csi_camera/image_rect_color", Image, self.image_callback, queue_size=1)
        self.pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        time.sleep(0.2)
        self.run_action_group_srv('stand')
        self.run()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 将自定义图像消息转化为图像
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    # 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # 鼠标起始位置
            self.track_window = None
        if self.drag_start:  # 是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # 鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None  # 实时跟踪鼠标的跟踪区域
            self.track_window = None  # 要检测的物体所在区域
            self.drag_start = None  # 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False
            self.run_action_group_srv('stand')
            self.tracker = cv2.TrackerNano_create(self.params)

    def image_proc(self, image):
        if self.start_circle:
            # 用鼠标拖拽一个框来指定区域
            if self.track_window:  # 跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:  # 跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 0, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click:
                if not self.mouse_click:
                    self.start_circle = False
            if not self.start_circle:
                print('start tracking')
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
        else:
            ok, box = self.tracker.update(image)
            if ok and min(box) > 0:
                p1 = (int(box[0]), int(box[1]))
                p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))
                cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
                center_y = (p1[1] + p2[1]) / 2
                img_h, img_w = image.shape[:2]
                self.y_pid.SetPoint = img_h / 2.0

                self.y_pid.update(center_y)
                self.y_dis += self.y_pid.output

                self.y_dis = np.radians(20) if self.y_dis > np.radians(20) else self.y_dis
                self.y_dis = np.radians(-20) if self.y_dis < np.radians(-20) else self.y_dis

                self.pose_pub.publish(0, self.y_dis, 0, -0.13, 0, 0, 0, 0)
            else:
                # Tracking failure
                cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 0), 1)
        self.fps.update()
        result_image = self.fps.show_fps(image)

        return result_image

    def run(self):
        while self.running:
            image = self.image_queue.get(block=True)
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                print(e)
                result_image = image
            cv2.imshow(self.name, cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        self.run_action_group_srv('stand')
        rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    KCFTrackingNode('kcf_tracking')
