#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import time
import rospy
import queue
import signal
import threading
import numpy as np
from threading import RLock
from pug_app import common
from pug_app.common import Heart
from pug_app.yunet import YuNet
from sensor_msgs.msg import Image
from pug_control.srv import SetActionName
from std_srvs.srv import Trigger, SetBool, Empty, TriggerRequest

class FaceDetectNode:
    backend_target_pairs = [
        [cv2.dnn.DNN_BACKEND_OPENCV, cv2.dnn.DNN_TARGET_CPU],
        [cv2.dnn.DNN_BACKEND_CUDA,   cv2.dnn.DNN_TARGET_CUDA],
        [cv2.dnn.DNN_BACKEND_CUDA,   cv2.dnn.DNN_TARGET_CUDA_FP16],
        [cv2.dnn.DNN_BACKEND_TIMVX,  cv2.dnn.DNN_TARGET_NPU],
        [cv2.dnn.DNN_BACKEND_CANN,   cv2.dnn.DNN_TARGET_NPU]
    ]
    landmark_color = [
        (255,   0,   0), # right eye
        (  0,   0, 255), # left eye
        (  0, 255,   0), # nose tip
        (255,   0, 255), # right mouth corner
        (  0, 255, 255)  # left mouth corner
    ]
    model_path = os.path.split(os.path.realpath(__file__))[0]
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name
        backend_id = self.backend_target_pairs[1][0]
        target_id = self.backend_target_pairs[1][1]
        self.image_proc_size = [640, 360]
        self.image_queue = queue.Queue(maxsize=2)
        # Instantiate YuNet
        self.model = YuNet(modelPath=os.path.join(os.path.dirname(self.model_path), 'models/face_detection_yunet_2023mar.onnx'),
                      inputSize=self.image_proc_size,
                      confThreshold=0.6,
                      nmsThreshold=0.3,
                      topK=5000,
                      backendId=backend_id,
                      targetId=target_id)
        self.model.infer(np.zeros((self.image_proc_size[1], self.image_proc_size[0], 3), dtype=np.uint8))

        self.start = False
        self.image_sub = None
        self.lock = RLock()
        self.running = True
        self.detect_face = False
        self.face_count = 0
        self.start_greet = False
        self.action_finish = True
        self.enter = False
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)  # register result image publisher
    
        rospy.Service('~enter', Trigger, self.enter_func)
        rospy.Service('~exit', Trigger, self.exit_func)
        rospy.Service('~set_running', SetBool, self.set_running)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_func(None))  # 心跳包(heartbeat package)
        
        self.run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
        threading.Thread(target=self.move, daemon=True).start() 
        self.run()

    def shutdown(self, signum, frame):
        self.running = False
    
    def enter_func(self, msg):
        rospy.loginfo("enter face detect")
        with self.lock:
            self.run_action_group_srv('sit')
            time.sleep(1)
            self.start_greet = False
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
        rospy.loginfo("exit face detect")
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
                self.run_action_group_srv('up')
            self.enter = False
        return [True, 'exit']

    def set_running(self, msg):
        if msg.data:
            rospy.loginfo("start running face detect")
            with self.lock:
                self.start = True
        else:
            rospy.loginfo("stop running face detect")
            with self.lock:
                self.start = False
                self.start_greet = False

        return [True, 'set_running']
    
    def move(self):    
        while self.running:
            if self.start:
                if self.start_greet:
                    self.start_greet = False
                    self.action_finish = False
                    self.run_action_group_srv('shake_hands1')
                    time.sleep(1)
                    self.action_finish = True
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.1)

    def run(self):
        while self.running:
            if self.enter:
                image = self.image_queue.get(block=True)
                if self.start:
                    self.detect_face = False
                    results = self.model.infer(cv2.resize(image, tuple(self.image_proc_size)))
                    for det in results:
                        bbox = det[0:4].astype(np.int32)
                        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (0, 255, 0), 2)
                        self.detect_face = True
                        # conf = det[-1]
                        # cv2.putText(img, '{:.4f}'.format(conf), (bbox[0], bbox[1]+12), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

                        # landmarks = det[4:14].astype(np.int32).reshape((5, 2))*2
                        # for idx, landmark in enumerate(landmarks):
                            # cv2.circle(img, landmark, 2, self.landmark_color[idx], 2)
                            
                    if self.detect_face:
                        if self.action_finish:
                            self.face_count += 1
                            if self.face_count > 30:
                                self.start_greet = True
                        else:
                            self.face_count = 0
                    else:
                            self.face_count = 0
                self.image_pub.publish(common.cv2_image2ros(image, self.name))
            else:
                time.sleep(0.1)
        rospy.signal_shutdown('shutdown')
    
    def image_callback(self, ros_image):
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将自定义图像消息转化为图像
        cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(cv2_img)
    
if __name__ == '__main__':
    FaceDetectNode('face_detect')
