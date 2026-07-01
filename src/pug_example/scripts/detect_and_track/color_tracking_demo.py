#!/usr/bin/env python3
# encoding: utf-8
import cv2
import math
import time
import rospy
import numpy as np
from pug_sdk import pid, common
from pug_control.msg import Pose
from sensor_msgs.msg import Image
from pug_control.srv import SetActionName

size = (320, 180)
target_color = ''
color_range = None
pose_pub = None

y_dis = 0
y_pid = pid.PID(P=0.0003, I=0, D=0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'purple': (255, 0, 255),
}

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓

def run(img):
    global y_dis

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    area_max = 0
    area_max_contour = 0
    
    if target_color in color_range:
        target_color_range = color_range[target_color]
        frame_mask = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 对原图像和掩模进行位运算
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        area_max_contour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

    if area_max > 100:  # 有找到最大面积
        (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)  # 获取最小外接圆
        center_x = int(common.val_map(center_x, 0, size[0], 0, img_w))
        center_y = int(common.val_map(center_y, 0, size[1], 0, img_h))
        radius = int(common.val_map(radius, 0, size[0], 0, img_w))
        if radius > 100:
            return img
        cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[target_color], 2)
        cv2.circle(img, (int(center_x), int(center_y)), 5, range_rgb[target_color], -1)
        
        y_pid.SetPoint = img_h / 2.0
        y_pid.update(center_y)
        y_dis += y_pid.output

        y_dis = np.radians(20) if y_dis > np.radians(20) else y_dis
        y_dis = np.radians(-20) if y_dis < np.radians(-20) else y_dis
        pose_pub.publish(0, y_dis, 0, -0.13, 0, 0, 0, 0)

    return img

def image_callback(ros_image):
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame_result = run(cv2_img)
    cv2.imshow('frame_result', frame_result)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('color_tracking_demo', log_level=rospy.INFO)
    pose_pub = rospy.Publisher('/pug_control/pose', Pose, queue_size=1)
    rospy.Subscriber('/csi_camera/image_rect_color', Image, image_callback)
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})
    run_action_group_srv = rospy.ServiceProxy('/pug_control/run_action_group', SetActionName)
    time.sleep(0.1)
    
    run_action_group_srv('stand')
    target_color = 'purple'
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
