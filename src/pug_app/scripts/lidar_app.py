#!/usr/bin/env python3
# encoding: utf-8
import math
import time
import rospy
import numpy as np
from threading import RLock
from pug_app.common import Heart
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
from std_srvs.srv import Empty, Trigger, TriggerRequest, TriggerResponse
from pug_control.srv import SetInt64, SetInt64Request, SetInt64Response
from pug_control.srv import SetFloat64List, SetFloat64ListRequest, SetFloat64ListResponse

MAX_SCAN_ANGLE = 360 # 激光的扫描角度,去掉总是被遮挡的部分degree

class LidarController:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.running_mode = 0 
        self.threshold = 0.2 # meters  距离阈值
        self.scan_angle = math.radians(90)  # radians  向前的扫描角度
        self.speed = 0.12 # 单位米，避障模式的速度
        self.timestamp = 0
        self.lock = RLock()
        self.lidar_sub = None
        self.laser_projection = LaserProjection()
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Service('~enter', Trigger, self.enter_func)
        rospy.Service('~exit', Trigger, self.exit_func)
        rospy.Service("~set_running", SetInt64, self.set_running_srv_callback)
        rospy.Service("~set_parameters", SetFloat64List, self.set_parameters_srv_callback)

        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_func(None))  # 心跳包(heartbeat package)
        time.sleep(0.1)
        self.velocity_pub.publish(Twist())
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))

    def reset_value(self):
        with self.lock:
            self.running_mode = 0
            self.threshold = 0.8
            self.speed = 0.12
            self.scan_angle = math.radians(90)

    def enter_func(self, msg):
        rospy.loginfo("lidar enter")
        self.reset_value()
        rospy.ServiceProxy('/pug_control/go_home', Empty)()
        with self.lock:
            self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        return TriggerResponse(success=True)

    def exit_func(self, msg):
        rospy.loginfo('lidar exit')
        with self.lock:
            try:
                if self.lidar_sub is not None:
                    rospy.loginfo('lidar unregister')
                    self.lidar_sub.unregister()
                    self.lidar_sub = None
            except Exception as e:
                rospy.logerr(str(e))
            if isinstance(msg, TriggerRequest):
                self.heart.stop()
                self.velocity_pub.publish(Twist())
        return TriggerResponse(success=True)

    # 设置运行模式
    def set_running_srv_callback(self, req):
        rsp = SetInt64Response(success=True)
        new_running_mode = req.data
        rospy.loginfo("set_running " + str(new_running_mode))
        if not 0 <= new_running_mode <= 3:
            rsp.success = False
            rsp.message = "Invalid running mode {}".format(new_running_mode)
        else:
            self.running_mode = new_running_mode
        self.velocity_pub.publish(Twist())
        return rsp

    # 设置运行参数
    def set_parameters_srv_callback(self, req):
        rsp = SetFloat64ListResponse(success=True)
        new_parameters = req.data
        new_threshold, new_scan_angle, new_speed = new_parameters
        rospy.loginfo("n_t:{:2f}, n_a:{:2f}, n_s:{:2f}".format(new_threshold, new_scan_angle, new_speed))
        if not 0.3 <= new_threshold <= 1.5:
            rsp.success = False
            rsp.message = "New threshold ({:.2f}) is out of range (0.3 ~ 1.5)".format(new_threshold)
            return rsp
        if not new_speed > 0:
            rsp.success = False
            rsp.message = "Invalid speed"
            return rsp

        with self.lock:
            self.threshold = new_threshold
            self.speed = new_speed * 0.002

        return rsp

    def lidar_callback(self, lidar_data):
        cloud = self.laser_projection.projectLaser(lidar_data) # 将 Laserscan 转为点云
        points = np.array(list(pc2.read_points(cloud, skip_nans=True)), dtype=np.float32) # 将点云转为 array

        twist = Twist()
        with self.lock:
            #避障
            if self.running_mode == 1 and self.timestamp <= time.time():
                # 0.2 是机器人的宽度，单位是米。 这里的用途是去除不在机器人正前方的点, 这些点都是不会挡道的
                points = filter(lambda p: abs(p[1]) < 0.2, points)
                # 去除所有到雷达的x轴距离大于避障阈值的点, 这些点暂时还不会挡道
                points = filter(lambda p: p[0] <= self.threshold, points)
                # 去除与x轴夹角大于设置的扫描角度的点，这些点不在设定的扫描区域内。
                points = filter(lambda p: abs(math.atan2(p[1], p[0])) < self.scan_angle / 2, points)
                # 剩下的点就是会挡道的点了
                points = list(points)

                if len(points) > 0: # 有障碍
                    min_x, min_y, min_z, _, _ = min(points, key=lambda p: p[0])
                    twist.linear.x = self.speed / 6
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = -w
                    self.timestamp = time.time() + (max_angle / w) * 0.5
                else:
                    twist.linear.x = self.speed
                    twist.angular.z = 0
                self.velocity_pub.publish(twist)
            # 追踪
            elif self.running_mode == 2 and self.timestamp <= time.time():
                # 只追踪在雷达中心往前偏移10cm的前半球的点, 限制追踪区域避免误触发
                points = list(filter(lambda p: p[0] > 0.1, points))
                # 计算所有点到雷达的距离
                points = map(lambda p: (p[0], p[1], math.sqrt(p[0] * p[0] + p[1] * p[1])), points) 
                
                # 找出距离雷达最近的点
                point_x, point_y, dist = min(points, key=lambda p: p[2])

                # 计算距离最近的点的角度
                angle = math.atan2(point_y, point_x)
                # rospy.loginfo("angle:%.3f",angle)
                # rospy.loginfo("dist:%.3f",dist)
                twist.linear.x = self.speed
                if abs(angle) < self.scan_angle/2:
                    if dist < self.threshold and abs(math.degrees(angle)) > 10: # 控制左右
                        #twist.linear.x = 0.01 # x方向的校正
                        twist.angular.z = self.speed * 3 * np.sign(angle)
                        self.timestamp = time.time() + 0.4
                    else:
                        if self.threshold > dist > 0.25:#0.35
                            twist.linear.x = self.speed
                            twist.angular.z = 0
                            self.timestamp = time.time() + 0.4
                        else:
                            twist.linear.x = 0
                            twist.angular.z = 0
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                self.velocity_pub.publish(twist)
            # 警卫看守
            elif self.running_mode == 3 and self.timestamp <= time.time():
                # 计算所有点到雷达的距离
                points = map(lambda p: (p[0], p[1], math.sqrt(p[0] * p[0] + p[1] * p[1])), points)
                # 过滤掉距离太小的,
                points = filter(lambda p: p[2] > 0.15 , points)
                # 找出距离雷达最近的点
                point_x, point_y, dist = min(points, key=lambda p: p[2])
                # 计算距离最近的点的角度
                angle = math.atan2(point_y,point_x)

                if dist < self.threshold and abs(math.degrees(angle)) > 10: # 控制左右
                    twist.angular.z = self.speed * 3 * np.sign(angle)
                    self.timestamp = time.time() + 0.4
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0    
                self.velocity_pub.publish(twist)
    
if __name__ == "__main__":
    LidarController('lidar_app')
