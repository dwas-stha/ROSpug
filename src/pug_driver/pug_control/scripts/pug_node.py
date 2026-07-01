#!/usr/bin/env python3
# encoding: utf-8
import sys
import copy
import rospy
import math, time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, EmptyResponse
from pug_sdk.common import get_yaml_data, save_yaml_data
from pug_control.srv import SetActionName, GetParam, SetParam, SetParamResponse
from pug_control.msg import Velocity, Pose, Gait, WebController, WalkingParam
from ros_robot_controller.msg import SetPose, SetGait, SetJointAngle, Move, Stop, JointAngle
sys.path.append('/home/hiwonder/software/action_editor')
from action_group_controller import ActionGroupController

class PugNode:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.INFO)  # INFO
        self.name = name
        self.pose_status = SetPose()
        self.gait_status = SetGait()
        self.imu_sub = None
        self.last_status = None
        self.last_pose = None
        # self.is_set_pose = False
        self.enable_sim = rospy.get_param('~enable_sim', False)
        self.config = get_yaml_data('/home/hiwonder/pug/src/pug_driver/pug_control/config/walking_param.yaml')
        self.id_to_kinematics = {'1':0, '2':4, '3':8, '4':1, '5':5, '6':9, '7':2, '8':6, '9':10, '10':3, '11':7, '12':11}
        self.set_joint_angle = rospy.Publisher('/ros_robot_controller/robot/set_joint_angle', SetJointAngle, queue_size=10)
        self.set_pose_pub = rospy.Publisher('/ros_robot_controller/robot/set_pose', SetPose, queue_size=10)
        self.set_gait_pub = rospy.Publisher('/ros_robot_controller/robot/set_gait', SetGait, queue_size=10)
        self.move_pub = rospy.Publisher('/ros_robot_controller/robot/move', Move, queue_size=10)
        self.stop_pub = rospy.Publisher('/ros_robot_controller/robot/stop', Stop, queue_size=10)

        rospy.Subscriber('~gait', Gait, self.gait_callback)
        rospy.Subscriber('~pose', Pose, self.pose_callback)
        rospy.Subscriber('~velocity_move', Velocity, self.move_callback)

        rospy.Subscriber('/app_control/gait', Gait, self.app_control_gait_callback)
        rospy.Subscriber('/app_control/pose', Pose, self.app_control_pose_callback)
        rospy.Subscriber('/app_control/velocity_move', Velocity, self.app_control_move_callback)
        rospy.Service('/app_control/go_home', Empty, self.app_control_init_pose_srv)
        rospy.Service('/app_control/get_param', GetParam, self.get_walking_param)
        rospy.Service('/app_control/set_param', SetParam, self.set_walking_param)
        rospy.Service('/app_control/save_param', Empty, self.save_walking_param)
        
        rospy.Subscriber('~web_controller', WebController, self.web_controller_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_limit_callback)
        rospy.Subscriber('~cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Service('~go_home', Empty, self.init_pose_srv)
        rospy.Service('~enable_sim', SetBool, self.enable_sim_srv)
        rospy.Service('~run_action_group', SetActionName, self.run_action_group_srv)

        self.agc = ActionGroupController(self.set_joint_angle, True)
        joint_state_msg = JointState()
        joint_state_msg.header.frame_id = 'robot'
        # rf     lf    rb    lb
        # [1,     2,    3,    4]  calf
        # [5,     6,    7,    8]  thigh
        # [9,     10,   11,   12]  hip  
        joint_state_msg.name = ['rf_joint', 'rf_thigh', 'rf_calf', 'lf_joint', 'lf_thigh', 'lf_calf', 'rb_joint', 'rb_thigh', 'rb_calf', 'lb_joint', 'lb_thigh', 'lb_calf']

        command_topics = ["/rf_joint_position_controller/command",
                          "/rf_thigh_position_controller/command",
                          "/rf_calf_position_controller/command",

                          "/lf_joint_position_controller/command",
                          "/lf_thigh_position_controller/command",
                          "/lf_calf_position_controller/command",

                          "/rb_joint_position_controller/command",
                          "/rb_thigh_position_controller/command",
                          "/rb_calf_position_controller/command",
                          
                          "/lb_joint_position_controller/command",
                          "/lb_thigh_position_controller/command",
                          "/lb_calf_position_controller/command",]

        self.joint_controller_publishers = []
        for i in range(len(command_topics)):
            self.joint_controller_publishers.append(rospy.Publisher('/pug' + command_topics[i], Float64, queue_size=10))

        time.sleep(0.2)
        
        rospy.Subscriber('/ros_robot_controller/robot/get_leg_ik_sim', JointAngle, self.get_robot_leg_ik_sim, queue_size=10)
        self.init_pose_srv(None)
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))
            rospy.loginfo("Shutting down")       

    def get_walking_param(self, msg):
        return WalkingParam(**{key: self.config[key] for key in self.config})

    def set_walking_param(self, msg):
        msg = msg.parameters
        self.config.update({key: getattr(msg, key) for key in self.config if hasattr(msg, key)})
        return SetParamResponse(result=True)

    def save_walking_param(self, msg):
        save_yaml_data(self.config, '/home/hiwonder/pug/src/pug_driver/pug_control/config/walking_param.yaml')
        return EmptyResponse()

    def get_robot_leg_ik_sim(self, msg):
        joint_angle = msg.joint_angle
        for i in range(len(joint_angle)):
            angle = joint_angle[self.id_to_kinematics[str(i + 1)]]
            if i + 1 in [1, 4, 7, 10, 3, 6, 9, 12]:
                if i + 1 in [3, 9]:
                    angle = (angle - 100 - 500) / 1000.0 * math.pi
                elif i + 1 in [6, 12]:
                    angle = (angle + 100 - 500) / 1000.0 * math.pi
                else:
                    angle = (angle - 500) / 1000.0 * math.pi
                if i + 1 in [1, 10]:
                    angle = -angle
            elif i + 1 in [2, 8]:
                angle = (angle - 500) / 1000.0 * math.pi + math.pi / 4
            elif i + 1 in [5, 11]:
                angle = (angle - 500) / 1000.0 * math.pi - math.pi / 4
            self.joint_controller_publishers[i].publish(angle)

    def enable_sim_srv(self, msg):
        self.enable_sim = msg.data
        return [True, 'enable_sim']

    def set_robot_pose(self, x, y, z, x_shift, roll, pitch, duration=0.0, update=True):
        msg = SetPose()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.x_shift = x_shift
        msg.roll = roll
        msg.pitch = pitch
        msg.duration = duration
        self.last_pose = msg
        if update:
            self.pose_status = msg
        self.set_pose_pub.publish(msg)

    def init_pose_srv(self, msg):
        # 初始姿态和步态参数
        msg = SetGait()
        msg.overlap_time = self.config['overlap_time']
        msg.swing_time = self.config['swing_time']  
        msg.clearance_time = self.config['clearance_time']
        msg.z_clearance = self.config['z_clearance']
        self.stop_pub.publish(0)
        self.set_gait_pub.publish(msg)

        self.set_robot_pose(self.config['stance_x'],
                            self.config['stance_y'],
                            self.config['height'],
                            self.config['x_shift'],
                            math.radians(self.config['roll']),
                            math.radians(self.config['pitch']),
                            0.5)

        return []

    def app_control_init_pose_srv(self, msg):
        return self.init_pose_srv(msg)

    def gait_callback(self, msg):
        gait_msg = SetGait()
        gait_msg.z_clearance = msg.z_clearance
        gait_msg.overlap_time = msg.overlap_time
        gait_msg.swing_time = msg.swing_time
        gait_msg.clearance_time = msg.clearance_time
        self.set_gait_pub.publish(gait_msg) 

    def app_control_gait_callback(self, msg):
        self.gait_callback(msg)

    def pose_callback(self, msg):
        self.set_robot_pose(msg.stance_x,
                            msg.stance_y,
                            msg.height,
                            msg.x_shift,
                            msg.roll,
                            msg.pitch,
                            msg.run_time
                            )

    def app_control_pose_callback(self, msg):
        self.pose_callback(msg)

    def cmd_vel_limit_callback(self, msg):
        if msg.linear.x > 0.1: 
            msg.linear.x = 0.1
        elif msg.linear.x < -0.1:
            msg.linear.x = -0.1

        if msg.linear.y > 0.1:
            msg.linear.y = 0.1
        elif msg.linear.y < -0.1:
            msg.linear.y = -0.1
        
        if msg.angular.z > 0.4:
            msg.angular.z = 0.4
        elif msg.angular.z < -0.4:
            msg.angular.z = -0.4

        self.cmd_vel_callback(msg)

    def cmd_vel_callback(self, msg):
        move_msg = Velocity()
        move_msg.x = msg.linear.x
        move_msg.y = msg.linear.y
        move_msg.yaw_rate = msg.angular.z
        move_msg.stop = True
        self.move_callback(move_msg)

    def move_callback(self, msg):
        if abs(msg.x) < 1e-6 and abs(msg.y) < 1e-6 and abs(msg.yaw_rate) < 1e-6 and not msg.stop:
            self.move_pub.publish(0, 0, 0)
        elif abs(msg.x) > 1e-6 or abs(msg.y) > 1e-6 or abs(msg.yaw_rate) > 1e-6:
            self.move_pub.publish(msg.x, msg.y, msg.yaw_rate)
        elif msg.stop:
            self.stop_pub.publish(0)

    def app_control_move_callback(self, msg):
        if abs(msg.y) > 1e-6:
            msg.y = math.copysign(0.05, msg.y)
        if abs(msg.yaw_rate) > 1e-6:
            msg.yaw_rate = math.copysign(0.35, msg.yaw_rate)
        if abs(msg.x) > 1e-6 and abs(msg.y) < 1e-6 and abs(msg.yaw_rate) < 1e-6: # 前进和后退加参数进行调整
            if self.last_status is not None:
                if msg.x * self.last_status.x < -1e-6 or abs(self.last_status.x) < 1e-6: # 只有方向改变时才设置一次参数
                    pose_msg = copy.deepcopy(self.pose_status)
                    if msg.x < -1e-6:
                        pose_msg.x_shift += self.config['back_x_shift']
                    elif msg.x > 1e-6:
                        pose_msg.x_shift += self.config['forward_x_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
            else:
                pose_msg = copy.deepcopy(self.pose_status)
                if msg.x < -1e-6:
                    pose_msg.x_shift += self.config['back_x_shift']
                elif msg.x > 1e-6:
                    pose_msg.x_shift += self.config['forward_x_shift']
                self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
            if msg.x > 1e-6: # 前进后退偏移时加参数校正
                msg.yaw_rate += self.config['forward_offset']
            else:
                msg.yaw_rate += self.config['back_offset']
        else:
            if self.last_pose is not None:
                if abs(msg.x) < 1e-6 and abs(msg.y) < 1e-6 and msg.yaw_rate > 1e-6:  # 原地转弯加参数校正
                    pose_msg = copy.deepcopy(self.pose_status)
                    pose_msg.x_shift += self.config['turn_left_x_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
                if abs(msg.x) < 1e-6 and abs(msg.y) < 1e-6 and msg.yaw_rate < -1e-6:  # 原地转弯加参数校正
                    pose_msg = copy.deepcopy(self.pose_status)
                    pose_msg.x_shift += self.config['turn_right_x_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
                elif msg.x > 1e-6 and abs(msg.y) < 1e-6 and abs(msg.yaw_rate) > 1e-6: # 前进转弯加参数校正
                    pose_msg = copy.deepcopy(self.pose_status)
                    pose_msg.x_shift += self.config['forward_turn_x_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
                elif msg.x < -1e-6 and abs(msg.y) < 1e-6 and abs(msg.yaw_rate) > 1e-6: # 后退转弯加参数校正
                    pose_msg = copy.deepcopy(self.pose_status)
                    pose_msg.x_shift += self.config['back_turn_x_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
                elif abs(msg.x) < 1e-6 and msg.y < -1e-6 and abs(msg.yaw_rate) < 1e-6: # 右平移加参数校正
                    pose_msg = copy.deepcopy(self.pose_status)
                    pose_msg.x_shift += self.config['move_right_x_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
                elif abs(msg.x) < 1e-6 and msg.y > 1e-6 and abs(msg.yaw_rate) < 1e-6: # 左平移加参数校正
                    pose_msg = copy.deepcopy(self.pose_status)
                    pose_msg.x_shift += self.config['move_left_shift']
                    self.set_robot_pose(pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.x_shift, pose_msg.roll, pose_msg.pitch, update=False)
        self.last_status = msg
        self.move_callback(msg)

    def web_controller_callback(self, msg):
        gait_msg = Gait()
        gait_msg.z_clearance = msg.foot_height
        # 一个步态完整的周期 = 2*overlap_time + 2*swing_time + 2*clearance_time
        if msg.gait == 1: # Trot
            # clearance_time固定为0。这里设定overlap_time = swing_time
            gait_msg.overlap_time = msg.period * 0.5
            gait_msg.swing_time = msg.period * 0.45
            gait_msg.clearance_time = msg.period * 0.05
        elif msg.gait == 2: # Amble
            # 这里设定clearance_time为swing_time的一半，同时overlap_time = swing_time
            gait_msg.overlap_time = msg.period * 0.5
            gait_msg.swing_time = msg.period * 0.4
            gait_msg.clearance_time = msg.period * 0.1
        elif msg.gait == 3: # Walk
            # 这里设定clearance_time = swing_time，同时overlap_time = swing_time
            gait_msg.overlap_time = msg.period * 0.6
            gait_msg.swing_time = msg.period * 0.3
            gait_msg.clearance_time = msg.period * 0.1
        
        self.gait_callback(gait_msg)
        move_msg = Velocity()
        move_msg.x = msg.x
        move_msg.y = msg.y
        move_msg.yaw_rate = msg.yaw_rate
        move_msg.stop = msg.stop
        self.move_callback(move_msg)

    def run_action_group_srv(self, msg):
        # rospy.loginfo(msg)
        self.agc.runAction(msg.name)
        return [True, 'run_action_group']

if __name__ == '__main__':
    PugNode('pug_control')
