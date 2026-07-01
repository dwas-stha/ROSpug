#!/usr/bin/env python3
import os
import time
import copy
import rospy
import pygame as pg
from enum import Enum
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from pug_sdk.common import get_yaml_data
from pug_control.msg import Velocity, Pose
from ros_robot_controller.msg import SetGait
from ros_robot_controller.msg import BuzzerState

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickControlNode:
    def __init__(self, name):
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
        pg.display.init()
        self.last_stamp = time.time()
        self.name = name
        self.status = 0
        self.debug = False
        rospy.init_node(self.name, log_level=rospy.INFO)
        self.last_axes =  dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.config = get_yaml_data('/home/hiwonder/pug/src/pug_driver/pug_control/config/walking_param.yaml')
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.set_gait_pub = rospy.Publisher('/ros_robot_controller/robot/set_gait', SetGait, queue_size=1)
        self.velocity_pub = rospy.Publisher('/app_control/velocity_move', Velocity, queue_size=1)
        self.pose_pub = rospy.Publisher('/app_control/pose', Pose, queue_size=1)
        rospy.Subscriber('joy', Joy, self.joy_callback) 
        
        self.init()
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))       

    def init(self):
        self.velocity_msg = Velocity()
        self.pose_msg = Pose()
        self.pose_msg.stance_x = self.config['stance_x']
        self.pose_msg.stance_y = self.config['stance_y']
        self.pose_msg.x_shift = self.config['x_shift']
        self.pose_msg.height = self.config['height']
        self.pose_msg.pitch = self.config['pitch']
        self.pose_msg.roll = self.config['roll']

        self.last_velocity_msg = copy.deepcopy(self.velocity_msg)
        self.last_pose_msg = copy.deepcopy(self.pose_msg)

    def axes_callback(self, axes):
        if axes['ly'] > 0.5:
            self.velocity_msg.x = 0.13
        elif axes['ly'] < -0.5:
            self.velocity_msg.x = -0.13
        elif abs(axes['ly']) < 1e-6:
            self.velocity_msg.x = 0
            self.velocity_msg.stop = 1
        if axes['lx'] > 0.5:
            self.velocity_msg.y = 0.08
        elif axes['lx'] < -0.5:
            self.velocity_msg.y = -0.08
        elif abs(axes['lx']) < 1e-6:
            self.velocity_msg.y = 0
            self.velocity_msg.stop = 1
        if axes['rx'] > 0.5:
            self.velocity_msg.yaw_rate = 0.5
        elif axes['rx'] < -0.5:
            self.velocity_msg.yaw_rate = -0.5
        elif abs(axes['rx']) < 1e-6:
            self.velocity_msg.yaw_rate = 0
            self.velocity_msg.stop = 1
        # print(axes['ry'])
        # if axes['ry'] > 0.5:
            # if self.pose_msg.height < -0.1:
                # self.pose_msg.height += 0.002
        # elif axes['ry'] < -0.5:
            # if self.pose_msg.height > -0.15:
                # self.pose_msg.height -= 0.002
        if self.velocity_msg.__getstate__() != self.last_velocity_msg.__getstate__():
            # print(self.velocity_msg)
            self.velocity_pub.publish(self.velocity_msg)
        # if self.pose_msg.__getstate__() != self.last_pose_msg.__getstate__():
            # print(self.pose_msg.x_shift)
            # self.pose_pub.publish(self.pose_msg)
        self.last_velocity_msg = copy.deepcopy(self.velocity_msg)
        # print(axes)

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            if self.pose_msg.pitch < 0.35:
                self.pose_msg.pitch += 0.02

    def l2_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            if self.pose_msg.pitch > -0.35:
                self.pose_msg.pitch -= 0.02

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            if self.pose_msg.roll > -0.17:
                self.pose_msg.roll -= 0.02

    def r2_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            if self.pose_msg.roll < 0.17:
                self.pose_msg.roll += 0.02
        
    def cross_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            if self.pose_msg.height < -0.1:
                self.pose_msg.height += 0.002

    def triangle_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            if self.pose_msg.height > -0.15:
                self.pose_msg.height -= 0.002

    def circle_callback(self, new_state):
        if not self.debug:
            if new_state == ButtonState.Pressed:
                self.velocity_msg.y = -0.08
            elif new_state == ButtonState.Released:
                self.velocity_msg.y = 0
                self.velocity_msg.stop = 1
        else:
            if new_state == ButtonState.Pressed:
                if self.pose_msg.x_shift < 0.03:
                    self.pose_msg.x_shift += 0.001
                    print(self.pose_msg.x_shift)
                self.buzzer_pub.publish(1900, 0.1, 0.01, 1)
                time.sleep(0.1)

    def square_callback(self, new_state):
        if not self.debug:
            if new_state == ButtonState.Pressed:
                self.velocity_msg.y = 0.08
            elif new_state == ButtonState.Released:
                self.velocity_msg.y = 0
                self.velocity_msg.stop = 1
        else:
            if new_state == ButtonState.Pressed:
                if self.pose_msg.x_shift > -0.03:
                    self.pose_msg.x_shift -= 0.001
                    print(self.pose_msg.x_shift)
                self.buzzer_pub.publish(1900, 0.1, 0.01, 1)
                time.sleep(0.1)

    def hat_yu_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.velocity_msg.x = 0.13
        elif new_state == ButtonState.Released:
            self.velocity_msg.x = 0
            self.velocity_msg.stop = 1

    def hat_yd_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.velocity_msg.x = -0.13
        elif new_state == ButtonState.Released:
            self.velocity_msg.x = 0
            self.velocity_msg.stop = 1

    def hat_xl_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.velocity_msg.yaw_rate = 0.5
        elif new_state == ButtonState.Released:
            self.velocity_msg.yaw_rate = 0
            self.velocity_msg.stop = 1

    def hat_xr_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.velocity_msg.yaw_rate = -0.5
        elif new_state == ButtonState.Released:
            self.velocity_msg.yaw_rate = 0
            self.velocity_msg.stop = 1

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # print('start')
            self.buzzer_pub.publish(1900, 0.1, 0.01, 1)
            self.init()
            rospy.ServiceProxy('/pug_control/go_home', Empty)()

    def select_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.debug = not self.debug
            self.buzzer_pub.publish(1900, 0.05, 0.01, 2)
        # if new_state == ButtonState.Pressed:
            # print('select')
            # self.pose_msg.x_shift = -0
            # self.pose_msg.height = -0.09
            # self.pose_pub.publish(self.pose_msg)
            # time.sleep(1)
            # self.pose_msg.height = -0.18
            # self.pose_pub.publish(self.pose_msg)
            # time.sleep(0.08)
            # self.pose_msg.height = -0.1
            # self.pose_pub.publish(self.pose_msg)

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        for key, value in axes.items(): # 轴的值被改变
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            # print(key)
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                # rospy.loginfo(key + ': ' + str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                        if self.pose_msg.__getstate__() != self.last_pose_msg.__getstate__():
                            # print(self.pose_msg.x_shift)
                            self.pose_pub.publish(self.pose_msg)
                        if self.velocity_msg.__getstate__() != self.last_velocity_msg.__getstate__():
                            # print(self.velocity_msg)
                            self.velocity_pub.publish(self.velocity_msg)
                        self.last_pose_msg = copy.deepcopy(self.pose_msg)
                        self.last_velocity_msg = copy.deepcopy(self.velocity_msg)
                    except Exception as e:
                        rospy.logerr(str(e))
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    JoystickControlNode('joystick_control')

