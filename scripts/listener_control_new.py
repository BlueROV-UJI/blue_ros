#!/usr/bin/env python
from doctest import master
import math
from os import kill
import string

import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
# from waterlinked_a50_ros_driver.msg import DVL
# from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
# from autonomous_rov.msg import pwm
from alpha_beta_gamma_filter import alpha_beta_gamma_filter
from PI_Controller import*
# from brping import Ping1D
import time
import sys
import argparse


# ---------- Global Variables ---------------

set_mode = [0]*3
# Vmax_mot = 1900
# Vmin_mot = 1100

Vmax_mot = 1600
Vmin_mot = 1400

# Conditions
arming = False
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

# visual servoing
global desired_points_vs
global vcam_vs
global lambda_vs
global n_points_vs

vcam_vs = np.array([0, 0, 0, 0, 0, 0])
lambda_vs = np.array([0.5,
                      0.5,
                      0.5,
                      0.25,
                      0.5,
                      0.1])  # yaw
n_points_vs = 8
desired_points_vs = []
enable_vs = 0

roll_left_right = 1500
yaw_left_right = 1500
ascend_descend = 1500
forward_reverse = 1500
lateral_left_right = 1500
pitch_left_right = 1500


def armDisarm(armed):
    # This functions sends a long command service with 400 code to arm or disarm motors
    if (armed):
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Arming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except arming")
    else:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Disarming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except disarming")

def map_vel_yaw(value, b=None):
    global Vmax_mot
    global Vmin_mot
    if (value >= 0):
        m = 400   # Slope of the positive force linear function
        b = 1536 if b is None else b
    else:
        m = 400  # Slope of the negtaive force linear function
        b = 1464 if b is None else b
        
    pulse_width = value * m + b

    # On limite la commande en vitesse
    if pulse_width > Vmax_mot:
        pulse_width = Vmax_mot
    if pulse_width < Vmin_mot:
        pulse_width = Vmin_mot
    
    return pulse_width
    

def mapValueScalSat(value):
    global Vmax_mot
    global Vmin_mot
    # Correction_Vel and joy between -1 et 1
    # scaling for publishing with setOverrideRCIN values between 1100 and 1900
    # neutral point is 1500
    pulse_width = value * 400 + 1500

    # On limite la commande en vitesse
    if pulse_width > Vmax_mot:
        pulse_width = Vmax_mot
    if pulse_width < Vmin_mot:
        pulse_width = Vmin_mot

    return pulse_width


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
    # This function replaces setservo for motor commands.
    # It overrides Rc channels inputs and simulates motor controls.
    # In this case, each channel manages a group of motors not individually as servo set

    msg = OverrideRCIn()
    msg.channels = [channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral,
                    1700, 1500, 1100, 1100, 0, 0, 0, 0, 0, 0, 0, 0]
    msg.channels = list(map(np.uint, msg.channels))


    pub_msg_override.publish(msg)


def DoThing(msg):
    print(msg.data)
    setOverrideRCIN(1500, 1500, 1500, 1500, msg.data, 1500)


class Master:
    def __init__(self):
        
        self.surge_vel_setpoint_pub = rospy.Publisher(
            "controller/surge_vel/desired", Float64, queue_size=10)
        self.depth_setpoint_pub = rospy.Publisher("controller/depth/desired",Float64,queue_size=10)
        self.yaw_setpoint_pub = rospy.Publisher("controller/yaw/desired",Float64,queue_size=10)
        self.surge_setpoint_pub = rospy.Publisher("controller/surge/desired",Float64,queue_size=10)
        self.sway_setpoint_pub = rospy.Publisher("controller/sway/desired", Float64, queue_size=10)
        self.pitch_setpoint_pub = rospy.Publisher(
            "controller/pitch/desired", Float64, queue_size=10)


        # Surge Mode (vel from dvl vs Pos from pinger)
        self.use_surge_vel = False
        self.drowned = False
        
        # PWMs
        self.surge_vel_pwm = 1500
        self.surge_pwm = 1500
        self.depth_pwm = 1500
        self.sway_pwm = 1500
        self.yaw_pwm = 1500
        
        # Mission Thresholds
        self.free_dist_thresh = 1000
        self.max_depth_err = 0.1
        self.max_wall_dist_err = 100
        
        # Setpoints
        self.depth_desired = -0.6 #0.3
        self.yaw_desired = np.pi/2
        self.pitch_desired = 0.0
        self.surge_desired = 700
        self.surge_vel_desired = 0.0
        self.surge_vel_nominal = 0.1
        self.sway_desired = 0.0
        
        # Initial Values of Control Variables
        self.surge = 0
        self.surge_vel = 0
        self.yaw = 0
        self.pitch = 0
        self.sway = 0
        self.depth = 0
        
        # self.state_names = ["drown", "go_to_wall", "keep_to_wall", "search"]
        # self.state_nums = {name:val for val, name in self.state_names.items()}
        self.actions = {"vs": self.vs_action}
        
        self.state = "vs"
        # self.state = "drown_vs"
        # self.state = "yaw"
        
        rospy.sleep(2)
        
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                            surge=self.surge_desired, sway=self.sway_desired, pitch=self.pitch_desired)
        
        # rospy.Subscriber("controller/surge_vel/effort",
        #                  Float64MultiArray, self.surge_vel_cb)
        # rospy.Subscriber("controller/surge/effort",
        #                  Float64MultiArray, self.surge_cb)
        # rospy.Subscriber("controller/yaw/effort",
        #                  Float64MultiArray, self.yaw_cb)
        # rospy.Subscriber("controller/pitch/effort",
        #                  Float64MultiArray, self.pitch_cb)
        rospy.Subscriber("controller/depth/effort",
                         Float64MultiArray, self.depth_cb)
        # rospy.Subscriber("controller/sway/effort",
        #                  Float64MultiArray, self.sway_cb)
        rospy.Subscriber("/aruco_control", Twist, self.aruco_cb, queue_size=1)

    def aruco_cb(self, msg):
        global ascend_descend
        global lateral_left_right
        global forward_reverse
        global pitch_left_right
        global roll_left_right
        global yaw_left_right
        lateral_left_right = self.PWM_Cmd(msg.linear.y, b=1500)
        # ascend_descend = self.PWM_Cmd(msg.linear.z, b=1500)
        forward_reverse = self.PWM_Cmd(msg.linear.x, b=1500)
        yaw_left_right = self.PWM_Cmd(msg.angular.z, b=1500)
        ascend_descend = 1500 # not controlled
        # forward_reverse = 1500 # not controlled
        pitch_left_right = 1500 # imu controlled
        roll_left_right = 1500 # not controlled
        # yaw_left_right = 1500 # imu controlled
        self.update_state()

    def surge_cb(self, msg):
        self.surge_pwm = self.PWM_Cmd(msg.data[0], b=1500)
        self.surge = msg.data[1]
        self.update_state()
    
    def surge_vel_cb(self, msg):
        self.surge_vel_pwm = self.PWM_Cmd(msg.data[0])
        self.surge_vel = msg.data[1]
        self.update_state()

    def yaw_cb(self, msg):
        self.yaw_pwm = self.PWM_Cmd(msg.data[0])
        self.yaw = msg.data[1]
        print("yaw_cb")
        self.update_state()
    
    def pitch_cb(self,msg):
        # global pitch_left_right
        self.pitch_pwm = self.PWM_Cmd(msg.data[0], b=1500)
        self.pitch = msg.data[1]
        # pitch_left_right = self.pitch_pwm
        self.update_state()

    def depth_cb(self, msg):
        # global ascend_descend
        print(self.depth_pwm)
        self.depth_pwm = self.PWM_Cmd(msg.data[0])
        # ascend_descend = self.depth_pwm
        self.depth = msg.data[1]
        self.update_state()

    def sway_cb(self, msg):
        self.sway_pwm = self.PWM_Cmd(msg.data[0])
        self.sway = msg.data[1]
        self.update_state()

    # Function used to calculate the necessary PWM for each motor

    def PWM_Cmd(self, thrust_req, b = None):
        if (thrust_req >= 0):
            m = 86.93393326839376   # Slope of the positive force linear function
            b = 1536 if b is None else b
        else:
            m = 110.918185437553874  # Slope of the negtaive force linear function
            b = 1464 if b is None else b

        PWM = int(m * thrust_req/4) + b
        if PWM > Vmax_mot:
            PWM = Vmax_mot
        if PWM < Vmin_mot:
            PWM = Vmin_mot
        return PWM

    def send_setpoints(self, depth, yaw, surge, sway,pitch=0, surge_vel=0):
        # self.surge_vel_setpoint_pub.publish(Float64(surge_vel))
        self.depth_setpoint_pub.publish(Float64(depth))
        # self.yaw_setpoint_pub.publish(Float64(yaw))
        # self.surge_setpoint_pub.publish(Float64(surge))
        # self.sway_setpoint_pub.publish(Float64(sway))
        # self.pitch_setpoint_pub.publish(Float64(pitch))
    
    def update_state(self):
        if set_mode[2] or True:
            self.actions[self.state]()
            # print(f"state = {self.state}")
        if abs(abs(self.depth) - self.depth_desired) > self.max_depth_err and not self.drowned:
            # print("drowning")
            # self.state = "drown"
            # self.state = "drown_vs"
            self.state = "vs"
            # self.state = "yaw"
            return
        else:
            self.state = "vs"
        # elif self.state == "drown_vs":
        #     self.state = "vs"
        #     # self.drowned = True
        #     return

        # elif self.state == "drown":
        #     # if abs(abs(self.depth) - self.depth_desired) <= self.max_depth_err:
        #     self.state = "go_to_wall"
        #     return
        
        # elif self.state == "go_to_wall":
        #     print(f"cond = {abs(self.surge - self.surge_desired)}")
        #     if abs(self.surge - self.surge_desired) < self.max_wall_dist_err:
        #         self.state = "keep_to_wall"
        #     return
        
        # elif self.state == "keep_to_wall":
        #     # self.state = "search"
        #     self.state = "bonus"
        #     return
        
        # elif self.state == "search":
        #     if self.surge >= self.free_dist_thresh:
        #         self.state = "go_to_wall"
        #     return
        # elif self.state == "bonus":
        #     self.state = "go_to_wall"
        #     return
        # elif self.state == "yaw":
        #     return    

    # def drown_action(self):
    #     # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
    #     setOverrideRCIN(1500, 1500, self.depth_pwm,
    #                     self.yaw_pwm, 1500, self.sway_pwm)
    
    # def drown_vs_action(self):
    #     # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
    #     # print(self.depth_pwm)
    #     setOverrideRCIN(1500, 1500, self.depth_pwm,
    #                     1500, 1500, 1500)
    
    def vs_action(self):
        global roll_left_right
        global yaw_left_right
        global ascend_descend
        global forward_reverse
        global lateral_left_right
        global pitch_left_right
        # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
        # setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
        #                 yaw_left_right, forward_reverse, lateral_left_right)
        ascend_descend = self.depth_pwm
        print(f"forward = {forward_reverse}")
        print(f"lateral = {lateral_left_right}")
        print(f"yaw = {yaw_left_right}")
        setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
                        yaw_left_right, forward_reverse, lateral_left_right)
    
    # def yaw_action(self):
    #     # Correct sway, yaw, and depth, but not surge distance, keep rov in a vertical column
    #     setOverrideRCIN(1500, 1500, self.depth_pwm,
    #                     self.yaw_pwm, 1500, 1500)
    #     # setOverrideRCIN(1500, 1500, 1500,
    #     #                 self.yaw_pwm, 1500, 1500)


def subscriber():
    # rospy.Subscriber("joy", Joy, joyCallback)
    # rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("do/thing", Int16, DoThing)
    #camera
    # rospy.Subscriber("/aruco_control",Twist,aruco_callback, queue_size=1)
    # rospy.Subscriber("desired_points",Float64MultiArray,desiredpointscallback, queue_size=1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    armDisarm(True)  # Not automatically disarmed at startup
    rospy.init_node('autonomous_MIR', anonymous=False)
        
    pub_msg_override = rospy.Publisher(
        "mavros/rc/override", OverrideRCIn, queue_size=10, tcp_nodelay=True)
    pub_angle_degre = rospy.Publisher(
        'angle_degree', Twist, queue_size=10, tcp_nodelay=True)
    pub_depth = rospy.Publisher(
        'depth/state', Float64, queue_size=10, tcp_nodelay=True)
    pub_angular_velocity = rospy.Publisher(
        'angular_velocity', Twist, queue_size=10, tcp_nodelay=True)
    pub_linear_vel = rospy.Publisher(
        'linear_velocity', Twist, queue_size=10, tcp_nodelay=True)
    
    master = Master()

    subscriber()
