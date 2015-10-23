#!/usr/bin/env python

import numpy as np
from scipy.spatial import KDTree

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

class RobotInterface():
    def __init__(self, limb_name):
        rospy.init_node('robot_interface')
        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_kinematics(limb_name)
        self.joint_names = self.limb.joint_names()
        self.kp = 0.004
        self.dt = 0.005

    def get_joint_velocities(self, workspace_velocity_and_w):
        jacobian_pinv = self.limb_kin.jacobian_pseudo_inverse()
        return np.dot(jacobian_pinv, workspace_velocity_and_w)

    def make_velocity_dict(self, joint_velocity_vector):
        velocity_dict = {}
        for joint_velocity, joint_name in zip(joint_velocity_vector, self.joint_names):
            velocity_dict[joint_name] = joint_velocity
        return velocity_dict

    def moveto_proportional(self, final_position, speed):
       pass 
