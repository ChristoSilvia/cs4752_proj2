#!/usr/bin/env python

import numpy as np
from scipy.spatial import KDTree

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from cs4752_proj2.srv import *

class JointActionServer():
    def __init__(self, limb_name='left'):
        rospy.init_node('joint_action_server')
        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_kinematics(limb_name)
        self.joint_names = self.limb.joint_names()
        self.kp = 0.004
        self.dt = 0.005

        self.joint_action_srv = rospy.Service('joint_action_server', JointAction, self.move_trajectory)

        rospy.spin()

    def move_trajectory_blind(self, args):
        times, positions, velocities = self.unpack_joint_action_message(args)
        
        # add in derivatices
        spline_order = 3
        interpolator = PiecewisePolynomial(trajectory_times, full_trajectory, order=spline_order, direction=1)
        
        T = np.arange(0, total_time, dt)
        n = length(T)
        for i in xrange(0,n-1):
            t_start = rospy.get_time()
            
            velocity_and_w = np.zeros((6,1))
            velocity_and_w[0:2] = interpolator.derivative(T[i])
            self.limb.set_joint_velocities(
                self.make_velocity_dict(
                    self.get_joint_velocities(velocity_and_w)))
    
            extra_time = (T[i+1]-T[i]) - (rospy.get_time() - t_start)
            rospy.sleep(extra_time) 
        
        return JointActionResponse()
    
    def move_position_trajectory_blind(self, trajectory, total_time):
        """Trajectory is a three-row n-column array of
    	workspace positions"""
        full_trajectory = np.empty(3, length(trajectory)+1)
        full_trajectory[:,0] = self.get_position()
        full_trajectory[:,1:] = trajectory

        space_differences = np.linalg.norm(full_trajectory[:,1:] - full_trajectory[:,:-1], axis=0)
        trajectory_times = total_time * (space_differences / np.sum(space_differences))
        dt = 0.05
        T = np.arange(0, total_time, dt)

        spline_order = 3
        interpolator = PiecewisePolynomial(trajectory_times, full_trajectory, order=spline_order, direction=1)
        
        n = length(T)
        for i in xrange(0,n-1):
            t_start = rospy.get_time()
            
            velocity_and_w = np.zeros((6,1))
            velocity_and_w[0:2] = interpolator.derivative(T[i])
            self.limb.set_joint_velocities(
                self.make_velocity_dict(
                    self.get_joint_velocities(velocity_and_w)))
    
            extra_time = (T[i+1]-T[i]) - (rospy.get_time() - t_start)
            rospy.sleep(extra_time) 

    def get_joint_velocities(self, workspace_velocity_and_w):
        jacobian_pinv = self.limb_kin.jacobian_pseudo_inverse()
        return np.dot(jacobian_pinv, workspace_velocity_and_w)

    def make_velocity_dict(self, joint_velocity_vector):
        velocity_dict = {}
        for joint_velocity, joint_name in zip(joint_velocity_vector, self.joint_names):
            velocity_dict[joint_name] = joint_velocity
        return velocity_dict
    
    def unpack_joint_action_message(self, args):
        n = len(args.times)

        times_array = np.empty(n+1)
        times_array[0] = 0.0
        times_array[1:] = args.times

        positions_array = np.empty((3,len(args.positions)+1))
        positions_array[:,0] = self.get_position()

        for i, position in enumerate(args.positions):
            positions_array[0,i+1] = position.x
            positions_array[1,i+1] = position.y
            positions_array[2,i+1] = position.z

        velocities_array = np.empty((3,len(args.velocities)+1)
        velocities_array[:,0] = self.get_velocity()

        for i, velocity in enumerate(args.velocities):
            velocities_array[0,i+1] = velocity.x
            velocities_array[1,i+1] = velocity.y
            velocities_array[2,i+1] = velocity.z

        return times_array, positions_array, velocities_array

    def get_position(self):
        return np.array(self.limb.endpoint_pose()['position']) 

    def get_velocity(self):
        return np.array(self.limb.endpoint_velocity()['linear'])

def loginfo(message):
    rospy.loginfo("Joint Action Server: {0}".format(message))

if __name__ == '__main__':
    try: 
        JointActionServer(limb_name='left')
    except rospy.ROSInterruptException:
        pass
