#!/usr/bin/env python

import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial

import rospy
from geometry_msgs.msg import Vector3
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

        self.joint_action_srv = rospy.Service('joint_action', JointAction, self.move_trajectory)
        self.position_srv = rospy.Service('end_effector_position', EndEffectorPosition, self.get_position_response)
        self.velocity_srv = rospy.Service('end_effector_velocity', EndEffectorVelocity, self.get_velocity_response)

        rospy.spin()

    def move_trajectory(self, args):
        times, x_positions_velocities, y_positions_velocities, z_positions_velocities = self.unpack_joint_action_message(args)
        
        # add in derivatices
        spline_order = 3
        xinterpolator = PiecewisePolynomial(times, x_positions_velocities, orders=spline_order, direction=1)
        yinterpolator = PiecewisePolynomial(times, y_positions_velocities, orders=spline_order, direction=1)
        zinterpolator = PiecewisePolynomial(times, z_positions_velocities, orders=spline_order, direction=1)
        
        T = np.arange(0, times[-1], dt)
        n = length(T)
        velocity_and_w = np.zeros((6,1))
        velocity_and_w[0] = xinterpolator.derivative(times[0])
        velocity_and_w[1] = yinterpolator.derivative(times[0])
        velocity_and_w[2] = zinterpolator.derivative(times[0])
        for i in xrange(1,n):
            t_start = rospy.get_time()
            self.limb.set_joint_velocities(
                self.make_velocity_dict(
                    self.get_joint_velocities(velocity_and_w)))
            
            velocity_and_w[0] = xinterpolator.derivative(times[i])
            velocity_and_w[1] = yinterpolator.derivative(times[i])
            velocity_and_w[2] = zinterpolator.derivative(times[i])
                     
    
            desired_interval = T[i] - T[i-1]
            end_time = T[i] - T[i-1] + t_start
            rospy.sleep(end_time - rospy.get_time()) 
   
        self.limb.exit_control_mode()     
        return JointActionResponse()

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
        
	x_positions_velocities = np.empty((len(args.positions)+1, 2))
	y_positions_velocities = np.empty((len(args.positions)+1, 2))
	z_positions_velocities = np.empty((len(args.positions)+1, 2))

	for i in xrange(0,n):
            x_positions_velocities[i,0] = args.positions[i].x
            y_positions_velocities[i,0] = args.positions[i].y
            z_positions_velocities[i,0] = args.positions[i].z

            x_positions_velocities[i,1] = args.velocities[i].x
            y_positions_velocities[i,1] = args.velocities[i].y
            z_positions_velocities[i,1] = args.velocities[i].z

	current_velocity = self.get_velocity()
        x_positions_velocities[0,0] = current_velocity[0]
        y_positions_velocities[0,0] = current_velocity[1]
        z_positions_velocities[0,0] = current_velocity[2]
        
        current_position = self.get_position()
        x_positions_velocities[0,1] = current_position[0]
        y_positions_velocities[0,1] = current_position[1]
        z_positions_velocities[0,1] = current_position[2]

        return times_array, x_positions_velocities, y_positions_velocities, z_positions_velocities

    def get_position_response(self, args):
        loginfo("Recieved position request")
        loginfo("Arguments: {0}".format(args)) 
        position = self.get_position()
        return EndEffectorPositionResponse(Vector3(positon[0],position[1],position[2]))

    def get_velocity_response(self, args):
        velocity = self.get_velocity()
        return EndEffectorVelocityResponse(Vector3(velocity[0], velocity[1], velocity[2]))

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
