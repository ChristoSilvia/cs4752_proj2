#!/usr/bin/env python

import numpy as np
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
        self.kp = 0.05
        self.ki = 0.01
        self.dt = 0.01
        self.pen_length = 0.08
        self.deriv_step = 1e-5
        
        self.move_end_effector_trajectory = rospy.Service('move_end_effector_trajectory', JointAction, self.move_end_effector_trajectory)
        self.move_tool_trajectory = rospy.Service('move_tool_trajectory', JointAction, self.move_tool_trajectory)
        self.position_srv = rospy.Service('end_effector_position', EndEffectorPosition, self.get_position_response)
        self.velocity_srv = rospy.Service('end_effector_velocity', EndEffectorVelocity, self.get_velocity_response)
        self.tool_position_srv = rospy.Service('tool_position', EndEffectorPosition, self.get_tool_position_response)

        rospy.spin()

    def move_end_effector_trajectory(self, args):
        times, x_positions_velocities, y_positions_velocities, z_positions_velocities = self.unpack_joint_action_message(args)
        self.move_trajectory(times, x_positions_velocities, y_positions_velocities, z_positions_velocities)
        return JointActionResponse()

    def move_end_effector_trajectory(self, args):
        times, x_positions_velocities, y_positions_velocities, z_positions_velocities = self.unpack_joint_action_message(args)
        offset = self.get_tool_offset()
        self.move_trajectory(times, x_positions_velocities + offset[0], 
                                    y_positions_velocities + offset[1], 
                                    z_positions_velocities + offset[2])

        return JointActionResponse()

    def move_trajectory(self, times, x_positions_velocities, y_positions_velocities, z_positions_velocities)
 
        # add in derivatives
        spline_order = 3
        xinterpolator = PiecewisePolynomial(times, x_positions_velocities, orders=spline_order, direction=1)
        yinterpolator = PiecewisePolynomial(times, y_positions_velocities, orders=spline_order, direction=1)
        zinterpolator = PiecewisePolynomial(times, z_positions_velocities, orders=spline_order, direction=1)
        
        T = np.arange(0, times[-1], self.dt)
        n = len(T)
        velocity_and_w = np.zeros((6,1))
        velocity_and_w[0] = xinterpolator.derivative(T[0])
        velocity_and_w[1] = yinterpolator.derivative(T[0])
        velocity_and_w[2] = zinterpolator.derivative(T[0])
        vx_corrector, vy_corrector, vz_corrector = 0.0, 0.0, 0.0
        vx_integral, vy_integral, vz_integral = 0.0, 0.0, 0.0
        for i in xrange(1,n):
            t_start = rospy.get_time()
            self.limb.set_joint_velocities(
                self.make_joint_dict(
                    self.get_joint_velocities(velocity_and_w)))
            
            position = self.get_position()
            vx_corrector = xinterpolator(T[i]) - position[0]
            vy_corrector = yinterpolator(T[i]) - position[1]
            vz_corrector = zinterpolator(T[i]) - position[2]
            vx_integral += vx_corrector * (T[i] - T[i-1])
            vy_integral += vy_corrector * (T[i] - T[i-1])
            vz_integral += vz_corrector * (T[i] - T[i-1])

            velocity_and_w[0] = xinterpolator.derivative(T[i]) + self.kp * vx_corrector + self.ki * vx_integral
            velocity_and_w[1] = yinterpolator.derivative(T[i]) + self.kp * vy_corrector + self.ki * vy_integral
            velocity_and_w[2] = zinterpolator.derivative(T[i]) + self.kp * vz_corrector + self.ki * vz_integral
            loginfo("Computation Took: {0}".format(rospy.get_time() - t_start))
 
            desired_interval = T[i] - T[i-1]
            end_time = T[i] - T[i-1] + t_start
            rospy.sleep(end_time - rospy.get_time())
   
        self.limb.exit_control_mode()     

    def get_manipulability(self):
        jacobian = self.limb_kin.jacobian()
        return np.sqrt(np.dot(jacobian,jacobian.T))

    def get_joint_velocities(self, workspace_velocity_and_w):
        jacobian_pinv = self.limb_kin.jacobian_pseudo_inverse()
        jacobian = self.limb_kin.jacobian()
        
        rank, nullspace = null(jacobian)
        assert(rank == 6, "Jacobian is Singular")
        Jb = nullspace[:,0]

        J_squared = np.dot(J, J.T)
        J_squared_inv = np.linalg.inv(J_squared)

        direction_of_manipulatibility = np.empty((7,1))
        for i in xrange(0,7):
            angles = self.limb.get_joint_angles()
            angles[self.joint_names[i]] += self.deriv_step
            deltaJ = self.limb_kin.jacobian(joint_values=angles)
            dJ = (deltaJ - jacobian)/self.deriv_step
            dJSquared = np.dot(J,dJ.T) + np.dot(dJ,J.T)
            trace = 0.0
            for j in xrange(0,7):
                trace += np.dot(J_squared_inv[i,:], dJsquared[:,i])
            direction_of_manipulability[i] = trace
            
        return closest_between_two_lines(Jb, 
                                         np.dot(jacobian_pinv, workspace_velocity_and_w),
                                         direction_of_manipulability,
                                         0)

    def make_joint_dict(self, joint_vector):
        joint_dict = {}
        for joint_attribute, joint_name in zip(joint_vector, self.joint_names):
            joint_dict[joint_name] = joint_attribute
        return joint_dict
    
    def unpack_joint_action_message(self, args):
        n = len(args.times)

        times_array = np.empty(n+1)
        times_array[0] = 0.0
        times_array[1:] = args.times
        
        x_positions_velocities = np.empty((n+1, 2))
        y_positions_velocities = np.empty((n+1, 2))
        z_positions_velocities = np.empty((n+1, 2))

        for i in xrange(0,n):
            x_positions_velocities[i+1,0] = args.positions[i].x
            y_positions_velocities[i+1,0] = args.positions[i].y
            z_positions_velocities[i+1,0] = args.positions[i].z

            x_positions_velocities[i+1,1] = args.velocities[i].x
            y_positions_velocities[i+1,1] = args.velocities[i].y
            z_positions_velocities[i+1,1] = args.velocities[i].z

        current_velocity = self.get_velocity()
        x_positions_velocities[0,1] = current_velocity[0]
        y_positions_velocities[0,1] = current_velocity[1]
        z_positions_velocities[0,1] = current_velocity[2]
        
        current_position = self.get_position()
        x_positions_velocities[0,0] = current_position[0]
        y_positions_velocities[0,0] = current_position[1]
        z_positions_velocities[0,0] = current_position[2]

        return times_array, x_positions_velocities, y_positions_velocities, z_positions_velocities
    
    def get_tool_offset(self):
        np.dot(
            quaternion_to_rotation(self.limb.endpoint_pose()['orientation']),
            np.array([0,0,self.pen_length]))

    def get_tool_position_response(self, args): 
        position = self.get_position()
        offset = self.get_tool_offset()
        return EndEffectorPositionResponse(Vector3(position[0] + offset[0],
                                                   position[1] + offset[1],
                                                   position[2] + offset[2]))

    def get_position_response(self, args):
        position = self.get_position()
        return EndEffectorPositionResponse(Vector3(position[0],position[1],position[2]))

    def get_velocity_response(self, args):
        velocity = self.get_velocity()
        return EndEffectorVelocityResponse(Vector3(velocity[0], velocity[1], velocity[2]))

    def get_position(self):
        return np.array(self.limb.endpoint_pose()['position']) 

    def get_velocity(self):
        return np.array(self.limb.endpoint_velocity()['linear'])

def loginfo(message):
    rospy.loginfo("Joint Action Server: {0}".format(message))

def quaternion_to_rotation(q):
    # from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
    return np.array([[1 - 2*q.y**2 - 2*q.z**2, 2*q.x*q.y - 2*q.z*q.w, 2*q.x*q.z + 2*q.y*q.w],
                     [2*q.x*q.y + 2*q.z*q.w, 1 - 2*q.x**2 - 2*q.z**2, 2*q.y*q.z - 2*q.x*q.w],
                     [2*q.x*.q.z + 2*q.y*q.w, 2*q.y*q.z + 2*q.x*q.w, 1 - 2*q.x**2 - 2*q.y**2]])

def null(a, rtol=1e-5):
    # http://stackoverflow.com/questions/19820921/a-simple-matlab-like-way-of-finding-the-null-space-of-a-small-matrix-in-numpy
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol*s[0]).sum()
    return rank, v[rank:].T.copy()

def closest_between_two_lines(a,b,c,d)
    # if \vec{a} s + \vec{b} and \vec{c} t + \vec{d} are lines, this returns the point in the
    #   first line which is closest to the second line
    A = np.array([[np.dot(a,a), -np.dot(c,a)],[-np.dot(c,a), np.dot(c,c)])
    b = np.array([[- np.dot(a,b) + np.dot(d, a)/2.0],[- np.dot(c,d) + np.dot(c, b)/2]])
    s_optimal = np.linalg.solve(A,b)[0]
    return a * s_optimal + b

if __name__ == '__main__':
    try: 
        JointActionServer(limb_name='left')
    except rospy.ROSInterruptException:
        pass
