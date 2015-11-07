#!/usr/bin/env python

import numpy as np
from scipy.interpolate import PiecewisePolynomial

import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import * #(SolvePositionIK, SolvePositionIKRequest)
from baxter_core_msgs.srv import *
from baxter_interface import *
from baxter_pykdl import baxter_kinematics
from cs4752_proj2.srv import *
from tf.transformations import *
from copy import deepcopy

class JointActionServer():
    def __init__(self, limb_name='left'):
        rospy.init_node('joint_action_server')
        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_kinematics(limb_name)
        self.joint_names = self.limb.joint_names()
        self.kp = 0.01
        self.ki = 0.01
        self.kd = 0.0
        self.dt = 0.005
        self.extra_motion_maximum = 0.05
        self.extra_motion_multiple = 2.0
        self.pen_length = 0.165
        self.deriv_step = 1e-5
        self.secondary_objective = False 
        
        self.move_end_effector_trajectory = rospy.Service('move_end_effector_trajectory', JointAction, self.move_end_effector_trajectory)
        loginfo("Initialized /move_end_effector_trajectory")
        self.move_tool_trajectory = rospy.Service('move_tool_trajectory', JointAction, self.move_tool_trajectory)
        loginfo("Initialized /move_tool_trajectory")
        self.position_srv = rospy.Service('end_effector_position', EndEffectorPosition, self.get_position_response)
        loginfo("Initialized /end_effector_position")
        self.velocity_srv = rospy.Service('end_effector_velocity', EndEffectorVelocity, self.get_velocity_response)
        loginfo("Initialized /end_effector_velocity")
        self.tool_position_srv = rospy.Service('tool_position', EndEffectorPosition, self.get_tool_position_response)
        loginfo("Initialized /tool_position")
        self.param_src = rospy.Service('set_parameters', SetParameters, self.parameter_response)
        
        rospy.spin()

    def parameter_response(self, args):
        self.kp = args.kp
        self.ki = args.ki
        self.kd = args.kd
        self.extra_motion_multiple = args.emmult
        self.extra_motion_maximum = args.emmax
        return SetParametersResponse()

    def move_end_effector_trajectory(self, args):
        times, x_positions_velocities, y_positions_velocities, z_positions_velocities = self.unpack_joint_action_message(args)
        loginfo("Unpacked desired events")
        self.move_trajectory(times, x_positions_velocities, y_positions_velocities, z_positions_velocities)
        loginfo("After move_trajectory")
        return JointActionResponse()

    def move_tool_trajectory(self, args):
        times, x_positions_velocities, y_positions_velocities, z_positions_velocities = self.unpack_joint_action_message(args)
        offset = self.get_tool_offset()
        self.move_trajectory(times, x_positions_velocities - offset[0], 
                                    y_positions_velocities - offset[1], 
                                    z_positions_velocities - offset[2])

        return JointActionResponse()

    def move_trajectory(self, times, x_positions_velocities, y_positions_velocities, z_positions_velocities):
 
        # add in derivatives
        spline_order = 3
        xinterpolator = PiecewisePolynomial(times, x_positions_velocities, orders=spline_order, direction=1)
        yinterpolator = PiecewisePolynomial(times, y_positions_velocities, orders=spline_order, direction=1)
        zinterpolator = PiecewisePolynomial(times, z_positions_velocities, orders=spline_order, direction=1)
        
        T = np.arange(0, times[-1], self.dt)
        n = len(T)

        velocity_and_w = np.zeros(6)
        velocity_and_w[0] = xinterpolator.derivative(T[0])
        velocity_and_w[1] = yinterpolator.derivative(T[0])
        velocity_and_w[2] = zinterpolator.derivative(T[0])

        precomputed_positions = np.empty((3,n))
        precomputed_positions[0,:] = xinterpolator(T)
        precomputed_positions[1,:] = yinterpolator(T)
        precomputed_positions[2,:] = zinterpolator(T)

        precomputed_velocities = np.empty((3,n))
        precomputed_velocities[0,:] = xinterpolator.derivative(T)
        precomputed_velocities[1,:] = yinterpolator.derivative(T)
        precomputed_velocities[2,:] = zinterpolator.derivative(T)

        errors = np.empty((3,n))
        errors[:,0] = np.zeros(3)

        vx_corrector, vy_corrector, vz_corrector = 0.0, 0.0, 0.0
        last_vx_corrector, last_vy_corrector, last_vz_corrector = 0.0, 0.0, 0.0
        vx_integral, vy_integral, vz_integral = 0.0, 0.0, 0.0

        for i in xrange(1,n):
            t_start = rospy.get_time()
            self.limb.set_joint_velocities(
                self.make_joint_dict(
                    self.get_joint_velocities(velocity_and_w)))

            time_interval = T[i] - T[i-1]           
 
            position = self.get_position()
            vx_corrector = precomputed_positions[0,i] - position[0]
            vy_corrector = precomputed_positions[1,i] - position[1]
            vz_corrector = precomputed_positions[2,i] - position[2]
            vx_integral += vx_corrector * time_interval
            vy_integral += vy_corrector * time_interval
            vz_integral += vz_corrector * time_interval
            vx_derivative = (vx_corrector - last_vx_corrector)/time_interval
            vy_derivative = (vy_corrector - last_vy_corrector)/time_interval
            vz_derivative = (vz_corrector - last_vz_corrector)/time_interval

            velocity_and_w[0] = precomputed_velocities[0,i] + self.kp * vx_corrector + self.ki * vx_integral - self.kd * vx_derivative
            velocity_and_w[1] = precomputed_velocities[1,i] + self.kp * vy_corrector + self.ki * vy_integral - self.kd * vy_derivative
            velocity_and_w[2] = precomputed_velocities[2,i] + self.kp * vz_corrector + self.ki * vz_integral - self.kd * vz_derivative

            last_vx_corrector = vx_corrector
            last_vy_corrector = vy_corrector
            last_vz_corrector = vz_corrector

            errors[:,i] = np.array([vx_corrector, vy_corrector, vz_corrector])
 
            end_time = time_interval + t_start
            loginfo("Computation Took: {0} out of {1} seconds".format(rospy.get_time() - t_start, time_interval))
            rospy.sleep(end_time - rospy.get_time())

        loginfo("exit_control_mode")
   
        self.limb.exit_control_mode()     
  
        A = np.empty((n,4))
        A[:,0] = T
        A[:,1:] = errors.T
        paramtext = "%1.4f_%1.4f_%1.4f_%1.4f_%1.4f" % (self.kp, self.ki, self.kd, self.extra_motion_maximum, self.extra_motion_multiple)
        np.savetxt("/home/cs4752/ros_ws/src/cs4752_proj2/tests/errors-{0}.csv".format(paramtext),A)
        loginfo("saved errors")

    def get_manipulability(self):
        jacobian = self.limb_kin.jacobian()
        return np.sqrt(np.dot(jacobian,jacobian.T))

    def get_joint_velocities(self, workspace_velocity_and_w):
        J = np.asarray(self.limb_kin.jacobian())
        Jplus = np.linalg.pinv(J)
        if not self.secondary_objective:
            return np.dot(Jplus, workspace_velocity_and_w)
        else:
            rank, nullspace = null(J)
            Jb = np.empty(7)
            for i in xrange(7):
                Jb[i] = nullspace[i,0]
    
            J_squared = np.dot(J, J.T)
            J_squared_inv = np.linalg.inv(J_squared)
            direction_of_manipulability = np.empty(7)
            for i in xrange(0,7):
                angles = self.limb.joint_angles()
    #            loginfo("Current Joint angles: {0}".format(angles))
                angles[self.joint_names[i]] += self.deriv_step
                deltaJ = self.limb_kin.jacobian(joint_values=angles)
                dJ = (deltaJ - J)/self.deriv_step
    #            loginfo("Manipulability Derivatives: {0}".format(dJ))
                dJSquared = np.dot(J,dJ.T) + np.dot(dJ,J.T)
                trace = 0.0
    #            loginfo("J_squared_inv: {0}".format(J_squared_inv))
    #            loginfo("dJSquared: {0}\n{1} rows and {2} columns".format(dJSquared,dJSquared.shape[0],dJSquared.shape[1]))
    #            loginfo("Their Product: {0}".format(np.dot(J_squared_inv, dJSquared)))
                for j in xrange(0,6):
                    for k in xrange(0,6):
                        trace += J_squared_inv[j,k] * dJSquared[k,j]
                direction_of_manipulability[i] = trace
    #        loginfo("Direction of Manipulability: {0}".format(direction_of_manipulability))
            b = np.dot(Jplus, workspace_velocity_and_w)
            mag_b_squared = np.dot(b,b)
    #        loginfo("Pseudoinverted b: {0}".format(b)) 
    #        loginfo("Jb: {0}".format(Jb))
    #        loginfo("Direction of Manipulatbility: {0}".format(direction_of_manipulability))
    #        loginfo("Joint Velocity Limit: {0}".format(0.1 + 5*mag_b_squared))
            loginfo("b: {0}".format(b))
            loginfo("Jb: {0}".format(Jb))
            loginfo("dmu: {0}".format(direction_of_manipulability))
            return b + Jb * maximize_cosine_constrained(Jb, b , direction_of_manipulability , self.extra_motion_multiple*mag_b_squared + self.extra_motion_maximum)

    def make_joint_dict(self, joint_vector):
        joint_dict = {}
        for joint_attribute, joint_name in zip(joint_vector, self.joint_names):
            joint_dict[joint_name] = joint_attribute
        return joint_dict
    
    def unpack_joint_action_message(self, args):
        n = len(args.times)

        times_array = np.empty(n)
        # times_array[0] = 0.0
        # times_array[1:] = args.times
        times_array= args.times
        
        x_positions_velocities = np.empty((n, 2))
        y_positions_velocities = np.empty((n, 2))
        z_positions_velocities = np.empty((n, 2))

        for i in xrange(0,n):
            x_positions_velocities[i,0] = args.positions[i].x
            y_positions_velocities[i,0] = args.positions[i].y
            z_positions_velocities[i,0] = args.positions[i].z

            x_positions_velocities[i,1] = args.velocities[i].x
            y_positions_velocities[i,1] = args.velocities[i].y
            z_positions_velocities[i,1] = args.velocities[i].z

        
        # first_pose = Pose()
        # first_pose.orientation = deepcopy(self.limb.endpoint_pose()['orientation'])        
        # first_pose.position = Point(
        #     args.positions[0].x,
        #     args.positions[0].y,
        #     args.positions[0].z)

        # MoveToPose(first_pose)
        # rospy.sleep(2)

        # current_velocity = self.get_velocity()
        # x_positions_velocities[0,1] = current_velocity[0]
        # y_positions_velocities[0,1] = current_velocity[1]
        # z_positions_velocities[0,1] = current_velocity[2]
        
        
        # current_position = self.get_position()
        # x_positions_velocities[0,0] = current_position[0]
        # y_positions_velocities[0,0] = current_position[1]
        # z_positions_velocities[0,0] = current_position[2]

        return times_array, x_positions_velocities, y_positions_velocities, z_positions_velocities
    
    def get_tool_offset(self):
        off = np.dot(
            quaternion_matrix(self.limb.endpoint_pose()['orientation']),
            np.array([0,0,self.pen_length,1]).T)
        off = off[:3]/off[3]
        return off

    def get_tool_position_response(self, args): 
        position = self.get_position()
        offset = self.get_tool_offset()
        tool_pos = position + offset
        # print "*************************************"
        # print "offset"
        # print offset
        # print "*************************************"
        return EndEffectorPositionResponse(Vector3(tool_pos[0],
                                                   tool_pos[1],
                                                   tool_pos[2]))

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

def null(a, rtol=1e-5):
    # http://stackoverflow.com/questions/19820921/a-simple-matlab-like-way-of-finding-the-null-space-of-a-small-matrix-in-numpy
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol*s[0]).sum()
    return rank, v[rank:].T.copy()

def closest_between_two_lines(a,b,c,d):
    # if \vec{a} s + \vec{b} and \vec{c} t + \vec{d} are lines, this returns the point in the
    #   first line which is closest to the second line
    A = np.array([[np.dot(a,a), -np.dot(c,a)],[-np.dot(c,a), np.dot(c,c)]])
    b = np.array([[- np.dot(a,b) + np.dot(d, a)/2.0],[- np.dot(c,d) + np.dot(c, b)/2]])
    s_optimal = np.linalg.solve(A,b)[0]
    return a * s_optimal + b

def maximize_cosine_constrained(a,b,c,n2):
    # same as maximize_cosine but the result should have norm no greater than n^2
    loginfo(a)
    # already normalized by the norm routine
    aa = 1.0
    loginfo(np.dot(a,a))
    loginfo(b)
    ab = np.dot(a,b)
    loginfo(ab)
    ac = np.dot(a,c)
    bc = np.dot(b,c)
    bb = np.dot(b,b)
    loginfo("Computed Products")
    ab_over_aa = ab/aa
    lower_root = - ab_over_aa - np.sqrt(ab_over_aa**2 + (n2 - bb)/aa)
    upper_root = - ab_over_aa + np.sqrt(ab_over_aa**2 + (n2 - bb)/aa)
    s = (bc*ab - bb*ac)/(ab*ac - aa*bc)
    is_maximum = 2*ab*ac*s*s + (ab*ab*ac + 3*aa*ac*bb)*s + (aa*bc + 2*ab*ac)*bb > 2*bc*aa*aa*s*s + bc*ab*ab
    if is_maximum:
        loginfo("Maximum")
        return np.clip(s, lower_root, upper_root)
    else:
        loginfo("Minimum")
        upper_vec = a * upper_root + b
        lower_vec = a * lower_root + b
        upper_cos = np.dot(upper_vec,c)/np.sqrt(np.dot(upper_vec,upper_vec)*np.dot(c,c))
        lower_cos = np.dot(lower_vec,c)/np.sqrt(np.dot(lower_vec,lower_vec)*np.dot(c,c))
        if upper_cos > lower_cos:
            return upper_root
        else:
            return lower_root

if __name__ == '__main__':
    try: 
        JointActionServer(limb_name='left')
    except rospy.ROSInterruptException:
        pass
