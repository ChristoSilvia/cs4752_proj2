#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from scipy.optimize import minimize

import rospy
from geometry_msgs.msg import Vector3
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def test():
    rospy.init_node('test')
    loginfo("Initialized node Controller")

    rospy.wait_for_service("/move_end_effector_trajectory")
    joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
    loginfo("Initialized Joint Action Server Proxy")
    rospy.wait_for_service("/end_effector_position")
    position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    loginfo("Initialized position server proxy")
    parameter_server = rospy.ServiceProxy("/set_parameters", SetParameters)
    loginfo("Initialized parameter server")

    T_max = 10.0
    n_samples = 20
    T = np.linspace(0,T_max,n_samples)[1:]
    times = list(T)

    base_position = position_server().position
    
    def evaluate_parameters(params):
        loginfo(params)
        A = 0.06
        n_tests = 2
        parameter_server(params[0],params[1],params[2],params[3],params[4])
        # Tau = (T/T_max)**2 * (4.0 - 4.0*(T/T_max) + (T/T_max)**2)
        # Tauprime = 2*(T/T_max**2)*(4.0 - 4.0*(T/T_max) + (T/T_max)**2) + (T/T_max)**2 * (-4.0/T_max + 2.0*(T/T_max**2))
        X = A * np.sin(0.5 * (4.0 * np.pi * T/T_max))
        Y = A * np.sin((4.0 * np.pi * T/T_max))
        Xprime = A * np.cos(0.5 * (4.0 * np.pi * T/T_max)) * 0.5 * (4.0 * np.pi / T_max)
        Yprime = A * np.cos(4.0 * np.pi * T/T_max) * 4.0 * np.pi / T_max   
 

        errors = np.empty(n_tests)
        for j in xrange(n_tests):
            joint_action_server([0.0, 2.0], [position_server().position, base_position], [Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)])           
            initial_position = position_server().position
            positions = []
            velocities = []
            for i in xrange(len(T)):
                positions = positions + [Vector3(initial_position.x + X[i], initial_position.y + Y[i], initial_position.z)]
                velocities = velocities + [Vector3(Xprime[i], Yprime[i] , 0.0)]

            rospy.sleep(1.0)          
 
            joint_action_server(times, positions, velocities)
            final_position = position_server().position
            errors[j] = np.sqrt( (initial_position.x - final_position.x)**2 + (initial_position.y - final_position.y)**2 + (initial_position.z - final_position.z)**2)
            loginfo(errors[j])

            joint_action_server([0.0,2.0], [initial_position, base_position], [Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)])
       
        return np.mean(errors)

    guess_params = np.array([0.01, 0.0105, 0.0001, 0.05, 2.0])
    sweet_params = minimize(evaluate_parameters, 
                            np.array([0.01, 0.01, 0.0001, 0.05, 2.0]),
                            method='Nelder-Mead')
    loginfo("resultant parameters: {0}".format(sweet_params))

if __name__ == '__main__':
    test()
