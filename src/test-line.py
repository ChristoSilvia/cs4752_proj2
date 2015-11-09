#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from scipy.optimize import minimize

import matplotlib.pyplot as plt

import rospy
from config import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import *
from baxter_core_msgs.msg import * #(SolvePositionIK, SolvePositionIKRequest)
from baxter_core_msgs.srv import *
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *

MOVE_WAIT = 0.1
limb = "left"


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def test():
    rospy.init_node('test')
    loginfo("Initialized node Controller")

    global move_robot
    rospy.wait_for_service("/move_robot")
    move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
    loginfo("Initialized service proxy for /move_robot")

    rospy.wait_for_service("/move_end_effector_trajectory")
    joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    
    rospy.wait_for_service("/end_effector_position")
    position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    loginfo("Initialized position server proxy")

    rospy.wait_for_service("/set_parameters")
    parameter_server = rospy.ServiceProxy("/set_parameters", SetParameters)
    loginfo("Initialized parameter server")

    def evaluate_parameters(params):
        loginfo(params)
        L = 0.3
        T_max = 4.0
        v = L/T_max
        parameter_server(params[0],params[1],params[2],params[3],params[4])
 
        HomePose()
        position = position_server().position
        joint_action_server([0, T_max], [position, Vector3(position.x, position.y + L, position.z)], [Vector3(0,v,0),Vector3(0,v,0)])
        HomePose()

    T_eq = 0.9
    K_eq = 6.0
    #K_p = 0.6 * K_eq
    #K_i = 2.0 * K_eq / T_eq
    #K_d = 0.125 * K_eq * T_eq
    K_p = 1.5
    K_i = 0.72
    K_d = -0.0054

    guess_params = np.array([K_p, K_i, K_d, 0.05, 1.5])
    evaluate_parameters(guess_params)
    # sweet_params = minimize(evaluate_parameters, 
    #                        np.array([0.01, 0.01, 0.0001, 0.05, 2.0]),
    #                        method='Nelder-Mead')
    #loginfo("resultant parameters: {0}".format(sweet_params))

def HomePose() :
    rospy.loginfo("Going to Home Pose")
    homepose = Pose()
    homepose.position = Point(0.572578886689,0.081184911298,0.146191403844)
    homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
    # success = MoveToPose(homepose, False, False, False)
    global move_robot
    success = move_robot(MOVE_TO_POSE, "left", homepose)
    rospy.loginfo("Got to Home Pose : %r", success)

if __name__ == '__main__':
    test()
