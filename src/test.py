#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial

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

    rospy.wait_for_service("/joint_action")
    joint_action_server = rospy.ServiceProxy("/joint_action", JointAction)
    rospy.wait_for_service("/end_effector_position")
    position_server = rospy.ServiceProxy("/end_effector_position", JointAction)

    current_position = position_server()
    joint_action_server([1.0], 
                        [Vector3(current_position.x, current_position.y + 0.1, current_position.z + 0.1)],    
                        [Vector3(0.0,0.0,0.0)])

if __name__ == '__main__':
    test()
