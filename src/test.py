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

    rospy.wait_for_service("/joint_action_server")
    joint_action_server = rospy.ServiceProxy("/joint_action_server", JointAction)

    joint_action_server([0.0], 
                        [Vector3(0.0,0.0,0.0)],    
                        [Vector3(0.0,0.0,0.0)])

if __name__ == '__main__':
    test()
