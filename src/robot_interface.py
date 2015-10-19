#!/usr/bin/env python
# liscense removed for brevity

import numpy as np
from scipy.spatial import KDTree

import rospy
from cs4752_proj2.srv import *
import baxter_interface
from baxter_pykdl import baxter_kinematics



# FIXED PARAMETERS
joint_names = ['S0','S1','E0','E1','W0','W1','W2']

joint_limits = np.array([[-2.461, 0.890],
                         [-2.147, 1.047],
                         [-3.028, 3.028],
                         [-0.052, 2.618],
                         [-3.059, 3.059],
                         [-1.571, 2.094],
                         [-3.059, 3.059]])
n_dof = 7
############

# VARIABLE PARAMETERS
kdtree_leafsize = 10
n_points = 100000
limb = 'left'
############

global kdtree


def loginfo(infostring):
    rospy.loginfo("Robot Interface: {0}".format(infostring))

def sample_cspace():
    return joint_limits[:,0] + (joint_limits[:,1] - joint_limits[:,0]) * np.random.rand(7)

def sample_cspace_response(req):
    loginfo("Recieved Sample CSpace Query")
    points = sample_cspace()
    loginfo("Will return: {0}".format(points))
    return SampleCSpaceResponse(points)

def nearest_neighbor_response(req):
    loginfo("Recieved Nearest Neighbor Query")
    distance, nearest_point_index = kdtree.query(np.array(req.point))
    return NearestNeighborResponse(kdtree.data[nearest_point_index,:])



def robot_interface():
    global kdtree
    rospy.init_node('robot_interface')
    loginfo("Initialized Robot Interface")

    loginfo("Beginning to initialize Sampler Service")
    sampler = rospy.Service('sampler', SampleCSpace, sample_cspace_response)
    loginfo("Sampler Service initialized")

    loginfo("Beginning to sample {0} random points".format(n_points))
    input_array = np.empty((n_points, n_dof))
    for i in xrange(0,n_points):
        input_array[i,:] = sample_cspace()
    kdtree = KDTree(input_array, leafsize=kdtree_leafsize)

    loginfo("Beginning to initialize Nearest Neighbor Service")
    nearest = rospy.Service('nearest', NearestNeighbor, nearest_neighbor_response)
    loginfo("Initialized Nearest Neighbor Service")

    left = baxter_interface.Limb('left')
    left_kin = baxter_kinematics('left')

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
