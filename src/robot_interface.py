#!/usr/bin/env python
# liscense removed for brevity

import numpy as np
from scipy.spatial import KDTree

import rospy
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics



# FIXED PARAMETERS
joint_names = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']

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

def get_joint_velocities(workspace_velocity):
    global left_kin
    jacobian_pinv = left_kin.jacobian_pseudo_inverse()
    return np.dot(jacobian_pinv, workspace_velocity)

def moveto(final_position, speed):
    global left
    loginfo("Recieved Move Order")
    dt = 0.05
    velocities = {}
    current_position = np.array(left.endpoint_pose()['position'])
    loginfo("Current Position: {0}".format(current_position))
    total_distance = np.linalg.norm(current_position - final_position)
    loginfo("Distance to Target: {0}".format(total_distance))
    total_time = total_distance / speed
    loginfo("Expected Time at Desired Speed: {0}".format(total_time))
    velocity = speed * (final_position - current_position) / total_distance
    loginfo("Expected End Effector Velocity")
    left.set_command_timeout(3*dt)
    for t in np.arange(0, total_time, dt):
        t_start = rospy.get_time()
        velocity_and_angular_momentum = np.zeros((6,1))
        velocity_and_angular_momentum[0:3,0] = velocity
        joint_velocities = get_joint_velocities(velocity_and_angular_momentum)
        for i, name in enumerate(joint_names):
            velocities[name] = joint_velocities[i,0]
        left.set_joint_velocities(velocities)
        rospy.sleep(dt - (rospy.get_time() - t_start))

def robot_interface():
    global kdtree
    global left
    global left_kin
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

    baxter_interface.RobotEnable(CHECK_VERSION).enable()

    left = baxter_interface.Limb('left')
    left_kin = baxter_kinematics('left')
#    velocities = {}
#    zero_velocity = {}
#    for joint_name in joint_names:
#        zero_velocity[joint_name] = 0.0
#    loginfo("Starting to move arm")
#    left.set_command_timeout(0.5)
##    for i in xrange(0,200):
##        c_space_velocities = get_joint_velocities(np.array([[0],[0],[0.01],[0],[0],[0]]))
##        for j, joint_name in enumerate(joint_names):
##            velocities[joint_name] = c_space_velocities[j,0]
##        left.set_joint_velocities(velocities)
##        rospy.sleep(0.1)
#    for i in xrange(0,200):
#        c_space_velocities = get_joint_velocities(np.array([[0],[0],[0.0],[0.1],[0.0],[0]]))
#        for j, joint_name in enumerate(joint_names):
#            velocities[joint_name] = c_space_velocities[j,0]
#        left.set_joint_velocities(velocities)
#        rospy.sleep(0.1)
#    left.exit_control_mode()

    current_pose = np.array(left.endpoint_pose()['position'])
    loginfo("Current pose: {0}".format(current_pose))
    desired_pose = current_pose + np.array([0,0,0.2])
    loginfo("Desired Post: {0}".format(desired_pose))
    moveto(desired_pose, 0.05)

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
