#!/usr/bin/env python
# liscense removed for brevity

import numpy as np
from scipy.spatial import KDTree

import rospy
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *



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

plane_norm = np.array([0,0,0])
plane_origin = np.array([0,0,0])
plane_rotation = np.empty([3,3])

############

# VARIABLE PARAMETERS
kdtree_leafsize = 10
n_points = 100000
limb = 'left'
############

global kdtree

def calibrate_plane() :
    point_count = 0
    point_pos = []
    global left, plane_norm, plane_origin, plane_rotation
    while point_count < 3 :
        input("Press Any Key when Arm is on the %d plane point" %point_count)
        point_pos.append(np.array(left.endpoint_pose()['position']))
        print point_pos[point_count]
        point_count += 1

    vec1 = point_pos[1] - point_pos[0]
    vec2 = point_pos[2] - point_pos[0]
    plane_norm = np.cross(vec1, vec2)
    plane_origin = numpy.mean(point_pos)
    print "Finished Calibrating Plane"
    print plane_norm
    print plane_origin

    #need transform to make norm the z vector
    #def rotation_matrix(angle, direction, point=None):
    #Return matrix to rotate about axis defined by point and direction.

    x_plane = vec1/np.linalg.norm(vec1)
    y_plane = np.cross(vec1, plane_norm)
    y_plane = y_plane/np.linalg.norm(y_plane)
    z_plane = plane_norm/np.linalg.norm(plane_norm)
    plane_rotation = np.array([x_plane, y_plane, z_plane])
    print plane_rotation

#def BaseToPlane() :


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

def null(A):
    eps = 1e-4
    u, s, vh = np.linalg.svd(A, full_matrices=1, compute_uv = 1)
    null_space = np.compress(s < eps, vh, axis=0)
    return null_space.T

def get_joint_velocities(workspace_velocity):
    global left_kin
    jacobian_pinv = left_kin.jacobian_pseudo_inverse()
    loginfo(null(jacobian_pinv))
    return np.dot(jacobian_pinv, workspace_velocity)

def moveto(final_position, speed):
    global left
    loginfo("Recieved Move Order")
    dt = 0.05
    dx = 0.0001
    velocities = {}
    current_position = np.array(left.endpoint_pose()['position'])
    left.set_command_timeout(3*dt)
    
    while np.linalg.norm(current_position - final_position) < dx:
        t_start = rospy.get_time()
        loginfo("Starting new movement loop")
        total_distance = np.linalg.norm(final_position - current_position)
    	velocity = speed * (final_position - current_position) / total_distance
        velocity_and_angular_momentum = np.zeros((6,1))
        velocity_and_angular_momentum[0:3,0] = velocity
        joint_velocities = get_joint_velocities(velocity_and_angular_momentum)
        for i, name in enumerate(joint_names):
            velocities[name] = joint_velocities[i,0]
        left.set_joint_velocities(velocities)
        rospy.sleep(dt - (rospy.get_time() - t_start))
    left.exit_control_mode()

def moveto_proportional(final_position, speed):
    global left
#    loginfo("Recieved Move Order")
    dt = 0.005
    kp = 0.004*speed
    velocities = {}
    initial_position = np.array(left.endpoint_pose()['position'])
#    loginfo("Current Position: {0}".format(initial_position))
    total_distance = np.linalg.norm(initial_position - final_position)
#    loginfo("Distance to Target: {0}".format(total_distance))
    total_time = total_distance / speed
#    loginfo("Expected Time at Desired Speed: {0}".format(total_time))
    velocity = speed * (final_position - initial_position) / total_distance
#    loginfo("Expected End Effector Velocity")
    left.set_command_timeout(3*dt)

    for t in np.arange(0, total_time, dt):
        t_start = rospy.get_time()
        # Compute difference between where we should be and
	#   where we are
	current_position = np.array(left.endpoint_pose()['position'])
        desired_position = initial_position + (t/total_time)*(final_position - initial_position)
        current_error = current_position - desired_position
#        loginfo("Current position: {0}".format(current_position))
#        loginfo("Desired position: {0}".format(desired_position))
#        loginfo("Current error: {0}".format(current_error))

        # Subtract kp times from the current velocity.
        velocity -= kp * current_error
  
        # prepare velocity and angular momentum vector
        velocity_and_angular_momentum = np.zeros((6,1))
        velocity_and_angular_momentum[0:3,0] = velocity
 
        # solve pseudoinverse
        joint_velocities = get_joint_velocities(velocity_and_angular_momentum)

        # assemble joint velocities dict and execute command
        for i, name in enumerate(joint_names):
            velocities[name] = joint_velocities[i,0]
        left.set_joint_velocities(velocities)

        # wait until next timestep
        extra_time =dt - (rospy.get_time() - t_start)
#        loginfo("{0} sec left over".format(extra_time))
        rospy.sleep(extra_time)
    left.exit_control_mode()

def moveto_blind(final_position, speed):
    global left
    loginfo("Recieved Move Order")
    dt = 0.005
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
        extra_time =dt - (rospy.get_time() - t_start)
        loginfo("{0} sec left over".format(extra_time))
        rospy.sleep(extra_time)

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

    calibrate_plane()

    n_iterations = 10
    errors = np.empty(n_iterations)
    initial_pose = np.array(left.endpoint_pose()['position'])
    loginfo("Beginning Error Evaluation")
    for i in xrange(0,n_iterations):
        desired_pose = initial_pose + (0.3*np.random.rand(3) - 0.15)
        moveto_proportional(desired_pose, 0.05)
        rospy.sleep(0.1)
        error = desired_pose - left.endpoint_pose()['position']
        loginfo("Error: {0}".format(np.linalg.norm(error)))
        errors[i] = np.linalg.norm(error)
    loginfo("Mean Error: {0}".format(np.mean(errors)))

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
