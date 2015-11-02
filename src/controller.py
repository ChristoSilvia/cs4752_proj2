#!/usr/bin/env python
import rospy
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *
from geometry_msgs.msg import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *

global left, plane_norm, plane_rotation

def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def calibrate_plane():
    global left, plane_norm, plane_rotation, plane_translation

    point_count = 0
    point_pos = []
    while point_count < 3 :
        prompt = "Press Enter when Arm is on the %d plane point" % point_count
        cmd = raw_input(prompt)
        point_pos.append(np.array(left.endpoint_pose()['position']))
        # print point_pos[point_count]
        point_count += 1

    vec1 = point_pos[1] - point_pos[0]
    vec2 = point_pos[2] - point_pos[0]
    plane_norm = np.cross(vec1, vec2)
    plane_origin = np.average(point_pos, axis=0)
    # plane_origin = point_pos[0]
    plane_translation = np.identity(4)
    plane_translation[:3, 3] = plane_origin[:3]
    
    x_plane = vec1/np.linalg.norm(vec1)
    y_plane = np.cross(vec1, plane_norm)
    y_plane = y_plane/np.linalg.norm(y_plane)
    z_plane = plane_norm/np.linalg.norm(plane_norm)
    #need rotation to make norm the z vector
    plane_rotation = np.array([x_plane, y_plane, z_plane])
    plane_rotation = np.append(plane_rotation,[[0,0,0]],axis=0)
    plane_rotation = np.append(plane_rotation,[[0],[0],[0],[1]],axis=1)
    
    print "#################################"
    print "Finished Calibrating Plane"
    print "plane_translation"
    print plane_translation
    print "plane_rotation"
    print plane_rotation
    print "#################################"

def matrix_from_euler(r,p,y,radians=False):
    if not radians:
        a = math.pi/180.
        r *= a
        p *= a
        y *= a

    q = quaternion_from_euler(r, p, y)
    return quaternion_matrix(q)

def PlaneToBasePoint(plane_x,plane_y):
    global plane_rotation, plane_translation

    try:
        plane_translation
        plane_rotation
    except NameError:
        plane_translation = np.identity(4)
        plane_rotation = np.identity(4)
        # plane_rotation = matrix_from_euler(-45,0,0)

    M = np.dot(plane_translation, plane_rotation)

    plane_coords = np.array([plane_x,plane_y,0,1])
    base_coords = np.dot(M, plane_coords.T)
    base_coords = base_coords[:3]/base_coords[3]
    base_coords.reshape((1, 3))
    # print "base_coords: {0}".format(base_coords)
    return base_coords

def PlaneToBaseDir(plane_x,plane_y):
    global plane_rotation

    plane_dir = np.array([plane_x,plane_y,0,1])
    base_dir = np.dot(plane_rotation, plane_dir.T)
    base_dir = base_dir[:3]/base_dir[3]
    base_dir.reshape((1, 3))
    # print "base_dir: {0}".format(base_dir)
    return base_dir

def plane_trajCb(plane_traj_msg):
    global joint_action_server, position_server

    T = np.array(plane_traj_msg.times)
    P = np.zeros([0,3])
    V = np.zeros([0,3])

    positions = []
    velocities = []
    for i in range(0,len(plane_traj_msg.positions)):
        pp = plane_traj_msg.positions[i]
        wp = PlaneToBasePoint(pp.x,pp.y)
        P = np.append(P, [wp], axis=0)
        positions.append(Vector3(wp[0],wp[1],wp[2]))

        pv = plane_traj_msg.velocities[i]
        wv = PlaneToBaseDir(pv.x,pv.y)
        V = np.append(V, [wv], axis=0)
        velocities.append(Vector3(wv[0],wv[1],wv[2]))

    # print "#################################"
    # print "plane_traj_msg"
    # print "len(times): %d" % len(plane_traj_msg.times)
    # print "len(positions): %d" % len(positions)
    # print "len(velocities): %d" % len(velocities)
    # print "shapes:"
    # print T.shape
    # print P.shape
    # print V.shape
    # print "################ T #################"
    # print T
    # print "################ P #################"
    # print P
    # print "################ V #################"
    # print V
    # print "#################################"

    loginfo("Making position call")
    initial_position = position_server().position
    loginfo(initial_position)

    joint_action_server(plane_traj_msg.times, positions, velocities) 


def controller():
    global joint_action_server, left, left_kin, tool_trajectory, position_server
    rospy.init_node('controller')
    loginfo("Initialized node Controller")
    
    baxter_interface.RobotEnable(CHECK_VERSION).enable()

    left = baxter_interface.Limb('left')
    left_kin = baxter_kinematics('left')

    rospy.wait_for_service("/move_end_effector_trajectory")
    joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
    loginfo("Initialized Joint Action Server Proxy")
    rospy.wait_for_service("/end_effector_position")
    position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    loginfo("Initialized position server proxy")

    calibrate_plane()

    rospy.Subscriber("/plane_traj", Trajectory, plane_trajCb, queue_size=10000)

    rospy.spin()

if __name__ == '__main__':
    controller()
