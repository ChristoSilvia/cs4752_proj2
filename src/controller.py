#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial

import rospy
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *

global left, plane_norm, plane_origin, plane_rotation

def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def calibrate_plane() :
    point_count = 0
    point_pos = []
    global left, plane_norm, plane_origin, plane_rotation
    while point_count < 3 :
        prompt = "Press Any Key when Arm is on the %d plane point" % point_count
        cmd = raw_input(prompt)
        point_pos.append(np.array(left.endpoint_pose()['position']))
        # print point_pos[point_count]
        point_count += 1

    vec1 = point_pos[1] - point_pos[0]
    vec2 = point_pos[2] - point_pos[0]
    plane_norm = np.cross(vec1, vec2)
    plane_origin = np.average(point_pos, axis=0)
    # print "Finished Calibrating Plane"
    # print plane_norm
    # print plane_origin

    #need transform to make norm the z vector
    #def rotation_matrix(angle, direction, point=None):
    #Return matrix to rotate about axis defined by point and direction.

    x_plane = vec1/np.linalg.norm(vec1)
    y_plane = np.cross(vec1, plane_norm)
    y_plane = y_plane/np.linalg.norm(y_plane)
    z_plane = plane_norm/np.linalg.norm(plane_norm)
    plane_rotation = np.array([x_plane, y_plane, z_plane])
    # print plane_rotation

def PlaneToBase(plane_x,plane_y) :
    global left, plane_norm, plane_origin, plane_rotation

    translate = [plane_origin[0],plane_origin[1],plane_origin[2]]
    T = numpy.identity(4)
    T[:3, 3] = translate[:3]

    R = np.append(plane_rotation,[[0,0,0]],axis=0)
    R = np.append(R,[[0],[0],[0],[1]],axis=1)
    # print R

    M = np.dot(T, R)

    plane_coords = np.array([plane_x,plane_y,0,1])
    base_coords = np.dot(M, plane_coords.T)
    print "base_coords: {0}".format(base_coords)
    # print base_coords

def controller():
    rospy.init_node('controller')
    loginfo("Initialized node Controller")
    
    baxter_interface.RobotEnable(CHECK_VERSION).enable()

    left = baxter_interface.Limb('left')
    left_kin = baxter_kinematics('left')

    calibrate_plane()
    global plane_rotation
    print plane_rotation

if __name__ == '__main__':
    controller()
