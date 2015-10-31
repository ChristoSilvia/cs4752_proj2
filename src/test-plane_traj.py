#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial

import pylab as plt

import rospy
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *
from geometry_msgs.msg import Vector3
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *


def loginfo(logstring):
    rospy.loginfo("test-plane_traj: {0}".format(logstring))

def add_to_plane_traj_msg(P,V,t):
    """ P = position, V = velocitiy, A = acceleration t = time (sec)"""
    global plane_traj_msg, scale

    P = np.array(P).T*scale
    V = np.array(V).T*scale
    # A = np.array(A).T*scale
    r, c = P.shape

    # print "##################################"
    # print "type(P):"
    # print type(P)
    # print type(P)
    # print P.shape
    # print V.shape
    # print A.shape
    # print "##################################"

    for ti in t: plane_traj_msg.times.append(ti)
    for i in range(0,r):
        plane_traj_msg.positions.append( Vector3( P[i,0], P[i,1], 0 ) )
        plane_traj_msg.velocities.append( Vector3( V[i,0], V[i,1], 0 ) )
        # plane_traj_msg.accelerations.append( Vector3( A[i,0], A[i,1], 0 ) )

def send_plane_traj():
    global plane_traj_pub, plane_traj_msg, time
    time = 0
    plane_traj_msg.reference_frame = "/plane_frame"

    print "##################################"
    print "plane_traj_msg:"
    print "plane_traj_msg.reference_frame: %s" % plane_traj_msg.reference_frame
    print "len(times): %d" % len(plane_traj_msg.times)
    print "len(positions): %d" % len(plane_traj_msg.positions)
    print "len(velocities): %d" % len(plane_traj_msg.velocities)
    # print "len(accelerations): %d" % len(plane_traj_msg.accelerations)
    print "duration: %f" % plane_traj_msg.times[len(plane_traj_msg.times)-1]
    print "##################################"

    plane_traj_pub.publish(plane_traj_msg)
    plane_traj_msg = Trajectory()
    
def test_plane_traj():
    rospy.init_node("test_plane_traj")
    loginfo("Initialized node test_plane_traj")

    global fig, ax, plane_traj_pub
    
    #Generate the figure
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.hold(True)

    # rospy.Subscriber("/cmd_path", Path, pathCb, queue_size=10000)   
    plane_traj_pub = rospy.Publisher('/plane_traj', Trajectory, queue_size=10)

    plt.show()
    # rospy.spin()

if __name__ == "__main__":
    test_plane_traj()
