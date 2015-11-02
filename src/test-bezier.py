#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial


import rospy
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *
from geometry_msgs.msg import Vector3
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def test():
    # global position_server
    rospy.init_node('Controller')
    loginfo("Initialized node Controller")

    # global joint_action_server
    # rospy.wait_for_service("/move_end_effector_trajectory")
    # joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    # tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
    # loginfo("Initialized Joint Action Server Proxy")
    # rospy.wait_for_service("/end_effector_position")
    # position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    # loginfo("Initialized position server proxy")

    rospy.Subscriber("/plane_traj", Trajectory, plane_trajCb)
#    rospy.sleep(5.0)

    rospy.spin()

def plane_trajCb(plane_traj_msg):
    # global position_server
    # global joint_action_server
    # loginfo("Making position call")
    # initial_position = position_server().position
    # loginfo(initial_position)
    
    T = np.array(plane_traj_msg.times)
    P = np.zeros([0,3])
    V = np.zeros([0,3])
    for i in range(0,len(plane_traj_msg.positions)):
        pv = plane_traj_msg.positions[i]
        P = np.append(P, [[pv.x,pv.y,pv.z]], axis=0)
        vv = plane_traj_msg.velocities[i]
        V = np.append(V, [[vv.x,vv.y,vv.z]], axis=0)

    print "#################################"
    print "plane_traj_msg"
    print "len(times): %d" % len(plane_traj_msg.times)
    print "shapes:"
    print T.shape
    print P.shape
    print V.shape
    print "#################################"

    

# float64[] times
# geometry_msgs/Vector3[] positions
# geometry_msgs/Vector3[] velocities



    # A = 0.06

    # T_max = 10.0
    # n_samples = 20
    # T = np.linspace(0,T_max,n_samples)[1:]
    # # Tau = (T/T_max)**2 * (4.0 - 4.0*(T/T_max) + (T/T_max)**2)
    # # Tauprime = 2*(T/T_max**2)*(4.0 - 4.0*(T/T_max) + (T/T_max)**2) + (T/T_max)**2 * (-4.0/T_max + 2.0*(T/T_max**2))
    # X = A * (1.0 - np.cos(2*np.pi*T/T_max))
    # Y = A * np.sin(2*np.pi*T/T_max)
    # Xprime = A*np.sin(2*np.pi*T/T_max)*2*np.pi/T_max
    # Yprime = A*np.cos(2*np.pi*T/T_max)*2*np.pi/T_max

    # times = list(T)
    # positions = []
    # velocities = []
    # for i in xrange(len(T)):
    #     positions = positions + [Vector3(initial_position.x + X[i], initial_position.y + Y[i], initial_position.z)]
    #     velocities = velocities + [Vector3(Xprime[i], Yprime[i] , 0.0)]

    # import matplotlib.pyplot as plt
    # plt.plot(X,Y)
    # plt.show()
    # # plt.plot(T,X)
    # # plt.plot(T,Y)
    # # plt.show()
    # # plt.plot(T,Xprime)
    # # plt.plot(T,Yprime) 
    # # plt.show()

    # joint_action_server(times, positions, velocities) 

    # loginfo(position_server().position.x - initial_position.x)      
    # loginfo(position_server().position.y - initial_position.y)      
    # loginfo(position_server().position.z - initial_position.z)      
     
    # joint_action_server([4.0, 8.0], 
    #     [Vector3(current_position.x+0.05, 
    #            current_position.y-0.05, 
    #            current_position.z-0.005),
    #     Vector3(current_position.x,
    #            current_position.y-0.1,
    #            current_position.z-0.01)],    
    #               [Vector3(0.0,-0.05,0.0), Vector3(0.05,0.0,0.0)])

if __name__ == '__main__':
    test()
