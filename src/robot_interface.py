#!/usr/bin/env python
# liscense removed for brevity

import numpy as np

import rospy
from cs4752-proj2.srv import *

def loginfo(infostring):
    rospy.loginfo("Robot Interface: {0}".format(infostring))

def sample_cspace():
    loginfo("Sampling Random Point in Cspace")
    point = np.random.rand()
    loginfo("Point is {0}".format(point))
    return point

def sample_cspace_response():
    loginfo("Recieved Sample CSpace Query")
    return SampleCSpaceResponse(sample_cspace)

def robot_interface():
    rospy.init_node('robot_interface')
    loginfo("Initialized Robot Interface")

    loginfo("Beginning to initialize Sampler Service")
    sampler = rospy.Service('sampler', SampleCSpace, sample_cspace_response)
    loginfo("Sampler Service initialized")

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
