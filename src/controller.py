#!/usr/bin/env python
import rospy

def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def controller():
    rospy.init_node('controller')
    loginfo("Initialized node Controller")

if __name__ == '__main__':
    controller()
