#!/usr/bin/env python
# liscense removed for brevity

import rospy

def loginfo(infostring):
    rospy.loginfo("Robot Interface: {0}".format(infostring))

def robot_interface():
    rospy.init_node('robot_interface')
    loginfo("Initialized Robot Interface")

    rospy.spin()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
