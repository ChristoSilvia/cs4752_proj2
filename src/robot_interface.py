#!/usr/bin/env python
# license removed for brevity
#from beginner_tutorials.srv import * TODO need to replace
import rospy
from std_msgs.msg import String
from zic_proj1.msg import State
from zic_proj1.srv import *
from config import *

from baxter_interface import RobotEnable, Gripper

state = State()

#locations on table will be given by function in this file

def robot_interface():
    rospy.init_node('robot_interface')

    rospy.loginfo("Beginning to enable robot")
    global baxter
    baxter = RobotEnable()
    rospy.loginfo("Enabled Robot")

    rospy.loginfo("Beginning to initialize left gripper")
    global left_gripper
    left_gripper = Gripper('left')
    rospy.loginfo("Left Gripper initialized")

    state_publisher = rospy.Publisher('/state', State, queue_size=10) #initializes publisher to chatter, type of data to publish, size of messages to store
    
    move_robot_service = rospy.Service('/move_robot', MoveRobot, handle_move_robot) # /move_robot
    get_state_service = rospy.Service('/get_state', GetState, handle_get_world_state) # /move_robot

    print "Ready to move robot."

    config = rospy.get_param('configuration')
    num_blocks = rospy.get_param("num_blocks")

    state.gripper_closed = False
    state.block_in_gripper = 0
    state.stack = range(1, num_blocks+1)
    if config == "stacked_descending" :
        state.stack.reverse()
    state.table = []

    rospy.loginfo("configuration: %s",config)
    rospy.loginfo("num_blocks: %d",num_blocks)

    broadcast_rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        # publish state
        state_publisher.publish(state)
        rospy.loginfo(state)
        broadcast_rate.sleep()

    
    rospy.spin()

def handle_move_robot(req):

    success = True

    if req.action == OPEN_GRIPPER : #CHRIS  we are using the target as the destination for this block
        #in practice this means calling MoveRobot -OPEN_GRIPPER with same target as Move_Robot - 
        #Move_over_block
        print "opened gripper"
        if req.target == 0 : #putting block on table
            state.table.append(state.block_in_gripper)
            print "Deleting {0}".format(state.stack.index(state.block_in_gripper))
            del state.stack[state.stack.index(state.block_in_gripper)]
        else : #appending to stack
            state.stack.append(state.block_in_gripper)
            del state.table[state.table.index(state.block_in_gripper)]
        state.block_in_gripper = 0
        state.gripper_closed = False

        rospy.loginfo("Beginning to open gripper")
        left_gripper.open()
        rospy.loginfo("Opened Gripper")

    elif req.action == CLOSE_GRIPPER :

        rospy.loginfo("Beginning to close Gripper")
        left_gripper.close()
        rospy.loginfo("Closed Gripper")

        state.gripper_closed = True
        state.block_in_gripper = req.target

    elif req.action == MOVE_TO_BLOCK :
        print "Moved to block {0}".format(req.target)
        if state.block_in_gripper > 0 or state.gripper_closed:
            success = False

    elif req.action == MOVE_OVER_BLOCK :
        print "Moved over block {0}".format(req.target)
    elif req.action == MOVE_OVER_TABLE :
        print "Moved over table"
    elif req.action == MOVE_TO_STACK_BOTTOM :
        print "Moved to stack bottom"
    else :
        print "invalid action"

    return MoveRobotResponse(success)

def pick_and_place(source, destination):
    """ Routine is a stub, implemented for part 2(b) """
    pass



def handle_get_world_state(req):
    resp = GetStateResponse()
    resp.gripper_closed = state.gripper_closed
    resp.block_in_gripper = state.block_in_gripper
    resp.stack = state.stack
    resp.table = state.table
    return resp

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
