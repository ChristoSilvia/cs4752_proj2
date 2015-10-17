#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from zic_proj1.srv import MoveRobot, GetState
from config import * 

def log_info(log_string):
    rospy.loginfo("Controller: {0}".format(log_string))

def log_err(log_string):
    rospy.logerr("Controller: {0}".format(log_string))

def scatter():
    log_info("Beginning to scatter blocks")
    n = rospy.get_param("num_blocks")

    while len(get_state().stack) > 0:
        current_block = get_state().stack[-1]
        log_info("\nBeginnning remove block subroutine for block {0}".format(current_block))
        log_info("There are {0} blocks on the stack".format(len(get_state().stack)))
        log_info("There are {0} blocks on the table".format(len(get_state().table)))
        log_info("Beginning to take block {0} off of the stack".format(current_block))

        if len(get_state().table) == 0:
            log_info("Robot is in starting configuration. Skipping Directly to CLOSE_GRIPPER")
        else:
            log_info("Beginning to move hand to block {0}".format(current_block))
            move_robot(MOVE_TO_BLOCK, current_block)
            log_info("Successfully moved hand to block {0}".format(current_block))

        log_info("Beginning to close gripper around block {0}".format(current_block))
        move_robot(CLOSE_GRIPPER, current_block)
        log_info("Successfully closed gripper around block {0}".format(current_block))

        log_info("Begining to move gripper over table position for block {0}".format(current_block))
        move_robot(MOVE_OVER_TABLE, current_block)
        log_info("Successfully moved gripper over table position for block {0}".format(current_block))

        log_info("Beginning to open gripper to release block {0} onto table.".format(current_block))
        move_robot(OPEN_GRIPPER, -1)
        log_info("Successfully deposited block {0} at its position on table".format(current_block))
    
    log_info("\nSuccessfully scattered blocks.\n\n")

def stack_ascending():
    n = rospy.get_param("num_blocks")
    log_info("Beginning to stack blocks ascending")

    if sorted(get_state().stack) is get_state().stack:
        log_info("Blocks already stacked ascending.\n")
        return True
    elif len(get_state().stack) > 0:
        log_info("Blocks aren't scattered, beginning to scatter.\n")
        scatter()
        log_info("Successfully called Scatter subroutine, continuing to stack ascending.")

    for i in range(1,n+1):
        log_info("\nBeginning Stack Subroutine for block {0}".format(i))
        log_info("Beginning to put block {0} on the stack".format(i))

        log_info("Beginning to move to block {0}".format(i))
        move_robot(MOVE_TO_BLOCK, i)
        log_info("Moved to block {0}".format(i))

        log_info("Beginning to close gripper around block {0}".format(i))
        move_robot(CLOSE_GRIPPER, i)
        log_info("Closed gripper around block {0}".format(i))

        if i is 1:
            log_info("Moving block {0} to base of stack".format(i))
            move_robot(MOVE_TO_STACK_BOTTOM, 1)
            log_info("Moved block {0} to base of stack".format(i))
        else:
            log_info("Moving block {0} above block {1}, on top of stack".format(i, i-1))
            move_robot(MOVE_OVER_BLOCK, i-1)
            log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(i,i-1))
        
        log_info("Releasing gripper around block {0}".format(i))
        move_robot(OPEN_GRIPPER, i-1)
        log_info("Released block {0} from gripper".format(i))
    
    log_info("\nSuccessfully stacked blocks ascending.\n\n")

def stack_descending():
    n = rospy.get_param("num_blocks")
    log_info("Beginning to stack blocks descending")

    if list(reversed(sorted(get_state().stack))) is get_state().stack:
        log_info("Blocks already stacked ascending.\n")
        return True
    elif len(get_state().stack) > 0:
        log_info("Blocks aren't scattered, beginning to scatter.\n")
        scatter()
        log_info("Successfully called Scatter subroutine, continuing to stack descending.")

    for i in range(n, 0, -1):
        log_info("\nBeginning Stack Subroutine for block {0}".format(i))
        log_info("Beginning to put block {0} on the stack".format(i))

        log_info("Beginning to move to block {0}".format(i))
        move_robot(MOVE_TO_BLOCK, i)
        log_info("Moved to block {0}".format(i))

        log_info("Beginning to close gripper around block {0}".format(i))
        move_robot(CLOSE_GRIPPER, i)
        log_info("Closed gripper around block {0}".format(i))

        if i is n:
            log_info("Moving block {0} to base of stack".format(i))
            move_robot(MOVE_TO_STACK_BOTTOM, n)
            log_info("Moved block {0} to base of stack".format(i))
        else:
            log_info("Moving block {0} above block {1}, on top of stack".format(i, i+1))
            move_robot(MOVE_OVER_BLOCK, i+1)
            log_info("Moved block {0} over block {1}, block {0} is on top of stack".format(i,i+1))
        
        log_info("Releasing gripper around block {0}".format(i))
        move_robot(OPEN_GRIPPER, i+1)
        log_info("Released block {0} from gripper".format(i))
    
    log_info("\nSuccessfully stacked blocks descending.\n\n")


def respond_to_command(command):
    log_info("Recieved Command.")
    if command == String("scatter"):
        log_info("Command is \"scatter\"")
        scatter()
        log_info("Executed command \"scatter\"")
    elif command == String("stack_ascending"):
        log_info("Command is \"stack_ascending\"")
        stack_ascending()
        log_info("Executed command \"stack_ascending\"")
    elif command == String("stack_descending"):
        log_info("Command is \"stack_descending\"")
        stack_descending()
        log_info("Executed command \"stack_descending\"")
    else:
        log_err("Recieved invalid command: {0}".format(command))

def listener():
    # Initialize node.  This name should be unique, so it is not anonymous
    #log_info("Initializing node Controller")
    rospy.init_node('controller')
    log_info("Initialized node Controller")

    # Initialize the /move_robot service from robot_interface
    log_info("Beginning to wait for service /move_robot...")
    rospy.wait_for_service("/move_robot")
    log_info("Service /move_robot now available.")

    # Initialize the /get_state service from robot_interface
    log_info("Beginning to wait for service /get_state...")
    rospy.wait_for_service("/get_state")
    log_info("Service /get_state now available.")
    try:
        # Initialize the service proxy for the /move_robot server
        log_info("Initializing service proxy for /move_robot...")
        global move_robot
	move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
        log_info("Initialized service proxy for /move_robot")

        # Initialize the service proxy for the /get_state server
        log_info("Initializing service proxy for /get_state...")
        global get_state
        get_state = rospy.ServiceProxy("/get_state", GetState)
        log_info("Initialized service proxy for /get_state...")
    
        # the controller subscribes to the /command topic and listens for commands
        log_info("Subscribing to topic /command")
        rospy.Subscriber("command", String, respond_to_command)
        log_info("Subscribed to topic /command")

        # keep the server open
        rospy.spin()

    except rospy.ServiceException, e:
        log_err("Service Call Failed: {0}".format(e))


if __name__ == '__main__':
    listener()
        
