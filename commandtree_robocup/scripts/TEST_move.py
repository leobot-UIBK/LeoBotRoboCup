#!/usr/bin/env python

# next node



import rospy
from std_msgs.msg import *
from atwork_commander_msgs.msg import *
import actionlib
from commandtree_msgs.msg import *
import copy

def move_client(move_task):
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('move', ACT_MoveAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = ACT_MoveGoal(move_task=move_task)
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()  # A MoveResult




if __name__ == '__main__':
    move_task = MSG_MoveStamped()
    move_task.goal_position = "task.arena_start_state[0].workstation_name"
    rospy.init_node('TEST_move', anonymous=True)
    result = move_client(move_task)
    rospy.loginfo("%s : Position after Move: x=%i y=%i orientation x=%i" %(rospy.get_name(), result.last_position.pose.position.x, result.last_position.pose.position.y, result.last_position.pose.orientation.x))
    rospy.sleep(1)
    rospy.spin()
