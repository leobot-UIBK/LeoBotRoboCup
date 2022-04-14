#!/usr/bin/env python

import rospy
from franka_modules.panda_move_modules import makeAllObjStack, panda_init
from std_msgs.msg import *
from atwork_commander_msgs.msg import *
from commandtree_msgs.msg import *
import copy
import utils_arena
from service_area import Service_area
import numpy as np
import move 
from atwork_commander_msgs.msg import Task
import utils_move

#################################################
# CLIENT SERVER:
#################################################
def move_client(move_task, node):
    rospy.loginfo("Movement to {}, move Franka to drive position. ".format(move_task.goal_position))
    utils_move.move_to_named('drive')
    goal_pose = move.Goal().get_robot_goal(move_task.goal_position)

    result = move.movebase_client(goal_pose)
    # Creates the SimpleActionClient, passing the type of the action

    # client = actionlib.SimpleActionClient('move', ACT_MoveAction)
    # Waits until the action server has started up and started
    # listening for goals.
    # client.wait_for_server()
    # Creates a goal to send to the action server.
    # goal = ACT_MoveGoal(move_task=move_task)
    # Sends the goal to the action server.
    # client.send_goal(goal)

    # to cancel an action
    # the are more options to cancel an action, see documentation
    # client.cancel_all_goals()

    # Waits for the server to finish performing the action.
    # client.wait_for_result()
    return result # client.get_result()  # A MoveResult


#################################################
# GET MANIPULATION-LIST:
#################################################
# to find the workstation of an object in a state
def get_workstation_of_object(object, state):
    for workstation in state:
        if object in workstation.objects:
            return workstation, object
    return 0, 0



# returns a list of object manipulation tuples (start_place, destination_place, object)
# object with no manipulation are not listed
# Problem: there are more objects with the same number -> delet_unmanipulated_objects
def get_manipulation_list(task):
    # initialise list of manipulation objects
    manipulation_object_list = []

    # double loop to the objects
    for workstation in task.arena_start_state:
        if workstation.workstation_name == 'TT01':
            continue
        for obj in workstation.objects:
            # get the workstation of the object in target_state
            target_workst, target_obj = get_workstation_of_object(obj, task.arena_target_state)
            if target_workst.workstation_name == 'TT01':
                continue
            # did not find object in target_state
#            if target_obj:
            if obj.decoy:
#                rospy.loginfo('get_mainpulation_list: Object %s is not in target_state', obj)
                continue
            # check if target_state is not equal to the start_state
            if not(target_workst.workstation_name == workstation.workstation_name):
                manipulation_object_list.append((workstation.workstation_name,
                                                 target_workst.workstation_name, obj.object))
                # delete object in target_state -> no double use
                index = get_index_of_workstation_name(task.arena_target_state, target_workst.workstation_name)
                task.arena_target_state[index].objects.remove(obj)

    return manipulation_object_list


# to get the index of the workstation in a state
def get_index_of_workstation_name(state, workstation_name):
    for x in range(len(state)):
        if state[x].workstation_name == workstation_name:
            return x
    return -1


# get the start_state and the target_state as task without objects, which have not to be manipulated
def delet_unmanipulated_objects(task_origin):
    # get a deep copy of the task from the refreeBox
    task = copy.deepcopy(task_origin)
    for workstation in task_origin.arena_start_state:
        # get the same workstation of the target_state
        index = get_index_of_workstation_name(task.arena_target_state, workstation.workstation_name)
        if index == -1:
            continue
        # delete objects which have the same target_state and start_state
        for obj in workstation.objects:
            if obj in task.arena_target_state[index].objects:
                task.arena_start_state[index].objects.remove(obj)
                task.arena_target_state[index].objects.remove(obj)
    return task


def print_manipulation_list(manipulationList):
    for obj in manipulationList:
        print('Object: %s;\tstart_place: %s, destination_place %s' % (obj[2].object, obj[0], obj[1]))





#################################################
# TASK GENERATORS:
#################################################
def create_moveStamped_task(object):
    move_task = MSG_MoveStamped()
    move_task.goal_position = object
    return move_task


def create_transportStamped_task(object, source, destination):
    msg_ser_ar = MSG_TransportStamped()
    msg_ser_ar.transport.object = object
    msg_ser_ar.transport.destination = destination
    msg_ser_ar.transport.source = source
    return msg_ser_ar

def move_into_arena(node):
    rospy.loginfo("Movement into arena")
    arena_start = [1,0,0,0,0,0,1] # drive 1 m into the arena
    utils_move.move_to_named('drive') # predefined position drive
    result = move.movebase_client(arena_start)
    return result

#################################################
# MAIN PROCEDURE:
#################################################
def arena_start(Task, node):
    # objects of manipulation
    rospy.loginfo('Start task:')
    rospy.loginfo(Task)
    short_task = delet_unmanipulated_objects(Task)
    if 0: # filter unknown workstations
        # todo
        return

    manipulation_list = utils_arena.Manipulation_Data(get_manipulation_list(short_task))

    rospy.loginfo("List of objects, which have to be manipulated:")
    for i in range(len(manipulation_list.movements)):
        rospy.loginfo(str(manipulation_list.movements[i]))
#    print_manipulation_list(manipulation_list)

    # todo i guess objects that should leave the map at the end are totally ignored!!
    #grouped_manipulation_list = utils_arena.group_manipulation_list(manipulation_list)
#     rospy.loginfo("Grouped Manipulation List:")
#    for group in grouped_manipulation_list:
#        print_manipulation_list(group)
#    rospy.sleep(1)

    # todo make objstacle stack
    robot, scene, moveGroupArm, moveGroupHand = panda_init()
    makeAllObjStack(scene, moveGroupArm)

    utils_move.home_gripper()
    # move_into_arena(node)
    flag_movement_first = True

    workstations = []
    for movement in manipulation_list.movements:
        workstations.append(movement[0][0])
        workstations.append(movement[0][1])

    workstations = list(set(workstations))


    if flag_movement_first: # to get some points in advance

        print(workstations)
        for tasks in workstations:
            print(tasks)
            move_client(create_moveStamped_task(tasks), node)
            rospy.sleep(3)
        #  perform all movements and manipulations

    for i in range(len(manipulation_list.movements)):
        process_objects(manipulation_list.movements[i], node, scene)

    move_client(create_moveStamped_task('ArenaExit'), node)

    return

def process_objects(movements, node, scene):
    ac_obj = []
    if len(np.array(movements).shape) == 1:
        movements = list([movements])

    n_tasks = np.array(movements).shape[0]
    WS_src = movements[0][0]
    WS_tar = movements[0][1]
    for i in range(n_tasks):
        ac_obj += [movements[i][2]]


    print('-----------------------------------')
    rospy.loginfo('%s\n\tget objects %s from workstation %s to workstation %s.' % (rospy.get_caller_id(), ac_obj, WS_src, WS_tar))
    print('-----------------------------------\n')    

    workstation_src = Service_area(WS_src, node, scene) # source Workstation
    
    
    # get to start workstaton
    if not(workstation_src.debug_move): 
        result_from_move = move_client(create_moveStamped_task(WS_src), node)
    else: 
        rospy.loginfo('skipped movement '+ workstation_src._action_name + 'for debugging')
    #rospy.loginfo('%s :\n\tlast position : x=%i, y=%i; theta=%i\n\tnext: service area: object on robot' %
    #              (rospy.get_caller_id(), result_from_move.last_position.pose.position.x,
    #               result_from_move.last_position.pose.position.y, result_from_move.last_position.pose.orientation.x))

    # pick object
    workstation_src.frankaContainer.error_recovery()
    # objects, current workstation, current orientation
    success, objects_ac = workstation_src.to_robot(ac_obj) # objects_ac returns sucessfully grasped and retrieved objectsmove_to_named
    if not success:
        rospy.logerr("arena_info : on_robot of object[ {}] = {} exited with False".format(i, ac_obj))
        return False

    rospy.loginfo('arena_info : objects {} are at robot\tnext: move to destination'.format(ac_obj))

    workstation_tar = Service_area(WS_tar, node, scene)
    workstation_tar.occupied_stack = copy.deepcopy(workstation_src.occupied_stack)
    del workstation_src

    # move to target workstaton
    if not(workstation_tar.debug_move):
        workstation_tar.frankaContainer.error_recovery()
        result_from_move = move_client(create_moveStamped_task(WS_tar), node)
    else: 
        rospy.loginfo('skipped movement '+ str(workstation_tar._action_name) + 'for debugging')

    

    # place object
    # todo this is a default value. set orientation_to_table
#    orientation_to_table = "w2robotFront"
    # object, current workstation, current orientation
    # if not workstation_tar.from_robot(ac_obj):
    try:
        workstation_tar.frankaContainer.error_recovery()
        if not workstation_tar.from_robot(objects_ac):
            rospy.logerr("arena_info : from_robot exited with False")
            return False
    except:
        rospy.logerr("getting objects from robot failed")
#    rospy.loginfo('arena_info : objects %s are at {} \tnext: process next objects', ac_obj, WS_tar)
    rospy.loginfo('arena_info : objects {} are now at {}\t, process task finished. '.format(ac_obj, WS_tar))
    del workstation_tar
    return True



if __name__ == '__main__':
    arena_node = rospy.init_node('arena_info', anonymous=True)
#    refbox = rospy.wait_for_message('/atwork_commander/internal/state', RefboxState)
    Task = rospy.wait_for_message('/atwork_commander/task',  Task)
    print(Task)
    arena_start(Task, arena_node)
