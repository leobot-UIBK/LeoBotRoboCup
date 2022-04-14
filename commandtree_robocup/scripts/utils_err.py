import rospy
import geometry_msgs.msg
import franka_movement_msgs.srv
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from franka_msgs.msg import Errors, ErrorRecoveryAction, ErrorRecoveryActionGoal, FrankaState
from geometry_msgs.msg import WrenchStamped
from franka_modules.panda_move_modules import (
     panda_init,
     moveEE)
from collections import deque
import utils_move
from franka_modules.panda_grip_modules import pandagripper_move_client

import utils

class FrankaContainer:
    def __init__(self, node): 
        self.F = [deque(maxlen = 10), deque(maxlen = 10), deque(maxlen = 10)] #
        self.F_sign = [deque(maxlen = 10), deque(maxlen = 10), deque(maxlen = 10)] 
        self.F2 = np.array([0.0,0.0,0.0])
        self.q = JointState()
        self.Errors = Errors()
        self.node = node
        # self.subErr = rospy.Subscriber('') # subscribe to error
#        self.subq = # subscribe to joint positions
        self.subF = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.ForceCallback)
        self.counter = 0
        self.error_read = False
        self.subState = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.StateCallbackError) 
        # self.subJoints = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.JointCallback)
        self.debug = False
        self.gripperwidth = 0
        
    # read the force part of the wrench and sum up the squared values 
    def ForceCallback(self, data): 
        F = data.wrench.force
        F = np.array([F.x, F.y, F.z])
        F_sign = np.sign(F)
#        print(F)
        for i in range(3):     
            self.F[i].append(F[i])
            self.F_sign[i].append(F_sign[i])
            
        self.F2 += F_sign * F **2
        if len(self.F[0]) == 10: 
            self.F2 -= np.array([self.F[0][0]*self.F_sign[0][0], self.F[1][0]*self.F_sign[1][0], self.F[2][0]*self.F_sign[2][0]]) ** 2 # todo: may be a problem when signs change? 
            
            if self.debug: 
                print(str(self.F[0][0]) + ' | ' + str(self.F[1][0]) + ' | ' + str(self.F[2][0]))
                print(self.F2)
                print(self.counter)
        else: 
            self.counter +=1
            
    def JointCallback(self, data): 
        self.q = data
        # the current gripperwidth is the sum of panda_finger1 and panda_finger2
        self.gripperwidth = data.position[7] + data.position[8] 
        return
    
    def StateCallbackError(self, data): 
        self.Errors = data.current_errors
        self.error_read = True

    def print_current_error(self, showNoErr = True):
        i_wait = 0
        while(self.error_read == False and i_wait < 10): 
            rospy.sleep(0.05)
            i_wait +=1 
        str_error = ''
        Err_types = dir(self.Errors)
        for i in range(len(Err_types)): 
            if Err_types[i][0] == '_' or Err_types[i] in ['serialize_numpy','deserialize_numpy', 'serialize', 'deserialize']: 
                continue
            if self.debug: 
                print('i = ' + str(i) + ':' + Err_types[i] + ' = ' + str(getattr(self.Errors, Err_types[i])) )
            if getattr(self.Errors, Err_types[i]) == True: 
                str_error += str(Err_types[i]) + ','
        if str_error == '' and not(showNoErr):
            rospy.loginfo('No error occured') 
        else:
            rospy.loginfo('The following error occured:')
            rospy.loginfo(str_error)

    def retreat_from_colission(self, distance): 
    #    franka = FrankaContainer(node)
        (robot, scene, moveGroupArm, moveGroupHand) = panda_init()
        rospy.sleep(0.5)
        F = np.sign(self.F2) * np.sqrt(np.abs(self.F2)/self.counter)
        if np.linalg.norm(F) < 0.01: 
            rospy.loginfo('Force close to zero, error recovery may not work well. Distance is decreased.')
            distance = distance * 0.5
        if np.linalg.norm(F) < 0.2: 
           F[2] += 0.2 
        r = distance * F/np.linalg.norm(F, 2) 
        if np.isnan(r).any(): 
            rospy.loginfo('Retreat distance from colission is NaN.')
            rospy.loginfo(str(r))
            return False
        P = np.zeros([1,6])
        P[0,:] = np.array(np.append(r, [0,0,0]))
        (plan,fraction) = planning_cartesian_path(moveGroupArm,P)
        moveGroupArm.execute(plan, wait=True)   #to execute the plan
    
        print("Backup " + str(P) + " m.")
        return True

    def error_recovery(self): 
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)
        rospy.loginfo('Franka error recovery.')
        flag_wait = client.wait_for_server(timeout = rospy.Duration(5))
        if flag_wait == False: 
            print('Timeout')
            return False
        # Creates a goal to send to the action server.
        goal = ErrorRecoveryActionGoal()
        goal.header.stamp = rospy.Time.now() # the current time
        # Sends the goal to the action server.
        client.send_goal(goal)
    
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        return client.get_result()  



    def retreat_from_singularity(self):

        # ToDo! 
        return False

    # should be triggered when for a pick or place task the position before or after the task itself was not reached
    # the retreat distance is in the EndEffector space with inverted z-axis (so that a positive retreat moves in negative z)
    def handle_position(self, retreat_distance, str_predef_position, moveGroup):
        rospy.loginfo('{} was called with retreat distance {} and the predefined position {}'.format(__name__, retreat_distance, str_predef_position))
        self.error_recovery()
        moveEE(moveGroup,[0,0,-retreat_distance, 0,0,0]) # go back in local z direction
        utils_move.move_to_named(str_predef_position)
        return

    # handle error when a workstation was approached as part of a pick process
    def handle_approach_grasp_workstation(self, retreat_distance, moveitId, moveGroup):
        rospy.loginfo('{} was called with retreat_distance for the moveitID'.format(__name__, retreat_distance, moveitId))
        self.error_recovery()
        moveEE(moveGroup,[0,0,-0.002, 0,0,0]) # go back in local z direction
        grasp_result = utils_move.grasp_object_by_id('')
        if grasp_result:
            # attatch object to arm!
            moveGroup.attach_object(moveitId, link_name='panda_gripper_center',touch_links=['panda_leftfinger','panda_rightfinger'])
            moveEE(moveGroup,[0,0,-retreat_distance, 0,0,0]) # go back in local z direction
            return True # grasped object
        return False # failed to pick something

    def handle_approach_place_workstation(self, retreat_distance, moveitId, moveGroup):
        rospy.loginfo('{} was called with retreat_distance for the moveitID'.format(__name__, retreat_distance, moveitId))
        self.print_current_error()
        if self.Errors.cartesian_reflex: # we hit an object by going down
            # there is an object where it was attempted to place, so go back and try again
            self.error_recovery()
            moveEE(moveGroup,[0,0,-retreat_distance, 0,0,0]) # go back in local z direction
            return False
        else: # nothing there, continue
            self.error_recovery()
            pandagripper_move_client(0.07, grippersimulation=False)           # open gripper
            moveGroup.detach_object(moveitId)
            moveEE(moveGroup,[0,0,-retreat_distance, 0,0,0]) # go back in local z direction
            return True

# handle errors when placing on the stack (between preposition and opening of gripper)
    def handle_place_stack(self, retreat_distance, moveitId, moveGroup):
        rospy.loginfo('{} was called with retreat_distance {} for object {}'.format(__name__, retreat_distance, moveitId))
        self.error_recovery()

        name, ID = utils.get_name_and_number_from_planning_scene_id(moveitId)
        bb_width = rospy.get_param('/objects_manipulation/' + name + '/boundingbox')[1] # width of the bounding box
        pandagripper_move_client(bb_width*1.2, grippersimulation=False)           # open gripper
        moveGroup.detach_object(moveitId)
        moveEE(moveGroup,[0,0,-retreat_distance, 0,0,0]) # go back in local z direction
        return


    def handle_approach_grasp_stack(self, retreat_distance, moveitId, moveGroup):
        rospy.loginfo('{} was called'.format(__name__))
        rospy.loginfo('Strategy is to retreat 0.002m, grasp object {} and retreat {}m'.format(moveitId, retreat_distance))
        self.error_recovery()
        moveEE(moveGroup,[0,0,-0.002, 0,0,0]) # go back in local z direction
        grasp_result = utils_move.grasp_object_by_id('')
        # moveEE(moveGroup,[0,0,-retreat_distance, 0,0,0]) # go back in local z direction
        return grasp_result





# """
# for testing, functions should be moved to utilities! 
if __name__ == '__main__':
    node = rospy.init_node('Err_recovery_debugger')
    franka = FrankaContainer(node) 
    franka.print_current_error()
    rospy.sleep(1)
    print(franka.q)
    print(franka.gripperwidth)
    # error_recovery()
    # r = retreat_from_colission(node, 0.1)
    #print(r)
# """

