#! /usr/bin/env python
import franka_movement_msgs.srv
import rospy
import actionlib
import time
from atwork_commander_msgs.msg import *
import geometry_msgs
from commandtree_msgs.msg import *
import commandtree_msgs.srv
import utils_move
import utils
import tf.transformations
import moveit_msgs.msg
import utils_err
import math
import rospy
from robocup_object_recognition.srv import StartYoloRecognition
from robocup_object_recognition.msg import RoboCupObject
import tf
import moveit_commander
from franka_modules.panda_move_modules import (
    moveEE,
    makeObjStack,
    makeArenaWorkStation,
    pickplaceMove,
    move2objStackpose,
    moveEE2seach,
    addAllWallsEnvironment,
    addWallEnvironment)


class Object_ac: 
    def __init__(self, name, robocupobject, moveit_id): 
        self.name = name
        self.data = robocupobject
        self.acID = utils_move.object_string_to_id(self.data.label.data)
        self.moveitID = moveit_id
        
class Service_area:
    def __init__(self, name, arena_node, scene):
        self._action_name = name
        # todo dict with a {ac_object, object_id, stack_num} entry for each object on the robot
        # ac_object: the atwork_commander object; object_id: the object id within the planning scene: stack_num: the number of the occupied stack (0 for ee) (-1 for None)
        self.occupied_stack = []
        self.found_objects = []
        self.table_id = name
        self.table_height = rospy.get_param('workstations/world2' + name + '/height')
        self.orientation_to_table = rospy.get_param('workstations/world2' + name + '/robotPose')
        self.current_grasped_object_id = ""
        self.arenanode = arena_node
        self.frankaContainer = utils_err.FrankaContainer(arena_node)        # needs rosnode for force subscriber s
        self.moveGroupArm = moveit_commander.MoveGroupCommander("panda_arm")
        self.place_counter = 0

        self.object_y_debug = [-0.5,  0.0, 0.5, -0.25, 0.25, 0.125, -0.125]      # todo remove this
        self.scene = scene
        self.debug_grasp = False
        self.debug_vis = False
        self.debug_move = False
        self.debug_place = False

        self.tf_listener_ = tf.TransformListener()

    # transforms from current frame to new frame
    def transform_pose_stamped(self, pose, new_frame):
        p_frame2 = self.tf_listener_.transformPose(new_frame, pose)
        # print "Position of the fingertip in the robot base:" # print for debugging.
        # print p_frame2
        return p_frame2

    """
    This is a routine that grasps an given object from the robot stack and places it on the current workstation.
    @return: True if success, False otherwise
    """
    def from_robot(self, ac_desired_objects):

        state = 0
        result_success = False
        # the current grasped object
        object_id = "" 
        i_object = 0
        i_fails = 0  
        n_fails = 3
        # todo: move to predefined position first: 
        
        
        while (i_object < len(ac_desired_objects)): 
                # todo grasp object

                if state is 0: # grasp from stack
                    if not(self.debug_grasp): 
                        # get from the stack on robot's back
                        rospy.loginfo('Get from object ' + str(ac_desired_objects[i_object]) + ' from the stack.')
                        result_success, object_id = self.perform_grasp_stack(ac_desired_objects[i_object])
                    else:
                        result_success = True
                    
                    if not result_success and i_fails < n_fails: # try again? 
                        # self.current_grasped_object_id = ac_desired_object
                        # this should work in most cases...
                        # todo write error
                        i_fails += 1
                        break
    
                    state = 1
                    continue
    
                # todo place object
                if state is 1:
                    if not(self.debug_grasp): 
                        rospy.loginfo('Place object ' + str(ac_desired_objects[i_object]) + ' on the the workstation.')
                        result_success = self.place_on_workstation(ac_desired_objects[i_object])
                    else: 
                        result_success = True
                    
                    if not result_success:
                        # todo write error
                        state=0
                        i_object +=1
                        continue
                    utils_move.remove_obj(ac_desired_objects[i_object].moveitID) # delete obejcts in moveit scene
                    state = 0
                    i_object += 1
                    i_fails = 0

                    continue

        return result_success

    """
    This is a routine that grasps a given object from a workstation and stores them in the robots function stack.
    """
    def to_robot(self, ac_desired_objects):
        _object = None
        result_success = False
        state = 0

        # do all three statuses with possible canceling from the arena_info
        # 0: find object
        # 1: grasp all objects
        # 2: place object
        finished = False
        i_object = 0
        objects_manipulated = []
        str_homingPosition =  str('poseDesk') + self.orientation_to_table[8:] # append  Right/Left/Front to the String for predef position
        utils_move.move_to_named(str_homingPosition)
        i_fails = 0 # counts fails and may break loop after too many unsuccessfull atempts

        if self._action_name == 'SH01':
            return 0, 0


        while finished == False:
            # find task objects
            if (state is 0 and i_object == 0):
                success, all_objects = self.localize_all_objects(ac_desired_objects)
                print("detected objects: ", all_objects)
                # if service_area got canceled during find_object
                if not success:
                    # todo call error prevention routine; for BMT receive all
                    rospy.logerr("{}: some objects from {} not found".format(self._action_name, ac_desired_objects))
                    result_success = False
                    if all_objects == []:
                        rospy.logerr('No objects found, skipping getting objects from this motion!')
                        return result_success
                    break
                else:
                    # str_obj = ''
                    # for object in [all_objects]:
                       #  str_obj += object.label.data + ', '
                    # rospy.loginfo('{}: located desired objects: {}\tnext: grasp objects '.format(self.to_robot.__name__, str_obj))
                    try:
                        _object = all_objects.pop()
                    except:
                        rospy.loginfo('all localized objects manipulated. ')
                        finished = True
                    state = 1
                    continue
            elif (state is 0 and i_object!=0): # only pop from list, no new localization needed
                _object = all_objects.pop()
                state = 1
                
            # grasping
            if state is 1:
                # result from find task object is the goal
                if not(self.debug_grasp): 
                    success = self.perform_grasp_workstation(_object)
                    print('Grasped with gripperwidth: ', self.frankaContainer.gripperwidth)
                    # _object = self.check_obj_gripperwidth(_object)
                        
                else: # skip for debugging 
                    success = True
                # if service_area got canceled during grasping
                if not success:
                    # publish the feedback
                    if (self.frankaContainer.error_recovery()) == False:
                        result_success = False # could not recover from error!
                        break
                    else:
                        # self.frankaContainer.retreat_from_colission(0.02) #
                        moveEE(self.moveGroupArm,[0,0,-0.004, 0,0,0]) # go back in local z direction

                        utils_move.grasp_object_by_id('')
                        utils_move.grasp_object_by_id('')
                        rospy.loginfo('triggered error recovery for Object ' + _object.moveitID)
                        # close gripper
                        state = 2
                else:
                    rospy.loginfo('%s : Successful Grasp\tnext: place object ' % (self._action_name))
                    state = 2
                    continue

            # placing on stack
            if state is 2:
                if not(self.debug_grasp): 
                    success = self.perform_place_stack(_object)
                    objects_manipulated += [_object]
                else:  # skip for debugging 
                    success = True
                # if service_area got canceled during placing
                if not success:
                    # todo implement recover behaviour
                    
                    # go back to home and try agan OR open gripper and let it fall? 
                    result_success = False
                    break
                else:
                    rospy.loginfo('%s : located object %s place it to robot' % (self._action_name, _object.acID))
                    result_success = True
                    state = 0
                    i_object += 1
                    if i_object == len(ac_desired_objects):
                        finished = True
                    rospy.loginfo('%s : finished task at workstation' % (self._action_name))


        # todo remove everythin remaining in all objects
        for obj in all_objects:
            utils_move.remove_obj(obj.moveitID)

        return result_success, objects_manipulated

    #################################################
    # *** ON ROBOT ROUTINES
    #################################################

    def check_obj_gripperwidth(self, _object): 
        if _object.name in ['F20_20_G', 'F20_20_B']:
            if self.frankaContainer.gripperwidth > 0.04: # 
                str_obj = 'S40_40_' + _object.name[-1]
                rospy.loginfo('gripperwidth too big for object' + _object.name + ', should be ' + str_obj )
        elif _object.name in ['S40_40_G', 'S40_40_B']: 
            if self.frankaContainer.gripperwidth < 0.035: 
                str_obj = 'F40_40_' + _object.name[-1]
                rospy.loginfo('gripperwidth too small for object' + _object.name + ', should be ' + str_obj)
        elif _object.name == 'M20': 
            if self.frankaContainer.gripperwidth > 0.038:
                rospy.loginfo('gripperwidth too big for object' + _object.name + ', should be M30')
                _object.name = 'M30'
        elif _object.name == 'M30': 
            if self.frankaContainer.gripperwidth < 0.038: 
                rospy.loginfo('gripperwidth too small for object ' + _object.name + ', should be M20')
                _object.name = 'M20'
        _object.acID = utils_move.object_string_to_id(_object.name)
        return _object
                        
    """
    Use the camera to localize all given objects.
    :param ac_objects A list of all atwork_commander objects that need to be localized
    :return result_success True if all objects could be localized
    :return desired_object_ids The ids of all located objects that were also in the desired list ac_objects
    """
    def localize_all_objects(self, ac_objects):
        
        # todo: interface Matteo!! 
        rospy.loginfo('{} :\tfind objects {} at the workstation {}'.format(
            self.localize_all_objects.__name__, ac_objects, self.table_id))
        result_success = True
        desired_object_ids = []
        manip_objects = []
        # todo: call Matteos service! 
        # map object id to name if a object needs to be transported more than once it will be added multiple to this list
        desired_object_names = []
        for ac_object in ac_objects:
            desired_object_names.append(utils.get_object_name_from_number(ac_object))
        
        if (self.debug_vis): 
            for i in range(len(ac_objects)): 
                object_i = RoboCupObject()
                object_i.pose.position.x = 0.4

                object_i.pose.position.y = self.object_y_debug.pop() # take from here for debugging possibly more than one object

                object_height = rospy.get_param('/objects_manipulation/' + utils_move.object_id_to_string(ac_objects[i]) + '/boundingbox')[2]
                object_i.pose.position.z = self.table_height + object_height*1/2.0 # 0.03

                object_i.pose.orientation.w = 1.0
                object_i.label.data = utils_move.object_id_to_string(ac_objects[i])
                manip_objects += [Object_ac(utils_move.object_id_to_string(ac_objects[i]), object_i, '')]
                success, moveitID = utils_move.spawn_obj_by_name(manip_objects[i].name, manip_objects[i].data.pose, 'panda_footprint')

                if not(success): 
                    rospy.loginfo('could not add spawn moveit object')
                manip_objects[i].moveitID = moveitID
                
            rospy.loginfo('skip vision for debugging')
            return True, manip_objects
        utils_move.drive_to_ws_pictures_poses(0,0.1, self.moveGroupArm)
        rospy.wait_for_service('start_yolo')
        rospy.sleep(1.0)
        start_yolo = rospy.ServiceProxy('start_yolo', StartYoloRecognition)
        detected_objects = start_yolo(1)        # 1 to detect current object
        print("detected objects:", detected_objects)
        # todo workaround for BMT height!
        #for idx, name in enumerate(desired_object_names):
        for object in detected_objects.objects:
            #if detected_objects[i].label.data == name:
            # q = tf.transformations.quaternion_from_euler(0.0, 0.0, -math.pi / 2.0)

            name = object.label.data

            pose_s = geometry_msgs.msg.PoseStamped()
            pose_s.header.frame_id = "camera_arm_color_optical_frame"
            pose_s.pose = object.pose
            object_height = rospy.get_param('/objects_manipulation/' + name + '/boundingbox')[2]
            #
            world_obj_pose_s = self.transform_pose_stamped(pose_s, "world")
            rospy.loginfo('object z position set to table height + object height/2(={})'.format(object_height*1/2.0))
            world_obj_pose_s.pose.position.z = self.table_height + object_height*1/2.0 # 0.03
            #object.pose.position.z = self.table_height + 0.02       # todo change this magic number to the half of the height of the object

            success, moveitID = utils_move.spawn_obj_by_name(name, world_obj_pose_s.pose, "world")
            rospy.loginfo("created moveIt object ID " + moveitID)
            manip_objects += [Object_ac(name, object, moveitID)]
            if not success:
                rospy.logerr("{}: error while adding object with name {} to the planning scene".format(
                    self.localize_all_objects.__name__, name))
                result_success = False
            # detected_objects[i].name = ''
            break
            # todo: error recovery; other arm position and try again?
        
        # todo call vision to find all objects on workstation
        # todo check if all desired objects could be located with a good certainty
        # todo add all objects with a good certainty
        # todo if bad certainty or no object found move to next search position
        # todo else add all objects to planning scene and return (the id from the planning scene)
        
        # todo: check certainty and repeatd possibly
        

        #obj_pose = geometry_msgs.msg.Pose()
        #obj_pose.position.x = .5
        #obj_pose.position.y = self.object_y_debug.pop()
        #obj_pose.position.z = .1
        #obj_pose.orientation.w = 1.
        #if not(self.debug):
        #        continue

        #desired_object_ids.append({"object_id": idx, "ac_object": ac_objects[idx]})

        #rospy.loginfo('%s: exited with: %s' % (self.localize_all_objects.__name__, result_success))
        return result_success, manip_objects

    """
    Grasps an object from an predifned table.
    :param object_id The planning scene ID of the object to grasp 
    :return True on success, false otherwise.
    """
    def perform_grasp_workstation(self, object_ac, n_grasp = 2):
        # todo this is the logic if we want to grasp from a workstation. Implement the grasp from robot back logic too
        rospy.loginfo('%s :\tgrasp object %s' % (self._action_name, object_ac.name))

        # move arm to grasp start position
        self.frankaContainer.print_current_error() # if any
        self.frankaContainer.error_recovery()
        if not utils_move.move_to_predefined_workstation_position(self.orientation_to_table, self.table_id):
            return False

        # grasp
        i_grasp = 0
        while i_grasp < n_grasp:
            error_code = utils_move.call_basic_pick(object_ac.moveitID)
            if error_code == franka_movement_msgs.srv.PickAndPlaceResponse.SUCCESS:
                break
            str_predef_ws_position =  'poseDesk' + self.orientation_to_table[8:] # append  Right/Left/Front to the String for predef position
            rospy.logerr("{} : Error while grasping object {}. Error code: {}".format(self._action_name, object_ac.name, error_code))
            if error_code == franka_movement_msgs.srv.PickAndPlaceResponse.INITIAL_POSE_ERROR:
                self.frankaContainer.handle_position(0.02, str_predef_ws_position, self.moveGroupArm)
            elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_APPROACH_ERROR:
                grasp_result = self.frankaContainer.handle_approach_grasp_workstation(0.2, object_ac.moveitID, self.moveGroupArm)
                # if grasped successfully continue with motion, otherwise try again
                if grasp_result:
                    break

            elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_RETREAT_ERROR:
                self.frankaContainer.handle_position(0.02, str_predef_ws_position, self.moveGroupArm)
                break
            elif error_code == 4: # todo: add grasp error to service response!
                moveEE(self.moveGroupArm,[0,0,-0.2, 0,0,0]) # retreat and try again


        if i_grasp > n_grasp:
            rospy.loginfo('{} : grasp object {} failed after {} trys'.format(self._action_name, object_ac.name, i_grasp))
            return False

        rospy.loginfo('%s : grasp object %s succeed' % (self._action_name, object_ac.name))
        return True

    """
    Places an object on the robots objects stack if possible (free stack place available).
    Otherwise the object will be kept in the gripper.
    :param object_id The planning scene ID of the object to place 
    :param ac_object The corresponding atwork_commander object
    :return True if place was successful or the object remains in the gripper intentionally
    """
    def perform_place_stack(self, object_ac, n_place = 2):
        rospy.loginfo('%s: place object with MoveitID %s at the robot stack' % (self._action_name, object_ac.moveitID))

        # drive to named place pose
        #if not utils_move.move_to_predefined_stack_position(self.orientation_to_table):
        #    return False

        # find free fitting place pose (if there is none keep the object in the hand)
        obj_name, obj_number = utils.get_name_and_number_from_planning_scene_id(object_ac.moveitID)

        stack_num, pos, ori, stack_name = utils.get_free_object_stack_params(object_ac.name, self.occupied_stack)
        # true if a place on the robot objects stack is free. Otherwise keep the object in the hand (stack_num == 0)
        if stack_num > 0:
            # call basic place for free fitting place pose
            pose_s = geometry_msgs.msg.PoseStamped()
            pose_s.header.frame_id = "panda_link0"

            pose_s.pose.position.x = pos[0]
            pose_s.pose.position.y = pos[1]
            pose_s.pose.position.z = pos[2]

            q = tf.transformations.quaternion_multiply(tf.transformations.quaternion_from_euler(ori[0], ori[1], ori[2], axes='rxyz'), tf.transformations.quaternion_from_euler(0.0, math.pi, 0.0, axes='rxyz'))
            pose_s.pose.orientation.x = q[0]
            pose_s.pose.orientation.y = q[1]
            pose_s.pose.orientation.z = q[2]
            pose_s.pose.orientation.w = q[3]
            if stack_num in [1,2,5,9]:
                str_predef_place_position = 'poseTransportPlateLeftCenter' # ellbow right
            elif stack_num in [3,4,6,7,8,10,11,12,13,14,15]:
                str_predef_place_position = 'poseTransportPlateRightCenter' # ellbow left
            utils_move.move_to_named(str_predef_place_position)
            rospy.loginfo("{}: Attempting to place {} on stack position {}".format(self._action_name, object_ac.moveitID, stack_num))

            i_places = 0
            while i_places < n_place:
                error_code = utils_move.basic_place(object_ac.moveitID, pose_s, stack_name, constrain_grasp_width=True)
                if not error_code == franka_movement_msgs.srv.PickAndPlaceResponse.SUCCESS:
                    rospy.logerr("{}: Error during placing {} at pos: {}, ori: {}".
                                 format(self._action_name, object_ac.moveitID, pos, q))
                    # print error, recover from it and try again
                    if stack_num in [1,5,9]:
                        str_predef_stack_position = 'poseTransportPlateLeftCenter' # ellbow right
                    elif stack_num in [2, 3,4,6,7,8,10,11,12,13,14,15]:
                        str_predef_stack_position = 'poseTransportPlateRightCenter' # ellbow left
                    else:
                        str_predef_stack_position = ''

                    if error_code == franka_movement_msgs.srv.PickAndPlaceResponse.INITIAL_POSE_ERROR:
                        self.frankaContainer.handle_position(0.02, str_predef_stack_position , self.moveGroupArm)
                    elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_APPROACH_ERROR:
                        self.frankaContainer.handle_place_stack(0.2, object_ac.moveitID, self.moveGroupArm)
                        cropedStackName = stack_name
                        makeObjStack(self.scene, cropedStackName.replace("objStack", ""), None, onoffToggle="on")


                    elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_RETREAT_ERROR:
                        self.frankaContainer.handle_position(0.02, str_predef_stack_position , self.moveGroupArm)
                        break
                    i_places +=1
                else:
                    break


            if i_places > n_place:
                rospy.loginfo('Failed after {} trys'.format(i_places))
                return False

        # if successful placing occupy stack
        # if in gripper: stack_name = EE, stack_num = 0
        self.occupied_stack.append({"ac_object": object_ac.acID, "object_id": object_ac.moveitID, "stack_num": stack_num, "stack_name": stack_name})

        print(self.occupied_stack)
        rospy.loginfo('%s: Placing Succeeded' % self._action_name)
        return True

    #################################################
    # *** FROM ROBOT ROUTINES
    #################################################

    """
    Grasps an object from the robot stack. If the object is already in the gripper this is ok too.
    If there are multiple objects of the same atwork_commander object type on the stack 
    this routine will decide on its own which object should be placed
    :param ac_object The atwork_commander object that should be grasped
    :return True if the object is successfully in the gripper after this operation. False otherwise
    :return object_id the grasped planning scene object id
    """
    def perform_grasp_stack(self, object_ac, n_grasps = 2):
        rospy.loginfo('{}: grasp object {} from the robot stack'.format(self._action_name, object_ac.acID))

        object_id = ""
        stack_name = ""
        # get a fitting object_id from the stack
        for stack_place in self.occupied_stack:
            # check if an object is in the ee
            if stack_place["stack_num"] is 0:
                # if object is desired object return true else throw error
                if stack_place["ac_object"].object == object_ac.name:
                    return True, object_id
                else:
                    rospy.logerr("{} : Error while grasping object {}. There is an object in the gripper but this is not the desired object.".format(self._action_name, object_ac.name))
                    return False, None

            # todo only the type of the object is considered -> objects with different destinations could be swap
            if stack_place["ac_object"] == object_ac.acID:
                object_id = stack_place["object_id"]
                stack_name = stack_place["stack_name"]

        # move arm to named place pose
        # drive to named place pose
        #if not utils_move.move_to_predefined_stack_position(self.orientation_to_table):
        #    return False
        
        # todo this is dependent on the robot pose to the table
        # grasp
        i_grasps = 0
        while i_grasps < n_grasps:
            error_code = utils_move.call_basic_pick(object_id, stack_name, constrain_grasp_width=True)
            if not error_code == franka_movement_msgs.srv.PickAndPlaceResponse.SUCCESS:
                rospy.logerr("{} : Error while grasping object {}".format(self._action_name, object_id))
                # print error, recover from it and try again
                if stack_place in [1,5,9]:
                    str_predef_stack_position = 'poseTransportPlateLeftCenter' # ellbow right
                elif stack_place in [2, 3,4,6,7,8,10,11,12,13,14,15]:
                    str_predef_stack_position = 'poseTransportPlateRightCenter' # ellbow left
                else:
                    str_predef_stack_position = ''

                if error_code == franka_movement_msgs.srv.PickAndPlaceResponse.INITIAL_POSE_ERROR:
                    self.frankaContainer.handle_position(0.02, str_predef_stack_position , self.moveGroupArm)
                elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_APPROACH_ERROR:
                    grasp_result = self.frankaContainer.handle_approach_grasp_stack(0.02, object_id, self.moveGroupArm)
                    # if something was grasped, continue
                    if grasp_result:
                        break

                elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_RETREAT_ERROR:
                    self.frankaContainer.handle_position(0.02, str_predef_stack_position , self.moveGroupArm)
                    break
                elif error_code == 4:  # todo: properly add error cod e
                    moveEE(self.moveGroupArm,[0,0,-0.2, 0,0,0]) # retreat and try again

                i_grasps +=1
            else:
                break
                #if not utils_move.grasp_object_by_id(object_ac.acID):
                #    self.frankaContainer.error_recovery()
                #    return False, None

            # free occupied place
        if i_grasps > n_grasps:
            rospy.loginfo('Failed after {} trys'.format(i_grasps))
            return False
        elem_to_remove = {}
        for stack_place in self.occupied_stack:
            if stack_place["object_id"] == object_id:
                elem_to_remove = stack_place
        self.occupied_stack.remove(elem_to_remove)

        rospy.loginfo('{} : grasp object {} succeed'.format(self._action_name, object_id))
        return True, object_id

    """
    Places a grasped object on the current workstation.
    :param object_id The planning scene id of the current grasped object.
    :return True if success, False otherwise
    """
    def place_on_workstation(self, object_ac, n_places= 2):
        rospy.loginfo('{}: place object {} on the current workstation'.format(self._action_name, object_ac.moveitID))

        # move arm to grasp start position
        if not utils_move.move_to_predefined_workstation_position(self.orientation_to_table, self.table_id):
            return False
        
        self.table_height = rospy.get_param('/workstations/world2' + self.table_id + '/height')
        
        # todo get placing coordinates
        # todo: retrieve objects with bounding boxes from vision to find a free place
        pose_s = utils.get_valid_place_pose(self.table_id, self.place_counter, debug=self.debug_place)
        pose_s.pose.position.z += 0.035
        

        # place
        print("{}: Attempting to place {} on current workstation {}. The place pose is: {}".format(self._action_name, object_ac.moveitID, self.table_id, pose_s))

        i_places = 0
        while i_places < n_places:
            error_code = utils_move.basic_place(object_ac.moveitID, pose_s)
            if error_code == franka_movement_msgs.srv.PickAndPlaceResponse.SUCCESS:
                break

            # error handling of the place depending on where the movement error occured
            if not error_code == franka_movement_msgs.srv.PickAndPlaceResponse.SUCCESS:
                rospy.logerr("{} : Error while placing object {} on workstation".format(self._action_name, object_ac.moveitID))
                # print error, recover from it and try again
                str_predef_ws_position =  str('poseDesk') + self.orientation_to_table[8:] # append  Right/Left/Front to the String for predef position
                if error_code == franka_movement_msgs.srv.PickAndPlaceResponse.INITIAL_POSE_ERROR:
                    self.frankaContainer.handle_position(0.02, str_predef_ws_position, self.moveGroupArm)
                elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_APPROACH_ERROR:
                    self.frankaContainer.handle_approach_place_workstation(0.02, object_ac.moveitID, self.moveGroupArm)
                    # do not break
                elif error_code == franka_movement_msgs.srv.PickAndPlaceResponse.PICK_PLACE_RETREAT_ERROR:
                    self.frankaContainer.handle_position(0.02, str_predef_ws_position, self.moveGroupArm)
                    break
                i_places +=1

        if i_places > n_places:
            rospy.loginfo('Failed after {} trys'.format(i_places))
            return False
        """
        if not error_code == franka_movement_msgs.srv.PickAndPlaceResponse.SUCCESS:
            rospy.logerr("{}: Error during placing. Error Code: {}".format(self._action_name, error_code))
            self.moveGroupArm.detach_object(object_ac.moveitID)     # robot opened gripper
            return False
        """
        rospy.loginfo('{}: Placing Succeeded'.format(self._action_name))

        utils_move.remove_obj(object_ac.moveitID)
        return True
