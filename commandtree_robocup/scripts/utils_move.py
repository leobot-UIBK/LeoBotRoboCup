import rospy
import geometry_msgs.msg
import franka_movement_msgs.srv
import franka_modules.panda_grip_modules as gripmodules
from franka_gripper.msg import HomingActionGoal, HomingAction
import actionlib

from franka_modules.panda_move_modules import moveEE


import tf

def spawn_obj_by_name(name, pose_obj, frame_id="world", workstation = None):
    service = "/planing_scene_node/add_collision_object_by_id"
    rospy.wait_for_service(service)

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose = pose_obj
    pose_stamped.header.frame_id = frame_id
    # todo: if workstation != nan, set z-direction to table height zero!
    # experimental!
    #if workstation != None:
    #    pose_stamped = workstation.example_TFs(pose_stamped, frame_id, 'world')
    #    rospy.loginfo('pose of colission object in world frame: ' )
    #    rospy.loginfo(pose_stamped)
        #pose_stamped.pose.
    add_collision_obj = franka_movement_msgs.srv.AddCollisionObjectByIdRequest()
    add_collision_obj.id = name
    add_collision_obj.poseStamped = pose_stamped

    try:
        call_add_collision_object_by_id = rospy.ServiceProxy(service, franka_movement_msgs.srv.AddCollisionObjectById)
        response = call_add_collision_object_by_id.call(add_collision_obj)
        return response.success, response.planning_scene_id
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def call_basic_pick(obj_id, support_surface="", constrain_grasp_width=False):
    service = "/franka_move_node/basic_pick"
    
    pick_msg = franka_movement_msgs.srv.FrankaBasicPickRequest()
    pick_msg.object_id = obj_id
    pick_msg.supported_surface = support_surface
    pick_msg.constrain_grasp_width = constrain_grasp_width
    try:
        call_pick = rospy.ServiceProxy(service, franka_movement_msgs.srv.FrankaBasicPick)
        response = call_pick.call(pick_msg)
        return response.error_code
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)


def move_to_named(name):
    service = "/franka_move_node/move_to_named"
    rospy.wait_for_service(service)

    franka_named = franka_movement_msgs.srv.FrankaNamedRequest()
    franka_named.pose_name = name

    try:
        call_named = rospy.ServiceProxy(service, franka_movement_msgs.srv.FrankaNamed)
        response = call_named.call(franka_named)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def basic_place(obj_id, pose_stamped, support_surface="", constrain_grasp_width=False):
    service = "/franka_move_node/basic_place"

    print("place moveIt ID {}".format(obj_id))
    place_msg = franka_movement_msgs.srv.FrankaBasicPlaceRequest()
    place_msg.object_id = obj_id
    place_msg.place_pose = pose_stamped
    place_msg.supported_surface = support_surface
    place_msg.constrain_grasp_width = constrain_grasp_width

    print(pose_stamped)

    try:
        call_place = rospy.ServiceProxy(service, franka_movement_msgs.srv.FrankaBasicPlace)
        response = call_place.call(place_msg)
        return response.error_code
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def move_to_predefined_stack_position(orientation_to_table):
    if orientation_to_table == "w2robotFront":
        arm_position_name = "poseTransportPlateLeftCenter"
    elif orientation_to_table == "w2robotLeft":
        arm_position_name = "poseTransportPlateLeftCenter"
    elif orientation_to_table == "w2robotRight":
        arm_position_name = "poseTransportPlateRightCenter"
    else:
        rospy.logerr('%s :\tno table orientation with id: %s' %
                     (move_to_predefined_stack_position.__name__, orientation_to_table))
        return False

    if not move_to_named(arm_position_name):
        rospy.logerr("{}: Error while moving to predefined position {}".
                     format(move_to_predefined_stack_position.__name__, arm_position_name))
        return False

    return True

def move_to_predefined_workstation_position(orientation_to_table, table_id):
    arm_position_name = ""
    if orientation_to_table == "ws2robotFront":
        arm_position_name = "poseDeskFront"
        if table_id == "SH01":
            arm_position_name = "poseShelfBottomFront"
    elif orientation_to_table == "ws2robotLeft":
        arm_position_name = "poseDeskLeft"
        if table_id == "SH01":
            arm_position_name = "poseShelfBottomLeft"
    elif orientation_to_table == "ws2robotRight":
        arm_position_name = "poseDeskRight"
        if table_id == "SH01":
            arm_position_name = "poseShelfBottomRight"

    
    else:
        # todo aborted states are not working since our actions cant be preempted by default
        rospy.logerr('move_to_predefined_workstation_position: \t no table orientation %s with id: %s' %
                     (orientation_to_table, table_id))
        return False

    if not move_to_named(arm_position_name):
        rospy.logerr("move_to_predefined_workstation_position: Error while moving to predefined position {}".
                     format(arm_position_name))
        return False
    return True

def object_id_to_string(obj_id): 
    strings = rospy.get_param('/objects_manipulation/').keys()
    IDs = []
    for i in range(len(strings)): 
        IDs += [rospy.get_param('/objects_manipulation/' + strings[i] + '/ID')]
    try: 
        return strings[IDs.index(obj_id)]
    except ValueError: 
        return '' 
    
def object_string_to_id(obj_str): 
    return rospy.get_param('/objects_manipulation/' + obj_str +'/ID') 

def grasp_object_by_id(obj_id, force_value=70):
        str_name = object_id_to_string(obj_id)
        speed = 0.05
        if str_name != '':      # string not found
            width = rospy.get_param('/objects_manipulation/' + str_name + '/grasp_width')
            # epsilon determines how far away the gripper can be to have a success 
            grip_result = gripmodules.pandagripper_grasp_client(width, force_value, speed, epsilon_inner=0.01, epsilon_outer=0.01, grippersimulation=False)
        else:
            width = 0.03            # try with a standard value
            epsilon_inner = 0.024 # to still be able to grasp the axis with d = 12mm
            epsilon_outer = 0.02
            rospy.logerr('grasp_object_by_id: try to grasp with standard parameters (width:{} speed:{} force:{} epsilon_inner:{} epsilon_outer:{}'.format(width, speed, force_value, epsilon_inner, epsilon_outer))
            grip_result = gripmodules.pandagripper_grasp_client(width, force_value, speed, epsilon_inner=0.015, epsilon_outer=0.02, grippersimulation=False) # epsilon so that everything can be grasped
        return grip_result


def remove_obj(name):
    service = "/planing_scene_node/collision_object"

    collision_obj = franka_movement_msgs.srv.CollisionObjectRequest()
    collision_obj.collision_object.id = name
    collision_obj.operation = collision_obj.REMOVE         # ADD

    try:
        call_collision_object = rospy.ServiceProxy(service, franka_movement_msgs.srv.CollisionObject)
        response = call_collision_object.call(collision_obj)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def home_gripper():
    client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
    client.wait_for_server()
    client.send_goal(HomingActionGoal())
    result = client.wait_for_result()
    if result:
        rospy.loginfo('Franka Gripper sucessfully homed.')
    else:
        rospy.logerr('Franka Gripper Hand could not be initialized.')


def drive_to_ws_pictures_poses(tableheight, tableDistance, moveGroup):
    moveEE(moveGroup,[0,0, tableDistance, 0,0,0]) # go back in local z direction



