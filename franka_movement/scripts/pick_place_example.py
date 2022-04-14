# ROS
import rospy
import franka_movement_msgs.srv as franka_srv
import moveit_msgs.msg as moveit_msg
import trajectory_msgs.msg as trajectory_msg
import shape_msgs.msg as shape_msg
import geometry_msgs.msg
#import tf
import math
import tf2_ros as tf
import tf2_geometry_msgs
import tf.transformations as tft

import planning_scene_example
import numpy as np

import time

def example_colision_obj(id):
    # define message
    primitve = shape_msg.SolidPrimitive()
    primitve.type = primitve.BOX
    primitve.dimensions = [0.05, 0.05, 0.05]

    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0.5
    pose.position.y = 0.0
    #pose.position.z = 0.145
    pose.position.z = 0.2

    q = tft.quaternion_from_euler(0.0, 0.0, 0.0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    collision_object = moveit_msg.CollisionObject()
    collision_object.header.frame_id = "panda_link0"
    collision_object.id = id
    collision_object.primitive_poses = [pose]
    collision_object.primitives = [primitve]

    return collision_object

def spawn_cube(obj):
    service = "/planing_scene_node/collision_object"
    rospy.wait_for_service(service)


    add_collision_obj = franka_srv.CollisionObjectRequest()
    add_collision_obj.collision_object = obj
    add_collision_obj.operation = add_collision_obj.ADD         # ADD

    try:
        call_collision_object = rospy.ServiceProxy(service, franka_srv.CollisionObject)
        response = call_collision_object.call(add_collision_obj)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def basic_pick(name):
    service = "/franka_move_node/basic_pick"

    pick_msg = franka_srv.FrankaBasicPickRequest()
    pick_msg.object_id = name
    pick_msg.constrain_grasp_width = False

    try:
        call_pick = rospy.ServiceProxy(service, franka_srv.FrankaBasicPick)
        response = call_pick.call(pick_msg)
        print("basic_pick error_code: ", response.error_code)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def basic_place(name, pose):
    service = "/franka_move_node/basic_place"

    place_msg = franka_srv.FrankaBasicPlaceRequest()
    place_msg.object_id = name
    place_msg.place_pose = pose
    place_msg.constrain_grasp_width = False

    try:
        call_place = rospy.ServiceProxy(service, franka_srv.FrankaBasicPlace)
        response = call_place.call(place_msg)
        print("basic_place error_code: ", response.error_code)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def remove_obj(name):
    service = "/planing_scene_node/collision_object"

    collision_obj = franka_srv.CollisionObjectRequest()
    collision_obj.collision_object.id = name
    collision_obj.operation = collision_obj.REMOVE         # ADD

    try:
        call_collision_object = rospy.ServiceProxy(service, franka_srv.CollisionObject)
        response = call_collision_object.call(collision_obj)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def remove_obj_attached(name):
    service = "/planing_scene_node/attached_collision_object"

    collision_obj = franka_srv.AttachedCollisionObjectRequest()
    collision_obj.attached_collision_object.object.id = name
    collision_obj.operation = collision_obj.REMOVE         # ADD

    try:
        call_collision_object = rospy.ServiceProxy(service, franka_srv.AttachedCollisionObject)
        response = call_collision_object.call(collision_obj)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def pick(obj):
    service = "/franka_move_node/pick"

    grasp = moveit_msg.Grasp()
    grasp.id = "test_grasp"
    grasp.grasp_pose.header.frame_id = "panda_link0"

    # position before grasp
    # TODO set rotation of obj
    obj_pose = obj.primitive_poses[0]
    q = tft.quaternion_from_euler(-math.pi, 0, -1 * math.pi / 4)
    grasp.grasp_pose.pose.orientation.x = q[0]
    grasp.grasp_pose.pose.orientation.y = q[1]
    grasp.grasp_pose.pose.orientation.z = q[2]
    grasp.grasp_pose.pose.orientation.w = q[3]

    # set pos of obj
    grasp.grasp_pose.pose.position.x = obj_pose.position.x
    grasp.grasp_pose.pose.position.y = obj_pose.position.y
    grasp.grasp_pose.pose.position.z = obj_pose.position.z + 0.1

    # before the grasp the robots moves from grasp_pose by pre_grasp_approach
    grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0"
    grasp.pre_grasp_approach.direction.vector.z = -1.0
    grasp.pre_grasp_approach.min_distance = 0.08
    grasp.pre_grasp_approach.desired_distance = 0.1

    # after the grasp the robot retreats by post_grasp_retreat
    grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0"
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.08
    grasp.post_grasp_retreat.desired_distance = 0.1

    traj_point = trajectory_msg.JointTrajectoryPoint()
    traj_point.positions.append(obj.primitives[0].dimensions[0] / 2.0)
    traj_point.positions.append(obj.primitives[0].dimensions[0] / 2.0)
    grasp.grasp_posture.points.append(traj_point)
    grasp.grasp_posture.joint_names.append("panda_finger_joint1")
    grasp.grasp_posture.joint_names.append("panda_finger_joint2")

    grasp.max_contact_force = 0

    pick_msg = franka_srv.FrankaPickRequest()
    pick_msg.grasp = grasp
    pick_msg.object_id = name

    try:
        call_pick = rospy.ServiceProxy(service, franka_srv.FrankaPick)
        response = call_pick.call(pick_msg)
        print("pick error code: ", response.error_code)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def place(name):
    service = "/franka_move_node/place"

    placeLocation = moveit_msg.PlaceLocation()
    placeLocation.id = "test_place"

    tf.TransformBroadcaster()

    #n = rospy.init_node("placer_private", anonymous=True)
    #tf_buffer = tf.Buffer(rospy.Duration(1200))   # tf buffer length
    #tf_listener = tf.TransformListener(tf_buffer)

    placeLocation.place_pose.header.frame_id = "panda_link0"
    q = tft.quaternion_from_euler(0, 0, 0)
    placeLocation.place_pose.pose.orientation.x = q[0]
    placeLocation.place_pose.pose.orientation.y = q[1]
    placeLocation.place_pose.pose.orientation.z = q[2]
    placeLocation.place_pose.pose.orientation.w = q[3]

    #transform = tf_buffer.lookup_transform("panda_link8", "panda_gripper_center", rospy.Time(0), rospy.Duration(1.0))
    #pose_transformed = tf2_geometry_msgs.do_transform_pose(place_pose, transform)

    # set pos of obj
    placeLocation.place_pose.pose.position.x = 0.5
    placeLocation.place_pose.pose.position.y = 0.3
    placeLocation.place_pose.pose.position.z = 0.13

    # before the grasp the robots moves from grasp_pose by pre_grasp_approach
    placeLocation.pre_place_approach.direction.header.frame_id = "panda_link0"
    placeLocation.pre_place_approach.direction.vector.z = -1.0
    placeLocation.pre_place_approach.min_distance = 0.08
    placeLocation.pre_place_approach.desired_distance = 0.1

    # after the grasp the robot retreats by post_grasp_retreat
    placeLocation.post_place_retreat.direction.header.frame_id = "panda_link0"
    placeLocation.post_place_retreat.direction.vector.z = 1.0
    placeLocation.post_place_retreat.min_distance = 0.08
    placeLocation.post_place_retreat.desired_distance = 0.1

    place_msg = franka_srv.FrankaPlaceRequest()
    place_msg.object_id = name
    place_msg.place_location = placeLocation

    print(place_msg)

    try:
        call_place = rospy.ServiceProxy(service, franka_srv.FrankaPlace)
        response = call_place.call(place_msg)
        print("place error code: ", response.error_code)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    name = "Motor"
    planning_scene_example.spawn_floor()
    id = planning_scene_example.spawn_obj_by_name(name, [0.5, 0.0, 0.042 / 2.0 + 0.01], np.array([0.0, 0.0, 0.0]) * (math.pi / 4.0))
    #print(remove_obj_attached("M20_100#0"))
    #obj = example_colision_obj(name)
    #spawn_cube(obj)
    basic_pick(id)
    #pick(obj)

    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "panda_link0"
    place_pose.pose.position.x = -0.505
    place_pose.pose.position.y = 0.205
    #place_pose.pose.position.z = 0.25
    place_pose.pose.position.z = 0.265

    q = tft.quaternion_multiply(tft.quaternion_from_euler(0.246, 0.5651, -0.4382), tft.quaternion_from_euler(0.0, math.pi, 0.0))
    place_pose.pose.orientation.x = q[0]
    place_pose.pose.orientation.y = q[1]
    place_pose.pose.orientation.z = q[2]
    place_pose.pose.orientation.w = q[3]

    print(place_pose.pose.orientation)


    basic_place(id, place_pose)
    #pick(obj)
    #place(name)

    time.sleep(10)
    print(remove_obj_attached(id))
    remove_obj(id)

