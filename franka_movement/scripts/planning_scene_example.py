import franka_movement_msgs.srv as franka_srv
import moveit_msgs.msg as moveit_msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msg
import tf.transformations as tft
import rospy
import math

def get_pose_object(pose, orientation):
    pose_obj = geometry_msgs.msg.Pose()
    pose_obj.position.x = pose[0]
    pose_obj.position.y = pose[1]
    pose_obj.position.z = pose[2]

    q = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    pose_obj.orientation.x = q[0]
    pose_obj.orientation.y = q[1]
    pose_obj.orientation.z = q[2]
    pose_obj.orientation.w = q[3]

    return pose_obj

def set_colision_obj(id, dimensions, pose, orientation, frame_id="world"):
    # define message
    primitve = shape_msg.SolidPrimitive()
    primitve.type = primitve.BOX
    primitve.dimensions = dimensions

    pose_obj = get_pose_object(pose, orientation)

    collision_object = moveit_msg.CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = id
    collision_object.primitive_poses = [pose_obj]
    collision_object.primitives = [primitve]

    return collision_object

def spawn_obj(obj):
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

def spawn_obj_by_name(name, pose, orientation, frame_id="world"):
    service = "/planing_scene_node/add_collision_object_by_id"
    rospy.wait_for_service(service)

    pose_obj = get_pose_object(pose, orientation)

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose = pose_obj
    pose_stamped.header.frame_id = frame_id

    add_collision_obj = franka_srv.AddCollisionObjectByIdRequest()
    add_collision_obj.id = name
    add_collision_obj.poseStamped = pose_stamped

    try:
        call_add_collision_object_by_id = rospy.ServiceProxy(service, franka_srv.AddCollisionObjectById)
        response = call_add_collision_object_by_id.call(add_collision_obj)
        if(response.success):
            print("new object added with id: {}".format(response.planning_scene_id))
        return response.planning_scene_id
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def spawn_floor():
    spawn_obj(set_colision_obj("ground", [5.0, 5.0, 5.0], [0.0, 0.0, -2.5], [0.0, 0.0, 0.0]))

if __name__=="__main__":
    spawn_floor()
    spawn_obj_by_name("M20_100", [0.5, 0.05, 0.03], [0.0, 0.0, math.pi])

