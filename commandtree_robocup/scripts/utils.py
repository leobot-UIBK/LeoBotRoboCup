import math
import rospy
import numpy as np
import geometry_msgs.msg
import tf.transformations


def get_object_name_from_number(number):
    object_params = rospy.get_param("/objects_manipulation")

    # find number of each object and return name
    for key, values in object_params.items():
        if values["ID"] == number:
            return key

    return None


def get_free_object_stack_params(name, occupied_stack):
    for stack_num in range(1, 3):
        stack_name = "objStack{}_{}".format(stack_num, name);

        stack_params = rospy.get_param("/objects_stack/" + stack_name)

        # check if stack is free
        # generate a list of all occupied stack numbers
        occupied_stack_nums = [element["stack_num"] for element in occupied_stack]
        if not stack_params["Stacknumber"] in occupied_stack_nums:
            return stack_params["Stacknumber"], stack_params["position"], stack_params["orientation"], stack_name

    # 0 is the stack number of the EE
    return 0, None, None, "EE"


def is_pose_in_bounding_box(box_pose, interfering_pose, box_width=0.1):
    box_v = np.array([box_pose.position.x, box_pose.position.y])
    interfering_v = np.array([interfering_pose.position.x, interfering_pose.position.y])
    box_to_interfering = interfering_v - box_v
    return np.linalg.norm(box_to_interfering, ord=2) < box_width / 2.0

def get_valid_place_pose(table_id, place_counter, debug=False):
    # get place poses
    place_pose_params = rospy.get_param("/workstations/placePose").values()
    # get table
    table_param = rospy.get_param("/workstations/world2{}".format(table_id))

    # transform place poses to world frame
    table_rot_mat = tf.transformations.euler_matrix(table_param["orientation"][0], table_param["orientation"][1], table_param["orientation"][2], axes="rxyz")
    table_pos = np.array(table_param["position"])

    # transform all place poses to world frame
    place_pose_params = sorted(place_pose_params, key=lambda k: k['number'])
    place_poses_world = []
    for place_pose_param in place_pose_params:
        # remove invalid place poses (should be marked in the table param space)
        if place_pose_param["number"] == table_param["noPlace"]:
            continue

        place = place_pose_param["position"]
        if table_id == "PP01":
            place = [0.0, 0.0]
        place.append(0.0)
        place_pos = np.array(place)         # add the height of the table
        place_pos_world = table_pos + np.matmul(table_rot_mat[:3, :3], place_pos)
        place_poses_world.append(place_pos_world)

        print(place_pose_param, " ", place_pos_world)

    # todo call vision to get blocked places
    # todo transform resultiong positions to world frame
    # todo check if resulting position is interfering with a place pose

    # todo return first remaining pose
    best_place_pos = place_poses_world[place_counter]
    place_counter += 1

    # set table rotation rotated by pi over x -> this should be a nice EE rotation to place
    flip_x_q = tf.transformations.quaternion_from_euler(0.0, math.pi, 0.0, axes="rxyz")
    table_q = tf.transformations.quaternion_from_matrix(table_rot_mat)
    EE_q = tf.transformations.quaternion_multiply(table_q, flip_x_q)
    # define resulting pose ready for the EE
    # todo (translation to the object center still needed)
    result = geometry_msgs.msg.PoseStamped()
    result.header.frame_id = "world"

    result.pose.position.x = best_place_pos[0]
    result.pose.position.y = best_place_pos[1]
    result.pose.position.z = best_place_pos[2]

    if debug:
        result.pose.position.x = best_place_pos[0] - place_poses_world[0][0] + 0.5
        result.pose.position.y = best_place_pos[1] - place_poses_world[0][1] + 0.5
        result.pose.position.z = best_place_pos[2]

    result.pose.orientation.x = EE_q[0]
    result.pose.orientation.y = EE_q[1]
    result.pose.orientation.z = EE_q[2]
    result.pose.orientation.w = EE_q[3]

    return result


def get_name_and_number_from_planning_scene_id(id):
    # find last hash in id if present
    split = id.split("#")
    # if id is a planning scene id
    if len(split) > 1:
        num = int(split.pop())
        return "#".join(split), num
    # id is just a name
    return id, None


if __name__=="__main__":
    #stack = [{"ac_object": -1, "object_id": -2, "stack_num": 2}, {"ac_object": -1, "object_id": -2, "stack_num": 1}]
    #res = [element["stack_num"] for element in stack ]
    #print(res)
    #if not 0 in res:
    #    print("not")
    #else:
    #    print("is")

    ############################################3

    #table_pose = geometry_msgs.msg.Pose()
    #table_pose.position.x = 0.0
    #table_pose.position.y = 0.0

    #obj_pose = geometry_msgs.msg.Pose()
    #obj_pose.position.x = 0.05 * math.cos(math.pi / 4.0)
    #obj_pose.position.y = 0.049 * math.cos(math.pi / 4.0)

    #print(is_pose_in_bounding_box(table_pose, obj_pose))

    print(get_valid_place_pose("WS01"))
