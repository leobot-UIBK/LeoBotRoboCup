import rospy
import geometry_msgs
import tf.transformations as tft
import franka_movement_msgs.srv
import math

def move_home():
    service = "/franka_move_node/move_to_home"
    rospy.wait_for_service(service)

    franka_home = franka_movement_msgs.srv.FrankaHomeRequest()

    try:
        call_home = rospy.ServiceProxy(service, franka_movement_msgs.srv.FrankaHome)
        response = call_home.call(franka_home)
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def example_pose():
    poseStamped = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = "panda_link0"

    pose = geometry_msgs.msg.Pose()
    pose.position.x = -0.3
    pose.position.y = 0.0
    pose.position.z = 0.25

    q = tft.quaternion_from_euler(0.0, math.pi, 0.0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    poseStamped.pose = pose

    return poseStamped

def move_to_pose(poseStamped):
    service = "/franka_move_node/move_to_pose"
    rospy.wait_for_service(service)

    franka_pose = franka_movement_msgs.srv.FrankaPoseRequest()
    franka_pose.poseStamped = poseStamped

    try:
        call_move = rospy.ServiceProxy(service, franka_movement_msgs.srv.FrankaPose)
        response = call_move.call(franka_pose)
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    move_to_pose(example_pose())
    #move_home()