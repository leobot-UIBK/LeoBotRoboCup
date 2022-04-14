import rospy
import actionlib
from copy import deepcopy
from sensor_msgs.msg import JointState
import moveit_commander
import moveit_msgs.msg

from franka_gripper.msg import ( GraspAction, GraspGoal, 
                                 HomingAction, HomingGoal,   
                                 MoveAction, MoveGoal,
                                 StopAction, StopGoal,
                                 GraspEpsilon )

def pandagripper_homing_client(grippersimulation=True):
    if not grippersimulation:
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("/franka_gripper/homing", HomingAction)
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("GripperInterface: Waiting for gripper action servers... HOMING")
        client.wait_for_server()
        rospy.loginfo("GripperInterface: Gripper action servers HOMING found! ")
        # Creates a goal to send to the action server.
        goal = HomingGoal()
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        result=client.wait_for_result(rospy.Duration(15.))
        # Prints out the result of executing the action
        return client.get_result()
    else:
        moveGroupHand = moveit_commander.MoveGroupCommander("hand")
        setJointGoal=[0.04,0.04]
        #jointGoal = move_group.get_current_joint_values()
        moveGroupHand.go(setJointGoal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        moveGroupHand.stop()
        return True

def pandagripper_grasp_client(width,force,speed=0.05,epsilon_inner=0.005,epsilon_outer=0.005,grippersimulation=True):
    if not grippersimulation:
        # An object is considered grasped if the distance `d` between the gripper fingers satisfies
        # (width - epsilon\_inner) < d < (width + epsilon\_outer)`.
        # Sends the goal to the action server.
        #  width :Size of object to grasp [m]   #min=0.0001, max=0.08
        # force : Grasping force [N] #min= 0.01, max=70N
        # spped: Closing speed [m/s]     #max 0.05
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon_inner,outer=epsilon_outer)
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        result = client.wait_for_result(rospy.Duration(15.))
        # Prints out the result of executing the action
        return client.get_result()
    else:
        moveGroupHand = moveit_commander.MoveGroupCommander("hand")
        setJointGoal = [width/2.0,width/2.0]
        # jointGoal = moveGroupHand.get_current_joint_values()
        moveGroupHand.go(setJointGoal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        moveGroupHand.stop()
        return True
    


def pandagripper_stop_client(grippersimulation=True):
    if not grippersimulation:
        
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("/franka_gripper/stop", StopAction)
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("GripperInterface: Waiting for gripper action servers... STOP")
        client.wait_for_server()
        rospy.loginfo("GripperInterface: Gripper action servers STOP found! ")
        # Creates a goal to send to the action server.
        goal = StopGoal()
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        result=client.wait_for_result(rospy.Duration(15.))
        # Prints out the result of executing the action
        return client.get_result()
    else:
        moveGroupHand = moveit_commander.MoveGroupCommander("hand")
        setJointGoal = moveGroupHand.get_current_joint_values()
        moveGroupHand.go(setJointGoal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        moveGroupHand.stop()
        return True


def pandagripper_move_client(width, speed=0.05, grippersimulation=True):
    if not grippersimulation:
        
        # width :Size of object to open finger [m]   #min=0.0001, max=0.08
        # spped: Closing speed [m/s]     #max 0.05
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo("GripperInterface: Waiting for gripper action servers... STOP")
        client.wait_for_server()
        rospy.loginfo("GripperInterface: Gripper action servers STOP found! ")
        # Creates a goal to send to the action server.
        goal = MoveGoal()
        goal.width = width
        goal.speed = speed
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        result=client.wait_for_result(rospy.Duration(15.))
        # Prints out the result of executing the action
        return client.get_result()
    else:
        moveGroupHand = moveit_commander.MoveGroupCommander("hand")
        setJointGoal=[width/2.0,width/2.0]
        #jointGoal = moveGroupHand.get_current_joint_values()
        moveGroupHand.go(setJointGoal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        moveGroupHand.stop()
        return True       

