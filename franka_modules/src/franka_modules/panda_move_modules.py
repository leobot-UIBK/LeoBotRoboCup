import rospy
import sys
import copy
import roslib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import franka_movement_msgs.srv as franka_srv
# from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
import franka_msgs.srv as frankaMsg    # import franka service for set load


import numpy as np
from scipy.spatial.transform import Rotation as Rot
import tf
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion #quaternion calculation from euler angels
import os

import franka_modules.panda_grip_modules as gripmodules

scene = None

#%%
def panda_init():
    # initialize moveit comander and a rospy node
    # usage if you want to start the script in spyder
    cleaned_args = [a for a in sys.argv if not os.path.basename(__file__) in os.path.basename(__file__)]
    moveit_commander.roscpp_initialize(cleaned_args)

    #standard solution
    #moveit_commander.roscpp_initialize(sys.argv)

    # instantiate a RobotCommander object (Provides information such as the robots kinematic model and the robots current joint state)
    robot =  moveit_commander.RobotCommander()

    # instantiate a PlanningSceneInterface object (provides a remote interface for getting, setting and updating the robots internal understanding of the surrounding world)
    scene = moveit_commander.PlanningSceneInterface()

    # instantiate a moveGroup commander
    #group_name="panda_arm"
    #group_name="panda_arm_hand"
    #group_name="hand"
    moveGroupArm= moveit_commander.MoveGroupCommander("panda_arm")
    moveGroupArm.set_max_velocity_scaling_factor(0.8)
    # Group needed to manipulate the grasper
    moveGroupHand = moveit_commander.MoveGroupCommander("hand")

    # create a displayTrajectory Ros publisher which is used to display trajectories in Rviz
    display_trajectory_publisher=rospy.Publisher('/move_group/disply_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = moveGroupArm.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = moveGroupArm.get_end_effector_link()     # standard 'panda_link8'
    print("============ End effector link: %s" % eef_link)
    moveGroupArm.set_end_effector_link('panda_gripper_center')

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())

    return robot, scene, moveGroupArm, moveGroupHand

#%%
def addBoxEnvironment(name,pose,dimension,adscene):
 # adding a desks and boxes into the world
 # name = "deks" name of environment
 # pose = ["world", 0.5, 0.5, 0.5, np.pi/4, 0, 0] frame of reference, xyz position and xyz rotations
 # dimenstion
    EnvironmentPose = geometry_msgs.msg.PoseStamped()
    EnvironmentPose.header.frame_id = pose[0]
    EnvironmentPose.pose.position.x = pose[1]
    EnvironmentPose.pose.position.y = pose[2]
    EnvironmentPose.pose.position.z = pose[3]

    Quaternonlist=quaternion_from_euler(pose[4],pose[5],pose[6])  # calculates quaternions from euler angels Rotx,Roty,Rotz

    EnvironmentPose.pose.orientation.x = Quaternonlist[0]
    EnvironmentPose.pose.orientation.y = Quaternonlist[1]
    EnvironmentPose.pose.orientation.z = Quaternonlist[2]
    EnvironmentPose.pose.orientation.w = Quaternonlist[3]

    adscene.add_box(name, EnvironmentPose, size=(dimension[0], dimension[1], dimension[2]))

#%%
def addWallEnvironment(name,pose,adscene):
 # adding a wall into the world
 # name = "wall_num" name of environment
 # pose = ["world", 0.5, 0.5, 0.5, np.pi/4, 0, 0] frame of reference, xyz position and xyz rotations
    dimension=[0.02,1.25,0.3]
    EnvironmentPose = geometry_msgs.msg.PoseStamped()
    EnvironmentPose.header.frame_id = pose[0]
    EnvironmentPose.pose.position.x = pose[1]
    EnvironmentPose.pose.position.y = pose[2]
    EnvironmentPose.pose.position.z = pose[3]

    Quaternonlist=quaternion_from_euler(pose[4],pose[5],pose[6])  # calculates quaternions from euler angels Rotx,Roty,Rotz

    EnvironmentPose.pose.orientation.x = Quaternonlist[0]
    EnvironmentPose.pose.orientation.y = Quaternonlist[1]
    EnvironmentPose.pose.orientation.z = Quaternonlist[2]
    EnvironmentPose.pose.orientation.w = Quaternonlist[3]

    adscene.add_box(name, EnvironmentPose, size=(dimension[0], dimension[1], dimension[2]))

 #%%
def addAllWallsEnvironment(adscene):
 # adding all walls in scene
    dimension=[0.02,1.25,0.3]
    wallZ=dimension[2]/2
    orient0=[0,0,0]
    orient90=[0,0,-np.pi/2]
    orient45=[0,0,-np.pi/4]
    orientp45=[0,0,np.pi/4]

    sceneObjects=moveit_commander.PlanningSceneInterface.get_known_object_names(adscene)
    if  'wall0'  not in sceneObjects:

        try:
            wall0=[0.35+0.01,0.666+dimension[1]/2,wallZ]
            addWallEnvironment('wall0',['world']+wall0+orient0,adscene)

            wall1=[wall0[0]+0.01+0.44, wall0[1]+dimension[1]/2+0.44,wallZ]
            addWallEnvironment('wall1',['world']+wall1+orient45,adscene)

            wall2=[wall1[0]+0.44+dimension[1]/2, wall1[1]+0.44+dimension[0]/2,wallZ]
            addWallEnvironment('wall2',['world']+wall2+orient90,adscene)

            wall3=[wall2[0]+dimension[1],wall2[1],wallZ]
            addWallEnvironment('wall3',['world']+wall3+orient90,adscene)

            wall4=[wall3[0]+dimension[1],wall3[1],wallZ]
            addWallEnvironment('wall4',['world']+wall4+orient90,adscene)

            wall5=[wall4[0]+dimension[1],wall4[1],wallZ]
            addWallEnvironment('wall5',['world']+wall5+orient90,adscene)

            wall6=[wall5[0]+dimension[1]/2+0.01, wall5[1]-0.01-dimension[1]/2,wallZ]
            addWallEnvironment('wall6',['world']+wall6+orient0,adscene)

            wall7=[wall6[0], wall6[1]-dimension[1],wallZ]
            addWallEnvironment('wall7',['world']+wall7+orient0,adscene)

            wall8=[wall7[0]-dimension[1]/2,wall7[1]-dimension[1]/2-2.10-0.01,wallZ]
            addWallEnvironment('wall8',['world']+wall8+orient90,adscene)

            wall9=[wall8[0]-dimension[1],wall8[1],wallZ]
            addWallEnvironment('wall9',['world']+wall9+orient90,adscene)

            wall10=[wall9[0]-dimension[1],wall9[1],wallZ]
            addWallEnvironment('wall10',['world']+wall10+orient90,adscene)

            wall11=[wall10[0]-dimension[1],wall10[1],wallZ]
            addWallEnvironment('wall11',['world']+wall11+orient90,adscene)

            wall12=[wall11[0]-dimension[1]/2-0.44,wall11[1]+0.01+0.44,wallZ]
            addWallEnvironment('wall12',['world']+wall12+orientp45,adscene)

            ####
            wall13=[wall7[0]-1.75-dimension[1]/2,wall8[1]+1.2,wallZ]
            addWallEnvironment('wall13',['world']+wall13+orient90,adscene)

            wall14=[wall13[0]-dimension[1]/2-0.01,wall13[1]+dimension[1]/2,wallZ]
            addWallEnvironment('wall14',['world']+wall14+orient0,adscene)

            wall15=[wall14[0]-dimension[1]/2-0.01,wall14[1]+dimension[1]/2+0.01,wallZ]
            addWallEnvironment('wall15',['world']+wall15+orient90,adscene)



        except OSError as err:
           print("OS error: {0}".format(err))
           print('exception during all walls building')
        return True

#%%
def addPlaneEnvironment(name,adscene,pose=["world", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],normal=[0.0,0.0,1.0]):

 # adding a desks and boxes into the world
 # name = "deks" name of environment
 # pose = ["world", 0.5, 0.5, 0.5, np.pi/4, 0, 0] frame of reference, xyz position and xyz rotations
 # normal=[0,0,1]   normal vector of plane
    EnvironmentPose = geometry_msgs.msg.PoseStamped()
    EnvironmentPose.header.frame_id = pose[0]
    EnvironmentPose.pose.position.x = pose[1]
    EnvironmentPose.pose.position.y = pose[2]
    EnvironmentPose.pose.position.z = pose[3]

    Quaternonlist=quaternion_from_euler(pose[4],pose[5],pose[6])  # calculates quaternions from euler angels Rotx,Roty,Rotz

    EnvironmentPose.pose.orientation.x = Quaternonlist[0]
    EnvironmentPose.pose.orientation.y = Quaternonlist[1]
    EnvironmentPose.pose.orientation.z = Quaternonlist[2]
    EnvironmentPose.pose.orientation.w = Quaternonlist[3]

    adscene.add_plane(name, EnvironmentPose,normal)

#%%
def addCylinderEnvironment(name,pose,height,radius,adscene):
 # adding a desks and boxes into the world
 # name = "deks" name of environment
 # pose = ["world", 0.5, 0.5, 0.5, np.pi/4, 0, 0] frame of reference, xyz position and xyz rotations
 # height=1.0       height of cylinder
 # radius=1.0       radius of cylinder
    EnvironmentPose = geometry_msgs.msg.PoseStamped()
    EnvironmentPose.header.frame_id = pose[0]
    EnvironmentPose.pose.position.x = pose[1]
    EnvironmentPose.pose.position.y = pose[2]
    EnvironmentPose.pose.position.z = pose[3]

    Quaternonlist=quaternion_from_euler(pose[4],pose[5],pose[6])  # calculates quaternions from euler angels Rotx,Roty,Rotz

    EnvironmentPose.pose.orientation.x = Quaternonlist[0]
    EnvironmentPose.pose.orientation.y = Quaternonlist[1]
    EnvironmentPose.pose.orientation.z = Quaternonlist[2]
    EnvironmentPose.pose.orientation.w = Quaternonlist[3]

    adscene.add_plane(name, EnvironmentPose,height,radius)

#%%
def planning_joint_goal(move_group,setJointGoal):
 # move_group=arm  which group should be used
 # jointGoal=[0,0,0,0,0,0,0]
 # The go command can be called with joint values, poses, or without any
 # parameters if you have already set the pose or joint target for the group
    jointGoal = move_group.get_current_joint_values()
    for i in range(len(setJointGoal)):
        jointGoal[i]=setJointGoal[i]

    move_group.go(jointGoal, wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

#%%
def planning_cartesian_goal(move_group,cartesianGoal):
 # cartesianGoal = ["world", 0.5, 0.5, 0.5, np.pi/4, 0, 0] frame of reference, xyz position and xyz rotations
    cGoal = geometry_msgs.msg.PoseStamped()
    cGoal.header.frame_id = cartesianGoal[0]
    cGoal.pose.position.x = cartesianGoal[1]
    cGoal.pose.position.y = cartesianGoal[2]
    cGoal.pose.position.z = cartesianGoal[3]

    Quaternonlist=quaternion_from_euler(cartesianGoal[4],cartesianGoal[5],cartesianGoal[6])  # calculates quaternions from euler angels Rotx,Roty,Rotz

    cGoal.pose.orientation.x = Quaternonlist[0]
    cGoal.pose.orientation.y = Quaternonlist[1]
    cGoal.pose.orientation.z = Quaternonlist[2]
    cGoal.pose.orientation.w = Quaternonlist[3]

    move_group.set_pose_target(cGoal)
    ## Now, we call the planner to compute the plan and execute it.

    executionSuccess= move_group.go(wait=True)
    print('return value planing_cartesian_goal: '+ str(executionSuccess))

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()
    return executionSuccess


#%%

def planning_cartesian_path(move_group,cartesianPath):
 # cartesianPath=[P1,P2,...,Pn] =[[p1xdistance,p1ydistance,p1zdistance,p1rotx,p1roty,p1rotz],...,[pnxdistance,pnydistance,pnzdistance,pnrotx,pnroty,pnrotz]]
 # planing from point to point and orientation to orientation always from current configuration
    waypoints = []
    wpose = move_group.get_current_pose().pose     #current pose of ee (including current planing frame)

    for i in range(len(cartesianPath)):
        scale=1.0 #

        wpose.position.x += scale * cartesianPath[i][0]
        wpose.position.y += scale * cartesianPath[i][1]
        wpose.position.z += scale * cartesianPath[i][2]

        Quaternionlist=[wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
        eulerNow=list(euler_from_quaternion(Quaternionlist))

        eulerNow[0] += cartesianPath[i][3]
        eulerNow[1] += cartesianPath[i][4]
        eulerNow[2] += cartesianPath[i][5]

        Quaternonlist=quaternion_from_euler(eulerNow[0],eulerNow[1],eulerNow[2])  # calculates quaternions from euler angels Rotx,Roty,Rotz

        wpose.orientation.x = Quaternonlist[0]
        wpose.orientation.y = Quaternonlist[1]
        wpose.orientation.z = Quaternonlist[2]
        wpose.orientation.w = Quaternonlist[3]

        waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    # Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.
    # Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between
    # consecutive points in the resultingpath.
    # The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # move_group.execute(plan, wait=True) to execute
    return plan, fraction
#%%
def wait_for_state_update(objectName,adscene,box_is_known=True,box_is_attached=True,timeout=0):
    box_name = objectName
    #DEBUG, HOTFIX
    #box_is_known = True, box_is_attached = True

    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
        attached_objects = adscene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in adscene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
          return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.05)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False



def nextexecution():
    char_inp = raw_input("press y (yes) /n (no) ") # in python 3 this is only input()
    if char_inp in ['q', 'Q', 'd', 'D', 'n', 'N']:
        rospy.loginfo('Terminated because of user input.')
        stopflag=1
    else:
        stopflag=0
    return stopflag


#%%
def moveEE(moveGroupArm,movementEE=[0,0,0,0,0,0]):
    # movementEE=[x,y,z,rx,ry,rz]  movement in EE FRAME
    try:
        # actual end effector orientation
        currentrpy = moveGroupArm.get_current_rpy()
        # actual end effector orientation as rotation matrix
        currentR=tf.transformations.euler_matrix(currentrpy[0],currentrpy[1],currentrpy[2], axes='rxyz')[0:3,0:3]
        # recalculation of end effector movement in ee-Frame to the world frame
        movement2World=np.matmul(currentR,movementEE[0:3])
        movement2World2=list(movement2World)+movementEE[3:6]
        P=[]
        P.append(movement2World2)
        (plan, fraction)=planning_cartesian_path(moveGroupArm, P)
        #print('plan of moveEE, planning cartesian path '+str(plan))

        executeplan=moveGroupArm.execute(plan, wait=True) #to execute the plan
        rospy.sleep(2)
        rospy.loginfo('Endeffector Motion {}'.format(executeplan))

    except OSError as err:
        print("OS error: {0}".format(err))
        print('exception during ws building')

    return executeplan





#%%
def moveEE2seach(moveGroupArm,movetask=0):
    # movement in EE FRAME, to find objects
    # after positioning on a workstation (front, left, right, shelf top, shelf bottom)
    try:
        # movement to standard pose

        if movetask==0:
            if moveEE(moveGroupArm,movementEE=[0,0,0.05,0,0,0]):
                print("no movement done")


        elif movetask==1:
            if moveEE(moveGroupArm,movementEE=[0,0,-0.05,0,0,0]):
                print("movement done")


        elif movetask==2:
            if moveEE(moveGroupArm,movementEE=[0.05,0,0,0,0,0]):

                print("movement done")


        elif movetask==3:
            if moveEE(moveGroupArm,movementEE=[-0.05,0,0,0,0,0]):
                print("movement done")

        elif movetask==4:
            if moveEE(moveGroupArm,movementEE=[0,0,0,0,-np.pi/8.0,0]):

                print("movement done")


        elif movetask==5:
            if moveEE(moveGroupArm,movementEE=[0,0,0,0,np.pi/8,0]):
                print("movement done")


    except OSError as err:
        print("OS error: {0}".format(err))
        print('exception during ws building')

    return True


#%%
# to do
def pickplaceMove(scene,moveGroupArm,task,cartesianGoal,pre_gripper_width,post_gripper_width,distance,object_id,stack='',grippersimulation=True):
    # obj='1_F20_20) should be object stack name

    try:
        result = franka_srv.PickAndPlaceResponse.SUCCESS
        #cartesianGoal=["world"]+[ 0.5, 0.5, 0.5]+[np.pi/4, 0, 0]
        # goal point
        if not planning_cartesian_goal(moveGroupArm,cartesianGoal):
            result = franka_srv.PickAndPlaceResponse.INITIAL_POSE_ERROR
            return result

        if stack != '':
            makeObjStack(scene, stack, moveGroupArm, onoffToggle='off')

        if task=='pick':
            # finger open
            width=pre_gripper_width
            gripmodules.pandagripper_move_client(width, grippersimulation=grippersimulation)
            # move down to object
            movementEE=[0,0,distance]+[0,0,0]
            if not moveEE(moveGroupArm,movementEE):
                result = franka_srv.PickAndPlaceResponse.PICK_PLACE_APPROACH_ERROR
                return result

            # finger close
            width=post_gripper_width
            force=80.0
            if object_id != '':
                moveGroupArm.attach_object(object_id,link_name='panda_gripper_center',touch_links=['panda_leftfinger','panda_rightfinger'])
                wait_for_state_update(object_id,scene)
                #set object mass for grasp
                objectToggle=True

            # setObjectLoad(objectToggle, object_id)
            grasp_result = gripmodules.pandagripper_grasp_client(width,force,speed=0.5,epsilon_inner=0.008,epsilon_outer=0.02,grippersimulation=grippersimulation)
            if not(grasp_result):
                result = 4 # todo
                return result
        else:
            print('no finger movement for place')
            # move down to object
            movementEE=[0,0,distance]+[0,0,0]
            print('driven down for EE')
            if not moveEE(moveGroupArm, movementEE):
                result = franka_srv.PickAndPlaceResponse.PICK_PLACE_APPROACH_ERROR
                print('drive down not done')
                return result
            # finger open
            print('for place finger open')
            width = post_gripper_width
            gripmodules.pandagripper_move_client(width, grippersimulation=grippersimulation)
            if object_id != '':
                moveGroupArm.detach_object(object_id)
                wait_for_state_update(object_id,scene)
                objectToggle=False

            # setObjectLoad(objectToggle, object_id)


        # move up
        movementEE=[0,0,-distance]+[0,0,0]
        if not moveEE(moveGroupArm,movementEE):
            result = franka_srv.PickAndPlaceResponse.PICK_PLACE_RETREAT_ERROR
            return result

        if stack != '':
            makeObjStack(scene, stack, moveGroupArm, onoffToggle='on')

    except OSError as err:
       print("OS error: {0}".format(err))
       print('exception during pic and place move')

    return result


#%%
def makeArenaWorkStation(scene,WSname):
    # make individual workstations in the arena using parameter server
    # scene= scenename
    # WSname= Name of Workstation as string 'WS01'


    sceneObjects=moveit_commander.PlanningSceneInterface.get_known_object_names(scene)
    if  WSname not in sceneObjects:

        try:
            wsHeight=rospy.get_param('/workstations/world2'+WSname+'/height')
            wsXdim=0.2
            wsYdim=0.8

            wsPostion=rospy.get_param('/workstations/world2'+WSname+'/position')
            wsPostion2=[wsPostion[0],wsPostion[1],wsPostion[2]-wsHeight/2]
            wsOrientation=rospy.get_param('/workstations/world2'+WSname+'/orientation')
            wsPose=['world']+wsPostion2+wsOrientation
            addBoxEnvironment(WSname,pose=wsPose,dimension=[wsXdim,wsYdim,wsHeight],adscene=scene)
            wait_for_state_update(WSname,scene)

            deadzoneXdim=0.1
            deadzoneOrientation=wsOrientation
            deadzoneRot=tf.transformations.euler_matrix(deadzoneOrientation[0],deadzoneOrientation[1],deadzoneOrientation[2], axes='sxyz')[0:3,0:3]
            deadzonePostion=list(np.array(wsPostion2)+np.matmul(deadzoneRot,np.array([0.15,0.0,0.0]))     )
            deadzonePose=['world']+deadzonePostion+deadzoneOrientation
            addBoxEnvironment(WSname+'Deadzone',pose=deadzonePose,dimension=[deadzoneXdim,wsYdim,wsHeight],adscene=scene)
            wait_for_state_update(WSname+'Deadzone',scene)


            if WSname=='SH01bottom':
                shSideXsize=0.3
                shSideYsize=0.005
                shSideZsize=0.2
                wsPostion3=list(np.array(wsPostion2)+np.matmul(deadzoneRot,np.array([0.25,0.40025,0.150]))     )
                sideleftpose=['world']+wsPostion3+wsOrientation
                addBoxEnvironment(WSname+'sideleft',pose=sideleftpose,dimension=[shSideXsize,shSideYsize,shSideZsize],adscene=scene)
                wait_for_state_update(WSname+'sideleft',scene)

                wsPostion4=list(np.array(wsPostion2)+np.matmul(deadzoneRot,np.array([0.25,-0.40025,0.150]))     )
                siderightpose=['world']+wsPostion4+wsOrientation
                addBoxEnvironment(WSname+'sideright',pose=siderightpose,dimension=[shSideXsize,shSideYsize,shSideZsize],adscene=scene)
                wait_for_state_update(WSname+'sideright',scene)

            if WSname=='SH01top':
                wsPostion5=list(np.array(wsPostion2)+np.matmul(deadzoneRot,np.array([-0.10025,0.0,0.005]))     )
                frontpose=['world']+wsPostion5+wsOrientation
                addBoxEnvironment(WSname+'front',pose=frontpose,dimension=[0.0025,0.8,0.015],adscene=scene)
                wait_for_state_update(WSname+'front',scene)

            if WSname=='WS03' or WSname=='WS06' or WSname=='WS07' or WSname=='PP01' or WSname=='SH01top' or WSname=='SH01bottom':
                deadzone2Xdim=0.2
                deadzone2Orientation=wsOrientation
                deadzone2Rot=tf.transformations.euler_matrix(deadzone2Orientation[0],deadzone2Orientation[1],deadzone2Orientation[2], axes='sxyz')[0:3,0:3]
                deadzone2Postion=list(np.array(wsPostion2)+np.matmul(deadzoneRot,np.array([0.15+0.15,0.0,0.0]))     )
                deadzone2Pose=['world']+deadzone2Postion+deadzone2Orientation
                addBoxEnvironment(WSname+'Deadzone2',pose=deadzone2Pose,dimension=[deadzone2Xdim,wsYdim,wsHeight],adscene=scene)
                wait_for_state_update(WSname+'Deadzone2',scene)

        except OSError as err:
           print("OS error: {0}".format(err))
           print('exception during ws building')

#%%
def makeCompetitionArena(scene):
# make the whole competition arena using individual makeArenaWorkStation function
    try:
        for i in range(1,10):
            makeArenaWorkStation(scene,'WS0'+str(i))
        #SpecialWS
        makeArenaWorkStation(scene,'SH01bottom')
        makeArenaWorkStation(scene,'SH01top')
        makeArenaWorkStation(scene,'PP01')

        #addFloor
        #addBoxEnvironment('Floor',pose=['world',0,0,-0.01,0,0,0],dimension=[8,6,0.0001],adscene=scene)
        #wait_for_state_update('Floor',scene)
        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.5)
    except OSError as err:
       print("OS error: {0}".format(err))
       print('exception during ws all building')
    return True

#%%
# make the object stack on the back of the robot, toggle on and off
def makeObjStack(scene, StacknameID, moveGroupArm, onoffToggle='on'):
# Builds individual object stack from parameter server, defined by stackName, for example 1_F20_20 or 1_M20 or 2_M20_100
# onoffToggel to turn it on, or off
    try:
        #sceneObjects=moveit_commander.PlanningSceneInterface.get_known_object_names(scene)
        stackObject=moveit_commander.PlanningSceneInterface.get_attached_objects(scene)
        stackNumber=rospy.get_param('objects_stack/objStack'+StacknameID+'/Stacknumber')

        stackinScene='stack_'+str(stackNumber)
        if stackinScene not in stackObject:
            if onoffToggle=='on':
            #if stackName=='ObjStack':
            #    # one box for all
            #    stackXsize=0.4
            #    stackYsize=0.5
            #    stackZsize=0.085
            #   addBoxEnvironment(stackName,pose=['panda_link0',-0.36,0,0.210+stackZsize/2,0,0,0],dimension=[stackXsize,stackYsize,stackZsize],adscene=scene)
            #    scene.attach_box('panda_link0', stackinScene, touch_links='panda_link0')
            #    #moveGroupArm.attach_object(stackName, link_name='panda_link0',touch_links=['panda_link0'])
            #    wait_for_state_update(stackinScene, scene)
            #else:
                # individual stacks
                stackSize=rospy.get_param('objects_stack/objStack'+StacknameID+'/size')
                stackPostion=rospy.get_param('objects_stack/objStack'+StacknameID+'/position')
                stackOrientation=rospy.get_param('objects_stack/objStack'+StacknameID+'/orientation')

                stackGmPose = geometry_msgs.msg.PoseStamped()
                stackGmPose.header.frame_id = "panda_link0"
                stackGmPose.pose.position.x = stackPostion[0]
                stackGmPose.pose.position.y = stackPostion[1]
                stackGmPose.pose.position.z = 0.210+stackSize[2]/2.0
                stackGmPoseQuaternionList = tf.transformations.quaternion_from_euler(0,0,stackOrientation[2],axes='rxyz')
                stackGmPose.pose.orientation.x = stackGmPoseQuaternionList[0]
                stackGmPose.pose.orientation.y = stackGmPoseQuaternionList[1]
                stackGmPose.pose.orientation.z = stackGmPoseQuaternionList[2]
                stackGmPose.pose.orientation.w = stackGmPoseQuaternionList[3]

                scene.attach_box('panda_link0', stackinScene,stackGmPose,stackSize)
                wait_for_state_update(stackinScene,scene)

        if onoffToggle=='off':
            scene.remove_attached_object('panda_link0', name=stackinScene)
            scene.remove_world_object(stackinScene)
            wait_for_state_update(stackinScene,scene)
    except OSError as err:
       print("OS error: {0}".format(err))
    return True


#%%
def makeAllObjStack(scene,moveGroupArm,onoffToggle='on'):
# Builds all object stacks from parameter server
# onoffToggel to turn all on, or all off
    try:
        allstacksName = rospy.get_param('objects_stack')
        #allstackNumber = rospy.get_param('objects_stack/objStack' + stackName + '/Stacknumber')
        # name
        for StackName in allstacksName:
            StacknameID=StackName.replace('objStack','')
            makeObjStack(scene,StacknameID,moveGroupArm,onoffToggle)
        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.5)
    except OSError as err:
        print("OS error: {0}".format(err))



#%%
def move2objStackpose(object2find,zDistance,moveGroupArm,scene,grippersimulation=True):
    try:

        makeObjStack(scene,object2find,moveGroupArm,onoffToggle='off')
        print('stack position free')
        poseStack=rospy.get_param('objects_stack/objStack'+object2find)
        stackSize=rospy.get_param('objects_stack/objStack'+object2find+'/size')

        # Gripper finger movement to object size
        # as_simulation has to be set
        width=stackSize[1]/2.0+0.02
        gripmodules.pandagripper_move_client(width, grippersimulation=True)


##################################
        # Object orientation defined by stack
        psorientation=[ poseStack['orientation'][0], poseStack['orientation'][1], poseStack['orientation'][2]]
        #print('psorientation: ' + str(psorientation))
        # here the problem appears, Rotation Matrix is not correct/ wrong euler angels ??
        # PROBLEM SOLVED!!!!  axes='sxyz' vs axes='rxyz'   static vs rotational
        psRot=tf.transformations.euler_matrix(psorientation[0],psorientation[1],psorientation[2], axes='rxyz') [0:3,0:3]
        print('psRotFalsch: ' + str(psRot))
##############################
        #RzRyFromStack = [poseStack['RzRy'][0], poseStack['RzRy'][1]]
        #obStackRotz=tf.transformations.rotation_matrix(RzRyFromStack[0]*np.pi/180,  [0,0,1], point=None)[0:3,0:3]
        #obStackRoty=tf.transformations.rotation_matrix(RzRyFromStack[1]*np.pi/180,  [0,1,0], point=None)[0:3,0:3]
        #psRot2=np.matmul(obStackRotz,obStackRoty)
        #print('psRotRichtig: ' + str(psRot2))

        #rotation of the end effector to be oposite of object, roty(pi)*rotz(pi)
        # ll=np.matmul([[1.0,0.0,0.0],[0.0,-1.0,0.0],[0.0,0.0,1.0]],[[1.0,0.0,0.0],[0.0,-1.0,0.0],[0.0,0.0,1.0]])
        # lleul=tf.transformations.euler_from_matrix(ll)
        # roty and rotz
        #psRotEE=np.matmul(psRot,      ([[ 1.0,  0,  0],
        #                                [ 0, -1.0,  0],
        #                                [ 0,  0, -1.0]]))

        # rotation of panda hand with just roty
        Rroty=np.array(([[ -1.0,  0,  0],
                        [ 0, 1.0,  0],
                        [ 0,  0, -1.0]]))
        psRotEE=np.matmul(psRot,Rroty)
        print('psRotEE: ' + str(psRotEE))
        # rotation of panda hand as euler angles
        # psorientationEE= (np.array(tf.transformations.euler_from_matrix(psRotEE, axes='sxyz')) * np.array([1,0,1])).tolist()
        psorientationEE = tf.transformations.euler_from_matrix(psRotEE, axes='sxyz')
        print('psorientationEE: ' + str(psorientationEE))

        psorientationEE1 = [3.0003, -0.3203, 2.7173]
        print('psorientationEE1: ' + str(psorientationEE1))

        # stack position in world frame
        positionStack=poseStack['position']

        # positon of ee= positon of stack plus additional z distance in world frame

        positionStack2=np.array(positionStack)+np.matmul(psRotEE,np.array([0.0,0.0,zDistance]))

        # final pose of EE in world frame to plan_cartesian_goal
        poseStackPose=['panda_link0']+list(positionStack2)+list(psorientationEE)
        print(poseStackPose)

        planSuccess=planning_cartesian_goal(moveGroupArm,poseStackPose)        # if eef_link='panda_hand'
        return planSuccess

    except OSError as err:
       print("OS error: {0}".format(err))
       print('exception during object stack move')

    return True


def setObjectLoad(objectToggle,object_id):
    service = "/franka_control/set_load"
    rospy.wait_for_service(service)

    if objectToggle == True:
        name, id = get_name_and_number_from_planning_scene_id(object_id)
        objectMass = rospy.get_param('/objects_manipulation/' + name + '/mass')
    else:
        objectMass = 0

    loadSet = frankaMsg.SetLoadRequest()
    loadSet.mass = objectMass
    loadSet.F_x_center_load = [0, 0, 0]
    loadSet.load_inertia= [0, 0, 0, 0, 0, 0, 0, 0, 0]
    try:
        rospy.loginfo("Load set to m= " + str(objectMass))
        SetObjMassAsLoad = rospy.ServiceProxy(service, frankaMsg.SetLoad)
        response = SetObjMassAsLoad.call(loadSet)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))


def get_name_and_number_from_planning_scene_id(id):
    # find last hash in id if present
    split = id.split("#")
    # if id is a planning scene id
    if len(split) > 1:
        num = int(split.pop())
        return "#".join(split), num
    # id is just a name
    return id, None




