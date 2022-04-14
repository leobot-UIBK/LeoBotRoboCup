#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
import actionlib
import tf.transformations
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


import tf2_ros
import geometry_msgs

'''
This function gets the goal workstation in form of a string. Then reads from the rosparam-server the midpoint location
of this workstation. Then it gets the corresponding location where the robot should stand in resepct to the station.
'''

class Goal:
    ''' gets the goal coordinates for a workstation'''

    def get_robot_goal(self, workstation):
        ''' This function returns the goal position and orientation for the robot when he has to grasp an object.
        That means transformation from odometry frame (0,0,0) -> workstation -> goal position '''
        
        try: 
            self.ws_pose = rospy.get_param("/workstations/world2{}/position".format(workstation))
            str_world = 'world2'
        except: 
            self.ws_pose = rospy.get_param("/workstations/{}/position".format(workstation))
            str_world = ''
        self.ws_orien = rospy.get_param("/workstations/" + str_world + "{}/orientation".format(workstation))
        self.ws2robo_pose_str = rospy.get_param("/workstations/" + str_world + "{}/robotPose".format(workstation))
        self.ws2robo_pose = rospy.get_param("/workstations/{}/position".format(self.ws2robo_pose_str))
        self.ws2robo_orien = rospy.get_param("/workstations/{}/orientation".format(self.ws2robo_pose_str))
        
        R_WS = tf.transformations.euler_matrix(self.ws_orien[0],self.ws_orien[1], self.ws_orien[2], axes='rxyz')    # xyz euler angles 
        R_Robo = tf.transformations.quaternion_matrix(self.ws2robo_orien) # quaternions
        R_W = np.matmul(R_WS, R_Robo)
        self.world2robo_orien = tf.transformations.quaternion_from_matrix(R_W)
        ''' create euler matrix from ws orientation'''
        self.ws_orien_mat = tf.transformations.euler_matrix(self.ws_orien[0],self.ws_orien[1],self.ws_orien[2], axes='rxyz') # xyz euler angles
        
        self.ws2robo_transformed = np.matmul(self.ws_orien_mat[0:3,0:3], np.array(self.ws2robo_pose))

        self.goal_pose = [0,0,0]

        self.goal_pose[0] = self.ws_pose[0] + self.ws2robo_transformed[0]
        self.goal_pose[1] = self.ws_pose[1] + self.ws2robo_transformed[1]
        self.goal_pose[2] = self.ws_pose[2] + self.ws2robo_transformed[2]
        if workstation == 'WS09':
            self.goal_pose[0] += 0.05

        self.goal = self.goal_pose + self.world2robo_orien.tolist() # append orientation to pose
        return(self.goal)

def movebase_client(goal_pose):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_pose[0]
    goal.target_pose.pose.position.y = goal_pose[1]
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = goal_pose[3]
    goal.target_pose.pose.orientation.y = goal_pose[4]
    goal.target_pose.pose.orientation.z = goal_pose[5]
    goal.target_pose.pose.orientation.w = goal_pose[6]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def move_base_simple(goal_pose):
    pub = rospy.Publisher('/move_base/goal', PoseStamped)

    rospy.init_node('move_goal_publisher', anonymous = True)
    
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = goal_pose[0]
    goal.pose.position.y = goal_pose[1]
    goal.pose.position.z = goal_pose[2]
    goal.pose.orientation.x = goal_pose[3]
    goal.pose.orientation.y = goal_pose[4]
    goal.pose.orientation.z = goal_pose[5]
    goal.pose.orientation.w = goal_pose[6]

    pub.publish(goal)
    rospy.loginfo('Published the new goal position')
    return(goal)

if __name__=="__main__":
    
    #print(Goal('WS08').get_robot_goal())
    goal_pose = Goal().get_robot_goal('PP01')
    #move_base_simple(goal_pose)
    
    rospy.init_node('movebase_client_py')
#    result = movebase_client(goal_pose)
#    if result:  
#        rospy.loginfo("Goal execution done!")
    
    broadcaster_tf = []
    static_transformStamped = []
    Workstations = ["WS05"] # , "WS02","WS03", "WS04", "WS05", "WS06", "WS07", "WS08", "WS09", "SH01top", "SH01bottom", "PP01"]
    i = 0
    for Workstation in Workstations:    
        broadcaster_tf += [tf2_ros.StaticTransformBroadcaster()]
        static_transformStamped += [geometry_msgs.msg.TransformStamped()]
        static_transformStamped[i].header.frame_id = 'world'
        static_transformStamped[i].child_frame_id = Workstation
        static_transformStamped[i].transform.translation.x = Goal().get_robot_goal(Workstation)[0]
        static_transformStamped[i].transform.translation.y = Goal().get_robot_goal(Workstation)[1]
        static_transformStamped[i].transform.translation.z = Goal().get_robot_goal(Workstation)[2]
        
        static_transformStamped[i].transform.rotation.x = Goal().get_robot_goal(Workstation)[3]
        static_transformStamped[i].transform.rotation.y = Goal().get_robot_goal(Workstation)[4]
        static_transformStamped[i].transform.rotation.z = Goal().get_robot_goal(Workstation)[5]
        static_transformStamped[i].transform.rotation.w = Goal().get_robot_goal(Workstation)[6]
        broadcaster_tf[i].sendTransform(static_transformStamped [i])
        print(static_transformStamped[i])
        i +=1
    
    
    
    
#    for Workstation in Workstations:    
#        broadcaster_tf +=[tf2_ros.StaticTransformBroadcaster()]
#        static_transformStamped += [geometry_msgs.msg.TransformStamped()]
#        static_transformStamped[i].header.frame_id = 'world'
#        static_transformStamped[i].child_frame_id = Workstation
#        static_transformStamped[i].transform = Goal().get_robot_goal(Workstation)
#        broadcaster_tf[i].sendTransform(static_transformStamped[i])
#        i +=1
    
    rospy.spin()
            