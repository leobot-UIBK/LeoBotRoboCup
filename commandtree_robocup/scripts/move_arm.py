#! /usr/bin/env python

import rospy

import actionlib

from commandtree_msgs.msg import *



class MoveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = ACT_Move_armFeedback()
    _result = ACT_Move_armResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ACT_Move_armAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        success = True

        # get the position of the goal
        rospy.loginfo('%s: Move to Goal: %s' %
                      (self._action_name, goal.information.task))
        rospy.sleep(1)



        if success:
            # current position
            self._result.result.task = "arm has moved"

            rospy.loginfo('%s: Moved to %s' % (self._action_name, goal.information.task))
            rospy.sleep(1)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('move')
    server = MoveAction(rospy.get_name())
    rospy.spin()
