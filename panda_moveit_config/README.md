# Franka Emika Panda MoveIt! Config Package

The Panda robot is the flagship MoveIt! integration robot used in the MoveIt! tutorials.
Any changes to MoveIt! need to be propagated into this config fast, so this package
is co-located under the ``ros-planning`` Github organization here.

## Changes by Patrick
(paddy-hofmann@web.de) \
To add use moveit together with libfranka I have created a virtual link panda_gripper_center according to this post: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/ \
Therefore I have changed following files:
- .setup_assistant
- config/hand.xacro
- config/panda_arm_hand.srdf.xacro
- launch/demo.launch
- launch/move_group.launch
- launch/planning_context.launch
- launch/panda_moveit.launch
- package.xml