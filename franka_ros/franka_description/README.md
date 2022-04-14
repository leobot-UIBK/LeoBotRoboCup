## Changes by Patrick
(paddy-hofmann@web.de) \
To add use moveit together with libfranka I have created a virtual link panda_gripper_center according to this post: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/ \
Therefore I have changed following files:
- robots/hand.urdf.xacro
- robots/hand.xacro
- robots/panda_arm_hand.urdf.xacro