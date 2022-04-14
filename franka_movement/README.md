# Franka Movement
Author: Patrick Hofmann <paddy-hofmann@web.de>

## Structure

- /apps -> executables e.g. ros nodes
- /launch -> launch files
- /extern -> external libraries/submodules
- /include -> header files
- /simulation -> simulation related files
- /src -> project sources

## Build
- Install dependencies with rosdep: rosdep install franka_movement --ignore-src -y
- To install the ros package of the realsense camera run: sudo apt-get install ros-melodic-realsense2-camera 
- You can activate rviz visualisation by setting the <bool visualize> argument in the startup of the FrankaNode node to 1.
- This package depends on the franka_movement_msgs package. 
During the first build it could happen that these packages are build in the wrong order.
So if you get any errors according to ros messages make sure to build franka_movement_msgs separate before franka_movement.
However this should be no problem if you are using catkin build tools instead of catkin_make.
- For some reasons you can't install eigen_conversions via rosdep so run: sudo apt-get install ros-noetic-eigen-conversions

This package also depends on (a version of each is provided in the robocup project):
- franka_ros (use the github repo and customize it)
- panda_moveit_config (use the github repo and customize it)


## Run
Use the **franka_node.launch** file to run this package.
If you want to use a RVIZ simulation instead of the real robot set the as_simulation argument to false.
More arguments can be found in the launchfile.

If you get an error similar to `material 'aluminum' is not unique.` this is because in the modified panda urdf are two 
cameras included and both are including these materials.
Go got your local realsense2_description folder and comment the following line in `_d415.urdf.xacro`:

    <!--xacro:include filename="$(find realsense2_description)/urdf/_materials.urdf.xacro" /-->

## Ros topic/service description
### Arm Movement
All arm movement services of this packed run under the frankaNode node.
- **/franka_move_node/move_joints_relative** (service): Moves the robot joints by the given joint positions relative to the currrent position.
- **/franka_move_node/move_joints** (service): Moves the robot joints to the given joint position.
- **/franka_move_node/move_to_pose** (service): Moves the end-effector to the given pose.
- **/franka_move_node/move_to_named** (service): Moves the arm in a predefined named pose. TODO available poses
- **/franka_move_node/move_to_home** (service): Moves the arm in the predefined home pose.
- **/franka_move_node/pick** (service): Uses a planning scene object id and a moveit_msgs/Grasp message to perform a pick of the desired object.
  - As result the moveit error code is forwarded. Check the official documentation for more information.
- **/franka_move_node/basic_pick** (service): Uses information of an object that is uploaded to the ros param server to perform the pick.
  - The arm will automatically rotate in a way to grasp the object with a good approachable rotation (see `Arm::shortest_grasp_rotation()` for more information).
  - If the flag constrain_grasp_width is set the gripper will only open as wide as needed to perform the grasp.
    This can be useful if the object is surrounded by other objects.
  - As supported_surface a planning scene id of an object that is not considered during collision checks can be passed. 
  - The perfect grasp parameters (grasp width, translation and orientation) are therefore set automatically.
  - As result the moveit error code is forwarded. Check the official documentation for more information.
  - Note that the object has to be added to the planning scene with a unique id (within the planning scene) as name.
    You can find more information in the section Planning Scene ID.
  - On the param server the objects need to be available under the namespace `/objects_manipulation` with a unique name of the object as primary id.
    Then the following fields must be set:
    - `/grasp_point`: (TODO not implemented yet) (double[3]) A translation in xyz (m) from the object center to the optimal point to grasp (eg. center of mass or 0 if not needed).
    - `/grasp_width`: (double) The width of the object at the grasp point (m). This can differ from the width of the bounding box.

- **/franka_move_node/place** (service): Uses a planning scene id and a `moveit_msgs/PlaceLocation` to perform a place motion of an object.
  - As result the moveit error code is forwarded. Check the official documentation for more information.
- **/franka_move_node/basic_place** (service): Uses information of an object that is uploaded to the ros param server to perform the place.
  - The arm will automatically rotate in a way to grasp the object with a good approachable rotation (see `Arm::shortest_grasp_rotation()` for more information).
  - If the flag constrain_grasp_width is set the gripper will only open as wide as needed to perform the place.
    This can be useful if the object is surrounded by other objects.
  - As supported_surface a planning scene id of an object that is not considered during collision checks can be passed.
  - For more information about the parameters that need to be set on the ros param server see the documentation of the `basic_pick` service above.
  - As result the moveit error code is forwarded. Check the official documentation for more information.


### Planning Scene
All planing scene related services run under the **planing_scene_node**.
- **/planing_scene_node/collision_object** (service): Add/Remove a collision **moveit_msgs/CollisionObject.msg**.
  - Make sure the id of the collision object is set.
  - Use the operation of the **franka_movement_msgs/CollisionObject.srv**. 
    The operation of the collision object will be overridden.
    - ADD: Adds an object to the scene. 
      If the object already exists it will be moved.
    - REMOVE: Removes an object from the scene.
- **/planing_scene_node/attached_collision_object** (service): Add/Attach/Detach/Remove an **moveit_msgs/AttachedCollisionObject.msg**
  - Make sure the id of the collision object is set.
  - Use the operation of the **franka_movement_msgs/AttachedCollisionObject.srv**.
    The operation of the collision object will be overridden.
    - ADD: Adds an attached object to the scene. 
      If a collision object with the same id exists this object will be removed.
      You can specify the linked joint in the **moveit_msgs/AttachedCollisionObject** msg.
    - REMOVE: Removes an object from the scene. 
    - DETACH: Detaches the object. It will be added as normal collision object.
- **/planing_scene_node/get_collision_object_by_id** (service): Returns the `moveit_msgs/CollisionObject` linked to a given planning scene ID.
- **/planing_scene_node/add_collision_object_by_id** (service): If the type of an object is known to the ros param server 
  to add an object to the planning scene only the ros param id of the object and the pose where the object should be added is needed.
  - To use this function the following parameter needs to be provided on the param server 
    (see the documentation of `/franka_move_node/basic_pick` for more information about how to properly add an object to the param server):
    - `/boundingbox`: (double[3]) A box that fully covers the object with the objects middle point as box middle point.\
  - If the object to add already exists (object of same type close to the given location) the existing object will be replaced.
  - This service returns the planning scene ID of the added/replaced object.
    Take a look at the Planning Scene ID section for more information.

### Planning Scene ID
In the planing scene it is required that every object has a unique ID.
Since an object of the same type could occur multiple times in an arena, addressing an object by its name is not sufficient. 
Therefore we use the following convention for planning scene IDs. 
The id has to be the name of the object (eg. the name used on the param server) and a number (separated by a `#`) to make the object distinguishable to other (existing) objects of the same type.
An id could look like this `Bearing_Box#3`.

## TODO
- The pointcloud is currently disabled. To use it you may need to reduce the camera framerate since the leobot main pc 
doesn't has a GPU.
- Currently, planing scene objects are sometimes added by a python interface and sometimes a cpp interface. 
This should be reduced to one general interface.
- The grasping is done in with a python interface but general arm motion is handled in a cpp interface.
This should be generalised.
- Implement the grasp point feature (in basic_pick and basic_place)

## Known Issues 
- check catkin dependencies in CMakeLists.txt. Are they really all needed?

- add interface to adjust arm movement speed

- There is no way to just detach/attach an existing object without sending the correct position and shape.

- If a timout occures during execution you can change the allowed_execution_duration_scaling value in panda_moveit_config/launch/trajectory_execution.launch.xml

- If the speed of the robot is to slow you can change the joint velocity limits in panda_moveit_config/config/joint_limits.yaml
 
- Added Collision objects are moving relative to the panda_link0 frame -> they are not static in the world