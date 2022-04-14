## leobot_config package
In this package various configuration and informatins are stored. In the object directory yaml files with all objects, which need to be manipulated are stored that can be loaded in the bringup of the robot. The object.yaml files include:  
 * boundingbox: The cuboid in which the object fits.  
 * description: name of the object to identify it further  
 * grasp_point: translation of the grasp position in the local frame of the object  
 * grasp_width: how far the gripper yaws need to close.   
 * mass: the mass of the object in kg  

The objects are loaded into the parameter  
/objects/sub_namespace/parametername  
where the sub_namespace equals the shortname of the object from the rulebook. 
The loading of the objects in the right namespace is done by the launch file *load_objects.launch*
### Conventions: 
 * The x-axis is orientated as the franke_EE with the z-axis in the negative world z-axis, x-axis in the longer principial axis and the y-axis in the shorter principial axis (as the gripper yaws are closing there)  
 * The middle of the bounding box is detected by the image recognition  
 * grasp_point is a vector to move that grasping position in the local frame.  
 * The objects are added to the planning scene according to the image recognition  
 * The x,y position is detected by the YOLO network, while the z-height of the service area (table) is detected using the depth values of the realsense camera. [ToDo!]


