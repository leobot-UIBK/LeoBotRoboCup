# The segmented Objects with position and rotation
Header header   # timestamp, sequence number etc.  
string objectType # the name of the identified object
geometry_msgs/Point position # x,y,z coordinates in frame 
float64 angle # around local z-axis
int32 numberOfObjects # specify how many objects were found so that the node can know when it is finished                                                                                                 
