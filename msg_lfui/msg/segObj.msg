# The segmented Objects with position and rotation
sensor_msgs/Image image # the image, contains the header with timestamp and frame
geometry_msgs/Point position # x,y,z coordinates in frame 
float64 angle # around local z-axis
int32 numberOfObjects # specify how many objects were found so that the node can know when it is finished
