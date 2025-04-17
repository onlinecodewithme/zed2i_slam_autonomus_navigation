# Message for airplane detection

std_msgs/Header header

# Bounding box of the detected airplane in image coordinates
int32 x        # x coordinate of the top-left corner
int32 y        # y coordinate of the top-left corner
int32 width    # width of the bounding box
int32 height   # height of the bounding box

# Detection confidence (0.0 to 1.0)
float32 confidence

# Estimated distance to the airplane in meters
float32 distance

# 3D position relative to the camera
geometry_msgs/Point position

# Class information
string class_id
