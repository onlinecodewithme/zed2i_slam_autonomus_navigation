# Message for inspection status

std_msgs/Header header

# Current inspection state
uint8 state
uint8 IDLE = 0
uint8 NAVIGATING = 1
uint8 INSPECTING = 2
uint8 RETURNING = 3
uint8 COMPLETED = 4
uint8 ERROR = 5

# Inspection progress (0.0 to 1.0)
float32 progress

# Current position of the robot
geometry_msgs/Pose current_pose

# Current target position (if any)
geometry_msgs/Pose target_pose

# Optional status message
string status_message
