# A Pose with reference coordinate frame, timestamp and some infomation for navigation
std_msgs/Header header
geometry_msgs/Pose pose

#target speed for robot (m/s)
float64 speed
uint32 id
uint32 link_id
uint32 safety_id

#Safety padding for robot
float64[] stop_padding #[front, back, left, right]
float64[] slow_padding #[front, back, left, right]
